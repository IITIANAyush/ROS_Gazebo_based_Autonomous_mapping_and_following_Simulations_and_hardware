#!/usr/bin/env python3

import math
import heapq
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

# 8-connected grid
NEIGHBORS = [
    (-1,  0, 1.0), (1,  0, 1.0),
    (0, -1, 1.0), (0,  1, 1.0),
    (-1, -1, 1.414), (-1,  1, 1.414),
    (1, -1, 1.414),  (1,  1, 1.414),
]

class PlannerServer(Node):

    def __init__(self):
        super().__init__('planner_server')
        
        # Internal state
        self.map_received = False
        self.map_info = None
        self.grid = None
        self.planned_once = False
        self.latest_path_astar = None  # Storage for republishing
        self.latest_path_gvd = None

        # QoS for static map
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscribers / Publishers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos
        )

        self.path_pub_astar = self.create_publisher(
            Path, '/planned_path_astar', 10
        )

        self.path_pub_gvd = self.create_publisher(
            Path, '/planned_path_gvd', 10
        )

        # Start & Goal (WORLD COORDINATES)
        self.world_goal = (0.0, 0.0) # get positioon using 2d pos estimate in rviz2
        self.world_start  = ( 0.37,  -1.11) # same 

        # Timers
        # Timer 1: Main planning logic (checks every second until path is found)
        self.planner_timer = self.create_timer(1.0, self.plan_once)
        
        # Timer 2: Heartbeat (keeps the path alive for the follower)
        self.republish_timer = self.create_timer(1.0, self.republish_callback)

        self.get_logger().info("Planner Server started. Waiting for map...")

    def republish_callback(self):
        """Broadcasts the latest calculated paths so late-joining nodes receive them."""
        now = self.get_clock().now().to_msg()
        
        if self.latest_path_astar is not None:
            self.latest_path_astar.header.stamp = now
            self.path_pub_astar.publish(self.latest_path_astar)
            
        if self.latest_path_gvd is not None:
            self.latest_path_gvd.header.stamp = now
            self.path_pub_gvd.publish(self.latest_path_gvd)

    def map_callback(self, msg):
        self.map_info = msg.info
        self.grid = msg.data
        self.map_received = True
        self.get_logger().info(f"Map received: {msg.info.width} x {msg.info.height}")

    def world_to_grid(self, wx, wy):
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        wx = gx * res + ox + res / 2.0
        wy = gy * res + oy + res / 2.0
        return (wx, wy)

    def is_free(self, gx, gy):
        w = self.map_info.width
        h = self.map_info.height
        if 0 <= gx < w and 0 <= gy < h:
            val = self.grid[gy * w + gx]
            return 0 <= val < 65 # 0-64 is free, 65-100 is occupied, -1 is unknown
        return False

    def plan_once(self):
        if not self.map_received or self.planned_once:
            return

        start = self.world_to_grid(*self.world_start)
        goal  = self.world_to_grid(*self.world_goal)

        self.get_logger().info(f"Attempting plan: {start} -> {goal}")

        if not self.is_free(*start) or not self.is_free(*goal):
            self.get_logger().error("Start or Goal is occupied! Check coordinates or map.")
            return

        # A* Planning
        astar_cells = self.astar(start, goal)
        if astar_cells:
            self.latest_path_astar = self.create_path_msg(astar_cells)
            self.path_pub_astar.publish(self.latest_path_astar)
            self.get_logger().info("A* path generated.")

        # GVD Planning
        clearance = self.compute_brushfire()
        gvd_cells = self.astar_with_clearance(start, goal, clearance)
        if gvd_cells:
            self.latest_path_gvd = self.create_path_msg(gvd_cells)
            self.path_pub_gvd.publish(self.latest_path_gvd)
            self.get_logger().info("GVD path generated.")

        if astar_cells or gvd_cells:
            self.planned_once = True

    def create_path_msg(self, path_cells):
        """Helper to convert grid cells to a NavMsgs Path."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for gx, gy in path_cells:
            wx, wy = self.grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def astar(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_cost = {start: 0}

        while open_list:
            _, current = heapq.heappop(open_list)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy, cost in NEIGHBORS:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.is_free(*neighbor):
                    continue
                tentative_g = g_cost[current] + cost
                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f, neighbor))
                    came_from[neighbor] = current
        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def compute_brushfire(self):
        w, h = self.map_info.width, self.map_info.height
        dist = [[float('inf')] * w for _ in range(h)]
        q = deque()
        for y in range(h):
            for x in range(w):
                if self.grid[y * w + x] >= 65:
                    dist[y][x] = 0
                    q.append((x, y))
        while q:
            x, y = q.popleft()
            for dx, dy, _ in NEIGHBORS:
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if dist[ny][nx] > dist[y][x] + 1:
                        dist[ny][nx] = dist[y][x] + 1
                        q.append((nx, ny))
        return dist

    def astar_with_clearance(self, start, goal, clearance):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from, g_cost = {}, {start: 0}

        while open_list:
            _, current = heapq.heappop(open_list)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy, step_cost in NEIGHBORS:
                nx, ny = current[0] + dx, current[1] + dy
                if not self.is_free(nx, ny):
                    continue
                
                # Penalty for being close to walls
                safety_penalty = 10.0 / (clearance[ny][nx] + 0.1) 
                tentative_g = g_cost[current] + step_cost + safety_penalty

                if (nx, ny) not in g_cost or tentative_g < g_cost[(nx, ny)]:
                    g_cost[(nx, ny)] = tentative_g
                    f = tentative_g + self.heuristic((nx, ny), goal)
                    heapq.heappush(open_list, (f, (nx, ny)))
                    came_from[(nx, ny)] = current
        return None

def main():
    rclpy.init()
    node = PlannerServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
