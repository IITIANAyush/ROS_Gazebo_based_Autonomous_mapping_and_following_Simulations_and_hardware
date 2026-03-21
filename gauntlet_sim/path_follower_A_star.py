#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        self.current_idx = 0
        self.path = None
        self.pose = None
        self.scan = None

        # ===== PARAMETERS =====
        self.LOOKAHEAD_DIST = 0.20   # Reduced from 0.45 — stays tight at corners.
                                     # 0.45 caused the lookahead to jump across bends,
                                     # making the robot swing wide into walls.

        self.GOAL_TOL = 0.15

        # Safety zones — kept as they were (correct values).
        self.STOP_DIST = 0.30   # Zone 3: hard stop
        self.SLOW_DIST = 0.50   # Zone 2: crawl
        self.WARN_DIST = 0.60   # Zone 1: slow down

        # PID gains             # used in controlling omega as its the main sourse of error in path following 
        self.Kp = 1.0
        self.Ki = 0.02   # kept small — integral winds up on long straights
        self.Kd = 0.15   # raised from 0.08/0.1 — more damping prevents oscillation
                         # at corners where Kp drives a large sudden correction

        self.integral   = 0.0
        self.prev_error = 0.0
        self.prev_time  = None   # initialised on first real tick, not here

        # ===== QoS =====
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # ===== Subscribers =====
        # Subscribes to /planned_path_astar (soft-penalty A* path).
        # Change to /planned_path_gvd for the hard-clearance GVD path.
        self.path_sub = self.create_subscription(
            Path,      '/planned_path_astar', self.path_callback, path_qos)
        self.odom_sub = self.create_subscription(
            Odometry,  '/odom',               self.odom_callback, odom_qos)
        # FIX: TurtleBot lidar publishes with BEST_EFFORT reliability.
        # Bare depth=10 defaults to RELIABLE — QoS mismatch means the
        # subscriber receives nothing and self.scan stays None forever.
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos)

        # ===== Publisher =====
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Path Follower Initialized.")

    # ================= CALLBACKS =================

    def path_callback(self, msg):
        self.path = msg.poses
        self.current_idx = 0
        self.get_logger().info(f"Path received: {len(self.path)} waypoints")

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def scan_callback(self, msg):
        self.scan = msg

    # ================= CONTROL LOOP =================

    def control_loop(self):

        # ---- Guard: wait for required data ----
        if self.path is None:
            self.get_logger().warn("Waiting for path...", throttle_duration_sec=2.0)
            return
        if self.pose is None:
            self.get_logger().warn("Waiting for odom...", throttle_duration_sec=2.0)
            return

        # Initialise prev_time on the first real tick so dt is valid next tick.
        if self.prev_time is None:
            self.prev_time = self.get_clock().now()
            return

        rx = self.pose.position.x
        ry = self.pose.position.y

        # ===== Goal Check =====
        goal_x = self.path[-1].pose.position.x
        goal_y = self.path[-1].pose.position.y
        dist_to_goal = math.hypot(goal_x - rx, goal_y - ry)

        if dist_to_goal < self.GOAL_TOL:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("GOAL REACHED")
            return

        # ===== 3-Zone Obstacle Check =====
        #
        # BUG FIX 1 (revised): The previous fix defaulted front_min=0.0 when
        # scan was None, which caused a permanent HARD STOP at 0.00m once the
        # scan arrived — because many lidar drivers return r=0.0 for invalid
        # rays (chassis occlusion, out-of-range) instead of nan/inf.
        # The old filter `not math.isfinite(r)` passed 0.0 straight through.
        #
        # Two-part fix:
        #   (a) When scan is None → hold position by returning early (no cmd_vel).
        #   (b) When scan is present → reject any ray where r <= range_min OR
        #       r >= range_max. range_min is provided in the LaserScan header
        #       and is typically 0.12–0.16 m for a TurtleBot lidar. Any reading
        #       at or below that is the robot seeing its own chassis — not a
        #       real obstacle.
        #
        if self.scan is None:
            # No lidar data yet — publish zero velocity and wait.
            self.get_logger().warn(
                "No scan received yet — holding position.",
                throttle_duration_sec=2.0
            )
            self.cmd_pub.publish(Twist())
            return

        front_min = float('inf')
        left_min  = float('inf')
        right_min = float('inf')

        # This lidar reports range_min=0.0 and range_max=100.0 in the header,
        # so using those header values as validity bounds is useless.
        # Use a hardcoded physical floor of 0.05 m instead — anything closer
        # is the robot chassis or a glitch, not a real obstacle.
        RANGE_FLOOR = 0.05

        angle_min = self.scan.angle_min
        angle_inc = self.scan.angle_increment
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or r < RANGE_FLOOR:
                continue
            angle = angle_min + i * angle_inc
            if abs(angle) < math.radians(30):
                front_min = min(front_min, r)
            elif math.radians(30) <= angle < math.radians(90):
                left_min = min(left_min, r)
            elif math.radians(-90) < angle <= math.radians(-30):
                right_min = min(right_min, r)

        # Zone 3: hard stop — obstacle too close, stop completely.
        if front_min < self.STOP_DIST:
            self.get_logger().warn(
                f"HARD STOP — obstacle at {front_min:.2f}m",
                throttle_duration_sec=0.5
            )
            self.cmd_pub.publish(Twist())
            return

        # Zone 1 & 2: ramp speed down as obstacle approaches.
        if front_min < self.WARN_DIST:
            t = max(0.0, (front_min - self.SLOW_DIST) / (self.WARN_DIST - self.SLOW_DIST))
            obstacle_speed_limit = 0.03 + t * (0.8 - 0.03)
        else:
            obstacle_speed_limit = 0.8

        # Side wall repulsion — steer away if either side is dangerously close.
        wall_steer = 0.0
        SIDE_WARN = 0.30
        if left_min < SIDE_WARN:
            wall_steer -= 0.3   # push right
        if right_min < SIDE_WARN:
            wall_steer += 0.3   # push left

        # ===== Find Closest Path Point =====
        # Scan forward from current_idx only — never look backwards.
        # Prevents the robot from re-targeting already-passed waypoints.
        min_dist    = float('inf')
        closest_idx = self.current_idx
        for i in range(self.current_idx, len(self.path)):
            px = self.path[i].pose.position.x
            py = self.path[i].pose.position.y
            d  = math.hypot(px - rx, py - ry)
            if d < min_dist:
                min_dist    = d
                closest_idx = i
        self.current_idx = closest_idx

        # ===== Lookahead Selection =====
        # Walk forward along the path until a point is further than
        # LOOKAHEAD_DIST away — that becomes the steering target.
        lookahead_idx = self.current_idx
        while lookahead_idx < len(self.path):
            px = self.path[lookahead_idx].pose.position.x
            py = self.path[lookahead_idx].pose.position.y
            if math.hypot(px - rx, py - ry) > self.LOOKAHEAD_DIST:
                break
            lookahead_idx += 1
        if lookahead_idx >= len(self.path):
            lookahead_idx = len(self.path) - 1

        tx = self.path[lookahead_idx].pose.position.x
        ty = self.path[lookahead_idx].pose.position.y

        # ===== Heading Error =====
        angle_to_target = math.atan2(ty - ry, tx - rx)

        q = self.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw       = math.atan2(siny_cosp, cosy_cosp)

        # Wrap error to [-π, π]
        error = math.atan2(
            math.sin(angle_to_target - yaw),
            math.cos(angle_to_target - yaw)
        )

        # ===== PID =====
        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        if dt <= 0:
            return

        self.integral  += error * dt
        derivative      = (error - self.prev_error) / dt
        self.prev_error = error

        omega = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # ===== Velocity Command =====
        #
        # BUG FIX 2: MAX_ANG was set to 0.07 rad/s — that is ~4°/second.
        # The robot physically could not rotate fast enough to steer away from
        # anything.  PID computed a large omega but the clip crushed it to 0.07.
        # Restored to 0.9 rad/s — sufficient for sharp in-place correction.
        #
        # BUG FIX 3: The error threshold for stopping forward motion was 0.3 rad
        # (~17°).  This caused the robot to halt and spin for any minor heading
        # deviation, and combined with the tiny MAX_ANG it would sit spinning at
        # 0.07 rad/s indefinitely.  Restored to 0.8 rad (~46°) — only stops
        # linear motion when severely misaligned.
        #
        MAX_LIN = 2   # FIX: was 0.08 — too slow to drive meaningfully
        MAX_ANG = 0.9    # FIX: was 0.07 — robot could not steer at all

        cmd = Twist()

        if abs(error) > 0.8:        # FIX: threshold was 0.3 (~17°), now 0.8 (~46°)
            cmd.linear.x = 0.0      # severely misaligned — rotate in place
        else:
            cmd.linear.x = min(obstacle_speed_limit, dist_to_goal, MAX_LIN)

        cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, omega + wall_steer))

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"Dist:{dist_to_goal:.2f} | Err:{math.degrees(error):.1f}deg "
            f"| Spd:{cmd.linear.x:.2f} | Ang:{cmd.angular.z:.2f} "
            f"| F:{front_min:.2f} L:{left_min:.2f} R:{right_min:.2f}",
            throttle_duration_sec=1.0
        )


def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
