#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import yaml
import sys

def load_waypoints(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
        return data['waypoints']

def quaternion_to_yaw(q):
    # ROS quaternion to yaw (in radians)
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class TrajectoryFollower(Node):
    def __init__(self, waypoints):
        super().__init__('trajectory_follower')
        self.cmd_pub = self.create_publisher(Twist, '/robot/robotnik_base_controller/cmd_vel', 10)
        self.gt_sub = self.create_subscription(Odometry, '/robot/ground_truth', self.gt_callback, 10)
        self.waypoints = waypoints
        self.current_idx = 0
        self.goal_reached = False
        self.ground_truth_ready = False  # Bandera

        self.declare_parameter('goal_tolerance_xy', 0.06)
        self.declare_parameter('goal_tolerance_yaw', 0.1)
        self.tol_xy = self.get_parameter('goal_tolerance_xy').get_parameter_value().double_value
        self.tol_yaw = self.get_parameter('goal_tolerance_yaw').get_parameter_value().double_value

        self.get_logger().info(f"{len(waypoints)} waypoints loaded.")

    def gt_callback(self, msg):
        if not self.ground_truth_ready:
            self.get_logger().info("¡Recibido primer ground truth! Empezando trayectoria.")
            self.ground_truth_ready = True

        if not self.ground_truth_ready or self.goal_reached or self.current_idx >= len(self.waypoints):
            self.cmd_pub.publish(Twist())  # Stop
            return

        wp = self.waypoints[self.current_idx]
        # Ground truth pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q)

        dx = wp['x'] - x
        dy = wp['y'] - y
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(angle_to_goal - yaw), math.cos(angle_to_goal - yaw))
        yaw_diff = math.atan2(math.sin(wp['yaw'] - yaw), math.cos(wp['yaw'] - yaw))

        twist = Twist()
        if dist > self.tol_xy:
            # First, rotate towards goal, then go straight
            if abs(angle_diff) > 0.2:
                twist.angular.z = 0.8 * angle_diff
            else:
                twist.linear.x = min(0.3, dist)
                twist.angular.z = 0.7 * angle_diff
        elif abs(yaw_diff) > self.tol_yaw:
            # If reached (x,y), align yaw
            twist.angular.z = 0.7 * yaw_diff
        else:
            self.get_logger().info(f"Reached waypoint {self.current_idx+1}/{len(self.waypoints)}")
            self.current_idx += 1
            if self.current_idx >= len(self.waypoints):
                self.get_logger().info("Trayectoria completada.")
                self.goal_reached = True
                rclpy.shutdown()
                return
            twist = Twist()  # Stop

        self.cmd_pub.publish(twist)

def main():
    if len(sys.argv) < 2:
        print("Uso: trajectory_follower.py waypoints.yaml")
        return
    waypoints = load_waypoints(sys.argv[1])
    rclpy.init()
    node = TrajectoryFollower(waypoints)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

