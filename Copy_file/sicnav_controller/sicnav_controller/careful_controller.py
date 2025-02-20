#!/usr/bin/env python3
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav2_pyif.controller import Controller

# Attempt to import a safe navigation utility from SICnav.
# This module should implement a safe control routine.
try:
    from sicnav.safe_nav import SafeNavigator
except ImportError:
    # Placeholder implementation if the module is not available.
    class SafeNavigator:
        def __init__(self, safety_margin=0.3):
            self.safety_margin = safety_margin
        def compute_control(self, current_pose, target_pose, current_velocity):
            # A dummy safe control routine that returns cautious commands.
            # In your real implementation, use sensor data and SICnav routines.
            safe_linear = 0.2  # cautious linear velocity
            safe_angular = 0.0 # no angular adjustment by default
            return safe_linear, safe_angular

class CarefulController(Controller):
    def __init__(self):
        super().__init__()
        self.plan = None
        self.current_target_index = 0
        self.max_speed = 0.4  # Lower max speed for a very careful approach.
        # Initialize the safe navigator with a safety margin (can be tuned).
        self.safe_nav = SafeNavigator(safety_margin=0.3)
        print("CarefulController initialized.")

    def configure(self):
        # Load any additional parameters here if needed.
        self.max_speed = 0.4
        print("CarefulController configured.")

    def set_plan(self, path):
        self.plan = path
        self.current_target_index = 0
        print(f"Plan set with {len(path)} waypoints.")

    def compute_velocity_commands(self, pose, velocity, goal_pose):
        # If no plan is provided or finished, stop the robot.
        if self.plan is None or self.current_target_index >= len(self.plan):
            return self._stop_cmd()

        # Get the current target waypoint.
        target_pose = self.plan[self.current_target_index]
        dx = target_pose.position.x - pose.position.x
        dy = target_pose.position.y - pose.position.y
        distance = math.hypot(dx, dy)

        # If the robot is very close to the waypoint, move to the next one.
        if distance < 0.2:
            self.current_target_index += 1
            if self.current_target_index >= len(self.plan):
                return self._stop_cmd()
            target_pose = self.plan[self.current_target_index]
            dx = target_pose.position.x - pose.position.x
            dy = target_pose.position.y - pose.position.y
            distance = math.hypot(dx, dy)

        # Compute desired heading.
        desired_heading = math.atan2(dy, dx)
        current_yaw = self._get_yaw(pose.orientation)
        heading_error = self._normalize_angle(desired_heading - current_yaw)

        # Use the safe navigator to compute cautious control commands.
        safe_linear, safe_angular = self.safe_nav.compute_control(pose, target_pose, velocity)

        # Combine the safe command with a proportional correction based on distance and heading error.
        linear_vel = np.clip(safe_linear + 0.3 * distance, 0, self.max_speed)
        angular_vel = safe_angular + 1.0 * heading_error

        return self._create_twist(linear_vel, angular_vel)

    def cleanup(self):
        print("Cleaning up CarefulController resources.")

    def shutdown(self):
        print("Shutting down CarefulController.")

    def _stop_cmd(self):
        return self._create_twist(0.0, 0.0)

    def _create_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _get_yaw(self, q):
        # Convert a quaternion (geometry_msgs/Quaternion) to a yaw angle.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    import rclpy
    from rclpy.node import Node
    rclpy.init(args=args)
    node = Node("careful_controller_node")
    controller = CarefulController()
    controller.configure()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    controller.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
