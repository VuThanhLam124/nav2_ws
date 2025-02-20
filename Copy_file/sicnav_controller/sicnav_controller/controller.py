import math
from geometry_msgs.msg import Twist
from nav2_core.controller import Controller as Nav2Controller

class Controller(Nav2Controller):
    def __init__(self):
        super().__init__()
        self.plan = None
        self.current_target_index = 0
        self.max_speed = 0.4
        self.safety_margin = 0.3

    def configure(self, node, name, tf_buffer, costmap_ros):
        self.node = node
        self.name = name
        self.tf_buffer = tf_buffer
        self.costmap_ros = costmap_ros

        self.node.declare_parameter(name + ".max_speed", 0.4)
        self.node.declare_parameter(name + ".safety_margin", 0.3)

        self.max_speed = self.node.get_parameter_or(name + ".max_speed", 0.4).value
        self.safety_margin = self.node.get_parameter_or(name + ".safety_margin", 0.3).value

        self.node.get_logger().info(f"Controller configured with max_speed: {self.max_speed}")

    def set_plan(self, path):
        self.plan = path
        self.current_target_index = 0
        self.node.get_logger().info(f"Plan set with {len(path)} waypoints.")

    def compute_velocity_commands(self, pose, velocity, goal_pose):
        if self.plan is None or self.current_target_index >= len(self.plan):
            return self._stop_cmd()

        target_pose = self.plan[self.current_target_index]
        dx = target_pose.position.x - pose.position.x
        dy = target_pose.position.y - pose.position.y
        distance = math.hypot(dx, dy)

        if distance < 0.2:
            self.current_target_index += 1
            if self.current_target_index >= len(self.plan):
                return self._stop_cmd()

        desired_heading = math.atan2(dy, dx)
        current_yaw = self._get_yaw(pose.orientation)
        heading_error = self._normalize_angle(desired_heading - current_yaw)

        linear_vel = min(self.max_speed, 0.3 * distance)
        angular_vel = heading_error * 1.0

        return self._create_twist(linear_vel, angular_vel)

    def cleanup(self):
        self.node.get_logger().info("Cleaning up Controller.")

    def shutdown(self):
        self.node.get_logger().info("Shutting down Controller.")

    def _stop_cmd(self):
        return self._create_twist(0.0, 0.0)

    def _create_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _get_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
