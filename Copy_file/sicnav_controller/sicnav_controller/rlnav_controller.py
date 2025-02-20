#!/usr/bin/env python3
import math
import numpy as np
import torch

from geometry_msgs.msg import Twist
from nav2_pyif.controller import Controller

class RLNavController(Controller):
    def __init__(self):
        super().__init__()
        self.plan = None
        self.current_target_index = 0
        self.max_speed = 0.5
        self.model_path = None
        self.policy = None  # The RL policy (a PyTorch model)
        print("RLNavController initialized.")

    def configure(self):
        # Load parameters (in a full implementation, these may be loaded from ROS parameters)
        self.max_speed = 0.5
        self.model_path = "/path/to/pretrained_rl_model.pth"  # Update this path accordingly

        # Load the pre-trained RL model
        try:
            self.policy = torch.load(self.model_path, map_location=torch.device("cpu"))
            self.policy.eval()
            print("RL model loaded successfully from:", self.model_path)
        except Exception as e:
            print("Error loading RL model:", e)
            self.policy = None

    def set_plan(self, path):
        """
        Store the navigation plan as a list of waypoints.
        """
        self.plan = path
        self.current_target_index = 0
        print("Plan set with {} waypoints".format(len(path)))

    def compute_velocity_commands(self, pose, velocity, goal_pose):
        """
        Compute velocity commands using a pre-trained RL policy.
        The RL model should take the current state (e.g., robot pose, current velocity)
        and output an action [linear_velocity, angular_velocity].
        """
        if self.policy is None:
            print("RL model not loaded. Stopping.")
            return self._stop_cmd()

        # Build a state representation.
        # Here we use a simple state: [x, y, linear_vel, angular_vel]
        state = np.array([
            pose.position.x,
            pose.position.y,
            velocity.linear.x,
            velocity.angular.z
        ], dtype=np.float32)
        state_tensor = torch.tensor(state).unsqueeze(0)  # Add batch dimension

        with torch.no_grad():
            action_tensor = self.policy(state_tensor)
        action = action_tensor.squeeze(0).numpy()

        # Interpret the RL model output
        linear_vel = float(np.clip(action[0], 0, self.max_speed))
        angular_vel = float(action[1])

        # Optionally, you can include additional logic (e.g., if approaching a waypoint, adjust the command)
        return self._create_twist(linear_vel, angular_vel)

    def cleanup(self):
        print("Cleaning up RLNavController resources.")

    def shutdown(self):
        print("Shutting down RLNavController.")

    def _stop_cmd(self):
        return self._create_twist(0.0, 0.0)

    def _create_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

def main(args=None):
    import rclpy
    from rclpy.node import Node
    rclpy.init(args=args)
    node = Node("rlnav_controller_node")
    controller = RLNavController()
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
