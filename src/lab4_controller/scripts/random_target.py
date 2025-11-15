#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from interfaces.srv import GetRandomPose, SendArrivalConfirmation
from std_msgs.msg import String

import random
import numpy as np
from math import pi, sqrt, sin, cos

# --- Kinematic Parameters (Must match controller/scheduler/URDF) ---
L1 = 0.20  # Base height (d1)
L2 = 0.25  # Shoulder-elbow length (a2)
L3 = 0.28  # Elbow-end effector length (a3)


class RandomPoseGenerator(Node):
    def __init__(self):
        super().__init__("random_pose_generator")

        # Publisher for the current target position (visualization in RViz)
        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)

        # Service Server to provide a target for Auto Mode
        self.random_pose_server = self.create_service(
            GetRandomPose, "get_random_pose", self.random_pose_service_callback
        )

        # Service Server to receive arrival confirmation (optional)
        self.arrival_server = self.create_service(
            SendArrivalConfirmation,
            "send_arrival_confirmation",
            self.arrival_confirmation_callback,
        )

        # Subscribe to /current_state to know when we are in AM mode
        self.current_state = "IDLE"
        self.state_sub = self.create_subscription(
            String, "/current_state", self.state_callback, 10
        )

        # Current pose & stats
        self.current_random_pose = PoseStamped()
        self.generation_attempts = 0
        self.max_attempts = 100

        # --- Workspace bounds (must match controller/scheduler) ---
        # r_min = |L2 - L3|, r_max = L2 + L3
        self.r_min = abs(L2 - L3)
        self.r_max = L2 + L3

        # Initial dummy target (won't be used until AM mode)
        self._generate_random_pose()

        # Timer to periodically publish the current target (for RViz)
        self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("Random Pose Generator Node has been started.")
        self.get_logger().info(
            f"Workspace sphere: center z={L1:.3f}, "
            f"r_min={self.r_min:.3f}, r_max={self.r_max:.3f}"
        )

    # ======================
    #  Callbacks
    # ======================

    def state_callback(self, msg: String):
        """Subscribe to /current_state from scheduler."""
        self.current_state = msg.data
        # Use debug to avoid spam
        self.get_logger().debug(f"[RandomNode] current_state = {self.current_state}")

    def random_pose_service_callback(
        self,
        request: GetRandomPose.Request,
        response: GetRandomPose.Response,
    ):
        """
        Service callback for Auto Mode.
        Only responds with a new random pose when state == 'AM'.
        """
        if self.current_state != "AM":
            self.get_logger().warn(
                f"Random pose requested but current_state = '{self.current_state}', "
                "not 'AM'. No new target will be generated."
            )
            response.success = False
            # Optionally still return the last pose
            response.target_pose = self.current_random_pose
            return response

        self.get_logger().info("Received request for a new random pose in AM mode.")

        # 1. Generate a new random pose
        new_pose = self._generate_random_pose()

        # 2. Publish immediately (for RViz visualization)
        self.target_pub.publish(new_pose)

        # 3. Fill the Service Response
        response.target_pose = new_pose
        response.success = True

        self.get_logger().info(
            f"New target generated in {self.generation_attempts} attempts: "
            f"({new_pose.pose.position.x:.3f}, "
            f"{new_pose.pose.position.y:.3f}, "
            f"{new_pose.pose.position.z:.3f})"
        )

        return response

    def arrival_confirmation_callback(
        self,
        request: SendArrivalConfirmation.Request,
        response: SendArrivalConfirmation.Response,
    ):
        """Optional callback for controller arrival confirmation."""
        if request.arrived_at_target:
            self.get_logger().info("Robot confirmed arrival at target.")
        response.acknowledged = True
        return response

    def timer_callback(self):
        """
        Periodically publishes the current target pose,
        but ONLY when the state is AM.
        """
        if self.current_state == "AM":
            # Only publish when we are in Auto Mode
            if self.current_random_pose.header.stamp.sec != 0:
                self.current_random_pose.header.stamp = self.get_clock().now().to_msg()
                self.target_pub.publish(self.current_random_pose)
        else:
            # Not in AM → do nothing
            pass

    # ======================
    #  Workspace & Sampling
    # ======================

    def _is_in_workspace(self, x: float, y: float, z: float) -> bool:
        """
        Same workspace check as controller/scheduler:

        distance_squared = x^2 + y^2 + (z - L1)^2
        r_min <= distance <= r_max
        """
        distance_squared = x**2 + y**2 + (z - L1) ** 2
        distance = np.sqrt(distance_squared)

        if distance_squared < self.r_min**2:
            self.get_logger().debug(
                f"Target too close: distance={distance:.3f} < r_min={self.r_min:.3f}"
            )
            return False

        if distance_squared > self.r_max**2:
            self.get_logger().debug(
                f"Target too far: distance={distance:.3f} > r_max={self.r_max:.3f}"
            )
            return False

        # Optional: avoid near-axis singularity
        R = np.sqrt(x**2 + y**2)
        if R < 0.05:
            self.get_logger().debug(
                f"Target too close to Z-axis (R={R:.3f} < 0.05), reject."
            )
            return False

        return True

    def _generate_random_pose(self) -> PoseStamped:
        """
        Generates a random (X, Y, Z) position within the spherical workspace.

        1. Sample distance d in [r_min, r_max]
        2. Sample angles theta, phi
        3. Convert to Cartesian relative to shoulder
        4. Shift by L1 in z
        5. Reject until _is_in_workspace() is satisfied
        """
        x = y = z = 0.0
        is_valid = False
        attempts = 0

        while not is_valid and attempts < self.max_attempts:
            attempts += 1

            # Distance from shoulder
            d = sqrt(random.uniform(self.r_min**2, self.r_max**2))

            # Theta: angle from +Z axis (0..pi)
            theta = random.uniform(0.0, pi)
            # Phi: azimuth around Z axis (-pi..pi)
            phi = random.uniform(-pi, pi)

            # Convert spherical -> Cartesian (shoulder frame)
            R = d * sin(theta)
            z_rel = d * cos(theta)

            x = R * cos(phi)
            y = R * sin(phi)
            z = z_rel + L1

            if self._is_in_workspace(x, y, z):
                is_valid = True

        if not is_valid:
            self.get_logger().warn(
                f"Failed to generate valid point after {attempts} attempts. "
                "Using safe default (0.3, 0.2, 0.3)."
            )
            x, y, z = 0.3, 0.2, 0.3

        # Build PoseStamped
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # IMPORTANT: match controller frame
        msg.header.frame_id = "link_0"

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.current_random_pose = msg
        self.generation_attempts = attempts
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = RandomPoseGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Random Pose Generator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
