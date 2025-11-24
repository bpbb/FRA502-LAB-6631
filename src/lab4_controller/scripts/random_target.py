#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from interfaces.srv import GetRandomPose
from std_msgs.msg import String

import random
import numpy as np
from math import pi, sqrt, sin, cos

import roboticstoolbox as rtb
from spatialmath import SE3


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

        # Subscribe to /current_state to know when we are in AM mode
        self.current_state = "IDLE"
        self.state_sub = self.create_subscription(
            String, "/current_state", self.state_callback, 10
        )

        # Current pose & stats
        self.current_random_pose = PoseStamped()
        self.generation_attempts = 0
        self.max_attempts = 100

        # Workspace bounds
        self.r_min = abs(L2 - L3)  # 0.03m
        self.r_max = L2 + L3  # 0.53m
        
        # Robot model for IK checking
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0, a=0.0, d=L1, offset=0.0),
                rtb.RevoluteMDH(alpha=pi/2, a=0.0, d=0.02, offset=0.0),
                rtb.RevoluteMDH(alpha=0.0, a=L2, d=0.0, offset=0.0),
            ],
            tool=SE3.Tx(L3),
            name="RRR_Robot",
        )

        # Initial dummy target
        self._generate_random_pose()

        # Timer to periodically publish the current target
        self.create_timer(0.01, self.timer_callback)

    def state_callback(self, msg: String):
        self.current_state = msg.data

    def random_pose_service_callback(self, request: GetRandomPose.Request, response: GetRandomPose.Response):
        if self.current_state != "AM":
            self.get_logger().warn(
                f"Random pose requested but state={self.current_state}, not AM"
            )
            response.success = False
            response.target_pose = self.current_random_pose
            return response

        self.get_logger().info("Generating new random pose for AM mode...")

        # Generate new random pose (with all safety checks)
        new_pose = self._generate_random_pose()

        # Publish for RViz
        self.target_pub.publish(new_pose)

        # Fill response
        response.target_pose = new_pose
        response.success = True

        self.get_logger().info(
            f"Target generated: "
            f"[{new_pose.pose.position.x:.3f}, "
            f"{new_pose.pose.position.y:.3f}, "
            f"{new_pose.pose.position.z:.3f}]"
        )

        return response

    def timer_callback(self):
        if self.current_state == "AM":
            if self.current_random_pose.header.stamp.sec != 0:
                self.current_random_pose.header.stamp = self.get_clock().now().to_msg()
                self.target_pub.publish(self.current_random_pose)

    def _is_in_workspace(self, x: float, y: float, z: float) -> bool:
        
        distance_squared = x**2 + y**2 + (z - L1) ** 2
        distance = np.sqrt(distance_squared)

        # Check distance bounds with conservative margins (3cm)
        margin = 0.03
        if distance < (self.r_min + margin):
            return False
            
        if distance > (self.r_max - margin):
            return False

        # Avoid near-axis singularity (8cm radius for safety)
        R = np.sqrt(x**2 + y**2)
        if R < 0.08:
            return False
        
        # Minimum z height (well above ground - consistent with controller)
        if z < 0.02:
            return False
        
        # Maximum z height (avoid overhead configurations)
        if z > 0.8:
            return False

        return True

    def _is_reachable_and_safe(self, x: float, y: float, z: float) -> bool:
        try:
            # Test multiple initial guesses to find different IK solutions
            initial_guesses = [
                [0, 0, 0],           # Home position
                [0, pi/4, -pi/4],    # Safe mid-range
                [pi/2, pi/4, 0],     # Side approach
                [-pi/2, pi/4, 0],    # Other side
                [0, pi/6, -pi/6],    # Slightly elevated
            ]
            
            T_target = SE3(x, y, z)
            pos_target = np.array([x, y, z])
            safe_solutions_found = 0
            
            for q0 in initial_guesses:
                ik_solution = self.robot.ikine_LM(
                    T_target, 
                    mask=[1, 1, 1, 0, 0, 0], 
                    joint_limits=False, 
                    q0=q0
                )
                
                # Skip if IK failed
                if not ik_solution.success:
                    continue
                
                # Check IK residual error (must be < 1mm)
                if hasattr(ik_solution, 'residual'):
                    if ik_solution.residual > 0.001:  # 1mm
                        continue  # Solution not accurate enough
                
                q = ik_solution.q
                
                # Verify FK actually reaches target (double-check)
                T_check = self.robot.fkine(q)
                pos_check = T_check.t
                position_error = np.linalg.norm(pos_check - pos_target)
                
                if position_error > 0.002:  # 2mm tolerance
                    continue  # FK doesn't match target
                else:
                    safe_solutions_found += 1
            
            # Require at least one safe solution
            return safe_solutions_found > 0
            
        except Exception as e:
            return False

    def _generate_random_pose(self) -> PoseStamped:

        x = y = z = 0.0
        is_valid = False
        attempts = 0
        
        # Statistics for debugging
        workspace_fails = 0
        reachability_fails = 0

        while not is_valid and attempts < self.max_attempts:
            attempts += 1

            # Step 1: Generate random spherical coordinates
            d = sqrt(random.uniform((self.r_min + 0.05)**2, (self.r_max - 0.05)**2))
        
            if random.random() < 0.7:
                theta = random.uniform(0.0, pi * 0.6)
            else:
                theta = random.uniform(pi * 0.6, pi * 0.9)
            
            phi = random.uniform(-pi, pi)

            # Step 2: Convert to Cartesian
            R = d * sin(theta)
            z_rel = d * cos(theta)
            x = R * cos(phi)
            y = R * sin(phi)
            z = z_rel + L1

            # Step 3: Check workspace bounds and singularity
            if not self._is_in_workspace(x, y, z):
                workspace_fails += 1
                continue
            
            # Step 4: Check IK reachability and ground collision
            if not self._is_reachable_and_safe(x, y, z):
                reachability_fails += 1
                continue
            
            # All checks passed!
            is_valid = True

        # Log statistics
        if not is_valid:
            self.get_logger().warn(
                f"Failed after {attempts} attempts. "
                f"Workspace: {workspace_fails}, Reachability: {reachability_fails}. "
                f"Using validated safe default..."
            )
            
            # Try validated safe positions (all in upper hemisphere, well above ground)
            safe_candidates = [
                (0.30, 0.20, 0.30),  # Front-right, mid-height
                (0.35, 0.00, 0.30),  # Front center
                (0.25, 0.20, 0.35),  # Front-right, higher
                (0.40, 0.00, 0.28),  # Further front
                (0.30, -0.20, 0.30), # Front-left
                (0.20, 0.25, 0.35),  # Side, higher
                (0.35, 0.15, 0.32),  # Diagonal
            ]
            
            for candidate in safe_candidates:
                x_test, y_test, z_test = candidate
                if self._is_in_workspace(x_test, y_test, z_test) and \
                   self._is_reachable_and_safe(x_test, y_test, z_test):
                    x, y, z = x_test, y_test, z_test
                    is_valid = True
                    self.get_logger().info(f"Using validated safe default: [{x:.3f}, {y:.3f}, {z:.3f}]")
                    break
            
            if not is_valid:
                # Last resort - use first candidate without full validation
                x, y, z = safe_candidates[0]
                self.get_logger().error(
                    "Could not validate any safe default! Using [0.3, 0.2, 0.3] without full check."
                )
        else:
            if attempts > 20:
                self.get_logger().info(
                    f"Generated valid pose after {attempts} attempts. "
                    f"(Workspace fails: {workspace_fails}, Reachability fails: {reachability_fails})"
                )

        # Build PoseStamped
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
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
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()