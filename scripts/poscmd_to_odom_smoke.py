#!/usr/bin/env python3
"""Runtime smoke test for the native ROS 2 PositionCommand adapter."""

import math
import time

import rclpy
from falcon_interfaces.msg import PositionCommand
from nav_msgs.msg import Odometry


EXPECTED_POSITION = (1.25, -2.5, 3.75)
EXPECTED_VELOCITY = (0.4, 0.2, 0.1)


def close(actual: float, expected: float) -> bool:
    return math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9)


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("poscmd_to_odom_smoke")
    publisher = node.create_publisher(PositionCommand, "/command", 1)
    matched = False

    def receive_odometry(message: Odometry) -> None:
        nonlocal matched
        position = message.pose.pose.position
        velocity = message.twist.twist.linear
        matched = (
            close(position.x, EXPECTED_POSITION[0])
            and close(position.y, EXPECTED_POSITION[1])
            and close(position.z, EXPECTED_POSITION[2])
            and close(velocity.x, EXPECTED_VELOCITY[0])
            and close(velocity.y, EXPECTED_VELOCITY[1])
            and close(velocity.z, EXPECTED_VELOCITY[2])
        )

    subscription = node.create_subscription(Odometry, "/odometry", receive_odometry, 1)

    command = PositionCommand()
    command.header.stamp = node.get_clock().now().to_msg()
    command.header.frame_id = "world"
    command.position.x, command.position.y, command.position.z = EXPECTED_POSITION
    command.velocity.x, command.velocity.y, command.velocity.z = EXPECTED_VELOCITY
    command.yaw = 0.0

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not matched:
        publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()

    if not matched:
        raise RuntimeError("No matching odometry message was received")

    print("PositionCommand -> Odometry integration test passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
