#!/usr/bin/env python3
"""
action_to_cmdvel_node.py
Converte le azioni discrete di NaVILA in comandi di velocità (geometry_msgs/Twist)
pubblicati su /cmd_vel.

Subscribes:
    /navila/action  (std_msgs/String)  — "forward" | "left" | "right" | "stop"

Publishes:
    /cmd_vel        (geometry_msgs/Twist)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# Valori di default per le velocità (modificabili via parametri ROS 2)
DEFAULT_LINEAR_X  = 0.3   # m/s — velocità avanti
DEFAULT_ANGULAR_Z = 0.5   # rad/s — velocità di rotazione


class ActionToCmdVelNode(Node):

    def __init__(self):
        super().__init__("action_to_cmdvel_node")

        # ------------------------------------------------------------------
        # Parametri ROS 2
        # ------------------------------------------------------------------
        self.declare_parameter("linear_x",  DEFAULT_LINEAR_X)
        self.declare_parameter("angular_z", DEFAULT_ANGULAR_Z)
        self.declare_parameter("action_topic", "/navila/action")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.linear_x  = self.get_parameter("linear_x").value
        self.angular_z = self.get_parameter("angular_z").value
        action_topic   = self.get_parameter("action_topic").value
        cmd_vel_topic  = self.get_parameter("cmd_vel_topic").value

        # ------------------------------------------------------------------
        # Subscriber / Publisher
        # ------------------------------------------------------------------
        self.sub_action = self.create_subscription(
            String, action_topic, self._action_cb, 10)

        self.pub_cmd_vel = self.create_publisher(
            Twist, cmd_vel_topic, 10)

        self.get_logger().info(
            f"action_to_cmdvel_node ready | "
            f"linear_x={self.linear_x} m/s | "
            f"angular_z={self.angular_z} rad/s"
        )

    def _action_cb(self, msg: String):
        action = msg.data.strip().lower()
        twist  = Twist()

        if action == "forward":
            twist.linear.x  =  self.linear_x
            twist.angular.z =  0.0

        elif action == "left":
            twist.linear.x  =  0.0
            twist.angular.z =  self.angular_z

        elif action == "right":
            twist.linear.x  =  0.0
            twist.angular.z = -self.angular_z

        elif action == "stop":
            twist.linear.x  =  0.0
            twist.angular.z =  0.0

        else:
            self.get_logger().warn(f"Azione non riconosciuta: '{action}' — stop")
            twist.linear.x  = 0.0
            twist.angular.z = 0.0

        self.get_logger().info(
            f"Action: {action:8s} → linear.x={twist.linear.x:.2f}  "
            f"angular.z={twist.angular.z:.2f}"
        )
        self.pub_cmd_vel.publish(twist)


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ActionToCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Pubblica stop prima di uscire
        stop = Twist()
        node.pub_cmd_vel.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()