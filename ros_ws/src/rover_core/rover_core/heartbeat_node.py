import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HeartbeatNode(Node):
    def __init__(self) -> None:
        super().__init__("heartbeat_node")

        self.publisher = self.create_publisher(
            String,
            "/rover/heartbeat",
            10,
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_heartbeat,
        )

    def publish_heartbeat(self) -> None:
        msg = String()
        msg.data = "rover_core alive"

        self.publisher.publish(msg)

        self.get_logger().info(msg.data)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = HeartbeatNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
