import rclpy
from rclpy.node import Node


class SkeletonNode(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_follower_node")
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        self.get_logger().debug("waypoint_follower_node alive")


def main() -> None:
    rclpy.init()
    node = SkeletonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
