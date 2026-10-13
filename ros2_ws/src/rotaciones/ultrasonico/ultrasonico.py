import rclpy
from rclpy.node import Node


class UltraSonico(Node):

    def __init__(self):
        super().__init__('ultrasonico')

        self.get_logger().info('Nodo del sensor ultrasónico iniciado')


def main(args=None):
    rclpy.init(args=args)

    node = UltraSonico()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()