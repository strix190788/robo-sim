"""odom_error - расхождение между рассчитанной и истинной позой робота.

Подписывается на /wheel_odom и /sim/ground_truth, запоминает последние
позы и раз в секунду публикует расстояние между ними в /odom_error.

Топик /sim/ground_truth есть только в симуляции. Ноду wheel_odometry
запускают со стартовой позой мира (-p start_x:=1.0 -p start_y:=1.0),
иначе число покажет разницу точек отсчёта вместо расхождения поз.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class OdomError(Node):
    def __init__(self):
        super().__init__("odom_error")

        # None означает, что сообщений из этого топика ещё не приходило
        self.odom_xy = None
        self.truth_xy = None

        self.odom_sub = self.create_subscription(
            Odometry, "/wheel_odom", self.on_odom, 10
        )
        self.truth_sub = self.create_subscription(
            Odometry, "/sim/ground_truth", self.on_truth, 10
        )
        self.error_pub = self.create_publisher(Float32, "/odom_error", 10)
        self.timer = self.create_timer(1.0, self.report)

        self.get_logger().info("odom_error запущен, ожидаю обе позы")

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)

    def on_truth(self, msg):
        p = msg.pose.pose.position
        self.truth_xy = (p.x, p.y)

    def report(self):
        if self.odom_xy is None or self.truth_xy is None:
            return

        dx = self.truth_xy[0] - self.odom_xy[0]
        dy = self.truth_xy[1] - self.odom_xy[1]
        error = math.hypot(dx, dy)

        out = Float32()
        out.data = float(error)
        self.error_pub.publish(out)

        self.get_logger().info(f"расхождение {error:.3f} м")


def main():
    rclpy.init()
    node = OdomError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
