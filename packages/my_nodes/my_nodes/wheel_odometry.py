"""wheel_odometry - счисление пути по скоростям колёс.

Читает /hardware/status, берёт измеренные энкодерами скорости колёс,
пересчитывает их в скорости корпуса и накапливает позу робота.
Публикует nav_msgs/Odometry в свой топик /wheel_odom.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from robomarvel_sim import kinematics as kin

TELEMETRY_HZ = 50.0  # частота публикации /hardware/status


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        # Размеры робота объявлены параметрами, поэтому их можно
        # задавать при запуске, не редактируя файл.
        self.declare_parameter("wheel_radius", 0.021)
        self.declare_parameter("wheel_base", 0.128)
        # Точка, от которой нода начинает счёт.
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw", 0.0)

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.x = float(self.get_parameter("start_x").value)
        self.y = float(self.get_parameter("start_y").value)
        self.yaw = float(self.get_parameter("start_yaw").value)

        self.status_sub = self.create_subscription(
            Float32MultiArray, "/hardware/status", self.on_status, 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/wheel_odom", 10)

        self.get_logger().info(
            f"wheel_odometry запущен, радиус {self.wheel_radius:.3f} м, "
            f"база {self.wheel_base:.3f} м"
        )

    def on_status(self, msg):
        # Числа 1 и 4 измерены энкодерами. Числа 0 и 3 драйвер только
        # приказал колёсам, и для счисления пути они не годятся.
        w_left = msg.data[1]
        w_right = msg.data[4]

        v = (w_left + w_right) * self.wheel_radius / 2.0
        w = (w_right - w_left) * self.wheel_radius / self.wheel_base

        dt = 1.0 / TELEMETRY_HZ
        self.x, self.y, self.yaw = kin.integrate_pose(
            self.x, self.y, self.yaw, v, w, dt
        )

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = WheelOdometry()
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
