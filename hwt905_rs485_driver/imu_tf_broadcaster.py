import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ImuTfBroadcaster(Node):
    """
    /wit/imu の orientation を TF に変換して配信するノード。

    parent_frame → child_frame の回転 = IMUの姿勢
    として TF を出す。
    """

    def __init__(self):
        super().__init__("imu_tf_broadcaster")

        # パラメータ宣言
        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("child_frame", "imu_link")

        self.parent_frame = (
            self.get_parameter("parent_frame").get_parameter_value().string_value
        )
        self.child_frame = (
            self.get_parameter("child_frame").get_parameter_value().string_value
        )

        self.get_logger().info(
            f"IMU TF Broadcaster: parent={self.parent_frame}, child={self.child_frame}"
        )

        # TFブロードキャスタ
        self.br = TransformBroadcaster(self)

        # IMUサブスクライバ
        self.subscription = self.create_subscription(
            Imu, "wit/imu", self.imu_callback, 10
        )

    def imu_callback(self, msg: Imu):
        """
        Imu メッセージを受け取ったら、その orientation を TF として配信。
        """
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # 位置はとりあえず原点固定（必要ならパラメータ化も可能）
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # IMUのクォータニオンをそのまま使う
        t.transform.rotation = msg.orientation

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
