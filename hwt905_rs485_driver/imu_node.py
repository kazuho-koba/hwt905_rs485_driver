import math
import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from tf_transformations import quaternion_from_euler

import serial
import serial.tools.list_ports

import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu


def find_ttyUSB(node: Node) -> None:
    """
    接続されている /dev/ttyUSB* デバイスを列挙してログに出すヘルパ関数。
    IMUに限らずUSBシリアルが複数ある場合の確認用。
    """
    ports = [
        port.device
        for port in serial.tools.list_ports.comports()
        if "USB" in port.device
    ]
    node.get_logger().info(f"現在接続されているUSBシリアルデバイス:{len(ports)} 個：{ports}")


class Hwt905ImuNode(Node):
    """
    HWT905-RS485 (Modbus RTU) IMUドライバノード (ROS2版)
    - /wit/imu (sensor_msgs/Imu)
    - /wit/mag (sensor_msgs/MagneticField)
    をPublishする。
    """

    def __init__(self):
        super().__init__("hwt905_imu_node")

        # パラメータ宣言（launchから上書き可）
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("slave_id", 80)  # modbusスレーブID（0x50）
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("poll_hz", 200.0)  # 読み取り周期

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.slave_id = (
            self.get_parameter("slave_id").get_parameter_value().integer_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.poll_hz = self.get_parameter("poll_hz").get_parameter_value().double_value

        self.get_logger().info(
            f"IMU Type: Modbus Port: {self.port} Baud: {self.baud} SlaveID: {self.slave_id}"
        )

        find_ttyUSB(self)

        # publisher
        self.imu_pub = self.create_publisher(Imu, "wit/imu", 10)
        self.mag_pub = self.create_publisher(MagneticField, "wit/mag", 10)

        # IMUメッセージの雛形
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

        # シリアルModbusの初期化
        self.master = None
        self._serial = None
        self._open_serial_and_modbus()

        # 読み取りループ用スレッド
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _open_serial_and_modbus(self):
        """シリアルポートとModbusマスターの初期化"""
        try:
            self._serial = serial.Serial(
                port=self.port, baudrate=self.baud, timeout=0.5
            )
            if self._serial.is_open:
                self.get_logger().info("シリアルポートをオープンしました")
            else:
                self._serial.open()
                self.get_logger().info("シリアルポートをオープンしました２")

            self.master = modbus_rtu.RtuMaster(self._serial)
            self.master.set_timeout(1.0)
            self.master.set_verbose(False)
            self.get_logger().info("Modbus RTU マスターを初期化しました")

        except Exception as e:
            self.get_logger().error(f"シリアルポート初期化に失敗しました：{e}")
            self.master = None

    def _read_loop(self):
        """IMUから定期的にデータを読み取り、トピックにPublishするループ"""
        if self.master is None:
            self.get_logger().error("Modbusマスターが初期化できていないため、読み取りループを終了します")
            return

        period = 1.0 / self.poll_hz if self.poll_hz > 0.0 else 0.005

        while rclpy.ok() and self._running:
            start_time = time.time()
            try:
                # レジスタ52から12個（加速度3、角速度3、磁気3、オイラー角3）
                # start_time = time.perf_counter()
                reg = self.master.execute(
                    self.slave_id, cst.READ_HOLDING_REGISTERS, 52, 12
                )
                # mid_time = time.perf_counter()
            except Exception as e:
                self.get_logger().warn(f"レジスタ読み取りに失敗しました。接続やボーレートを確認してください：{e}")
                time.sleep(0.1)
                continue

            # 16bitレジスタを符号付きに変換
            v = [0] * 12
            for i in range(12):
                if reg[i] > 32767:
                    v[i] = reg[i] - 65536
                else:
                    v[i] = reg[i]

            # スケーリングはROS1版と同じ
            acceleration = [v[i] / 32768.0 * 16.0 * 9.8 for i in range(0, 3)]
            angular_velocity = [
                v[i] / 32768.0 * 2000.0 * math.pi / 180.0 for i in range(3, 6)
            ]
            magnetometer = v[6:9]
            angle_degree = [v[i] / 32768.0 * 180.0 for i in range(9, 12)]
            angle_radian = [deg * math.pi / 180.0 for deg in angle_degree]

            qua = quaternion_from_euler(
                angle_radian[0], angle_radian[1], angle_radian[2]
            )

            # タイムスタンプとframe_id設定
            stamp = self.get_clock().now().to_msg()
            self.imu_msg.header.stamp = stamp
            self.imu_msg.header.frame_id = self.frame_id

            self.mag_msg.header.stamp = stamp
            self.mag_msg.header.frame_id = self.frame_id

            # orientation
            self.imu_msg.orientation.x = qua[0]
            self.imu_msg.orientation.y = qua[1]
            self.imu_msg.orientation.z = qua[2]
            self.imu_msg.orientation.w = qua[3]

            # angular velocity
            self.imu_msg.angular_velocity.x = angular_velocity[0]
            self.imu_msg.angular_velocity.y = angular_velocity[1]
            self.imu_msg.angular_velocity.z = angular_velocity[2]

            # linear acceleration
            self.imu_msg.linear_acceleration.x = acceleration[0]
            self.imu_msg.linear_acceleration.y = acceleration[1]
            self.imu_msg.linear_acceleration.z = acceleration[2]

            # magnetic field
            self.mag_msg.magnetic_field.x = float(magnetometer[0])
            self.mag_msg.magnetic_field.y = float(magnetometer[1])
            self.mag_msg.magnetic_field.z = float(magnetometer[2])

            # covariance
            self.imu_msg.orientation_covariance = [-1.0] + [0.0] * 8
            self.imu_msg.angular_velocity_covariance = [
                3e-08, 0.0, 0.0,
                0.0, 5e-08, 0.0,
                0.0, 0.0, 6e-08
            ]
            self.imu_msg.linear_acceleration_covariance = [
                8e-06, 0.0, 0.0,
                0.0, 6e-06, 0.0,
                0.0, 0.0, 4e-06
            ]
            # Publish
            self.imu_pub.publish(self.imu_msg)
            self.mag_pub.publish(self.mag_msg)

            # end_time = time.perf_counter()
            # self.get_logger().info(
            #     f"modbus: {(mid_time - start_time)*1000:.2f} ms, "
            #     f"proc+pub: {(end_time - mid_time)*1000:.2f} ms"
            # )

            # 周期調整
            elapsed = time.time() - start_time
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.get_logger().info("IMU読み取りループを終了します")

    def destroy_node(self):
        """ノード終了時のクリーンアップ"""
        self._running = False
        if hasattr(self, "_thread") and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass
        if self._serial is not None and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        super().destroy_node


def main(args=None):
    rclpy.init(args=args)
    node = Hwt905ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
