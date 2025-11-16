from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # IMUシリアル設定
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="IMU が接続されているシリアルポート",
    )

    baud_arg = DeclareLaunchArgument(
        "baud", default_value="115200", description="IMU のボーレート"
    )

    poll_hz_arg = DeclareLaunchArgument(
        "poll_hz", default_value="200", description="出力周波数"
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="base_link",
        description="Imu/MagneticField に設定する frame_id",
    )

    # TF用の親子フレーム
    parent_frame_arg = DeclareLaunchArgument(
        "parent_frame",
        default_value="world",
        description="TFの親フレーム名",
    )

    child_frame_arg = DeclareLaunchArgument(
        "child_frame",
        default_value="imu_link",
        description="TFの子フレーム名（IMUの姿勢を表す）",
    )

    imu_node = Node(
        package="hwt905_rs485_driver",
        executable="hwt905_imu_node",
        name="imu",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "baud": LaunchConfiguration("baud"),
                "frame_id": LaunchConfiguration("frame_id"),
                "poll_hz":LaunchConfiguration("poll_hz"),
            }
        ],
    )

    # ★ 追加: IMU → TF ブロードキャスタノード
    imu_tf_node = Node(
        package="hwt905_rs485_driver",
        executable="hwt905_imu_tf_broadcaster",
        name="imu_tf_broadcaster",
        output="screen",
        parameters=[
            {
                "parent_frame": LaunchConfiguration("parent_frame"),
                "child_frame": LaunchConfiguration("child_frame"),
            }
        ],
    )

    return LaunchDescription(
        [
            port_arg,
            baud_arg,
            frame_id_arg,
            poll_hz_arg,
            parent_frame_arg,
            child_frame_arg,
            imu_node,
            imu_tf_node,
        ]
    )
