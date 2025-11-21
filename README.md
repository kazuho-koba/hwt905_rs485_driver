# hwt905_rs485_driver

WitMotion HWT905/HWT905-485 IMU を **RS485（Modbus-RTU）** 経由で読み取り、  
ROS 2 (Humble) の `sensor_msgs/msg/Imu` と `sensor_msgs/msg/MagneticField` を publish するドライバです。