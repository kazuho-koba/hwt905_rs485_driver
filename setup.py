from setuptools import find_packages, setup

package_name = "hwt905_rs485_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/hwt905_imu.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kazuho",
    maintainer_email="kazuho.kobayashi.ynu@gmail.com",
    description="Driver for HWT905 RS485 IMU (Modbus RTU)",
    license="TODO: License declaration",
    tests_require=["pytest"],
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "hwt905_imu_node=hwt905_rs485_driver.imu_node:main",
            "hwt905_imu_tf_broadcaster = hwt905_rs485_driver.imu_tf_broadcaster:main",
        ],
    },
)
