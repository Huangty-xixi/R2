from setuptools import find_packages, setup

package_name = "vision_grasp_serial"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="shijue2",
    maintainer_email="shijue2@example.com",
    description="串口桥节点：自定义协议，上行里程计/目标/到达，下行摄像头开/关信号解析",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "serial_bridge_node = vision_grasp_serial.serial_bridge_node:main",
        ],
    },
)
