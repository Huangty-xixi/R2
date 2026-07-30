from setuptools import find_packages, setup

package_name = "vision_grasp_lidar"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shijue2",
    maintainer_email="shijue2@example.com",
    description="婵€鍏夐浄杈鹃噷绋嬭涓户鑺傜偣锛屾ˉ鎺ュ閮� livox_ros_driver2 鍖�",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_relay_node = vision_grasp_lidar.odom_relay_node:main",
        ],
    },
)
