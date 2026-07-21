from setuptools import find_packages, setup

package_name = "vision_grasp_state_machine"

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
    description="主工作流状态机：初始化 - 等待信号 - 追踪 - 到达 - 关摄像头",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_machine_node = vision_grasp_state_machine.state_machine_node:main",
        ],
    },
)
