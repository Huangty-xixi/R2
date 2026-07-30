from setuptools import find_packages, setup

package_name = "vision_grasp_camera"

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
    description="YOLOv8 分割检测 + 目标中心深度 + 手眼变换，可复用的摄像头检测节点",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = vision_grasp_camera.detector_node:main",
            # "debug_vis = vision_grasp_camera.debug_vis_node:main",
        ],
    },
)

