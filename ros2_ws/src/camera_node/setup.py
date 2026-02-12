from setuptools import find_packages, setup

package_name = "camera_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/camera_node.launch.py"]),
    ],
    install_requires=["setuptools"],
    extras_require={"webrtc": ["aiortc", "aiohttp"]},
    zip_safe=True,
    maintainer="dgx-ros2",
    maintainer_email="user@example.com",
    description="Publishes camera stream as ROS 2 sensor_msgs/Image",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_node = camera_node.camera_node_publisher:main",
        ],
    },
)
