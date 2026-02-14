from setuptools import find_packages, setup

package_name = "slam_pose_relay"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dgx-ros2",
    maintainer_email="user@example.com",
    description="Relay SLAM camera pose to /map_odom and optionally occupancy grid to /map",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "slam_pose_relay_node = slam_pose_relay.slam_pose_relay_node:main",
        ],
    },
)
