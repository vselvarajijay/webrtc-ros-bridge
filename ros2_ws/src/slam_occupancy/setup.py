from setuptools import find_packages, setup

package_name = "slam_occupancy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="dgx-ros2",
    maintainer_email="user@example.com",
    description="Occupancy grid from SLAM map points (ground plane + obstacles)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "slam_occupancy_node = slam_occupancy.slam_occupancy_node:main",
        ],
    },
)
