from setuptools import find_packages, setup

package_name = "occupancy_grid"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/occupancy_node.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dgx-ros2",
    maintainer_email="user@example.com",
    description="Floor occupancy grid from camera: segmentation, BEV, annotated video",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "occupancy_node = occupancy_grid.occupancy_node:main",
        ],
    },
)
