from setuptools import find_packages, setup

package_name = "wander_planning"

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
    description="Reactive wander + obstacle avoid from occupancy grid; publishes to /robot/control",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "wander_node = wander_planning.wander_node:main",
            "frontier_node = wander_planning.frontier_node:main",
        ],
    },
)
