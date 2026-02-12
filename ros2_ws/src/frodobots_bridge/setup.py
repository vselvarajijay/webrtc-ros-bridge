from setuptools import find_packages, setup
import os
from glob import glob

package_name = "frodobots_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    extras_require={
        "webrtc": ["aiortc", "aiohttp"],
        "http": ["requests", "aiohttp"],
    },
    zip_safe=True,
    maintainer="dgx-ros2",
    maintainer_email="user@example.com",
    description="Frodobots SDK bridge adapter for ROS2",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "frodobots_bridge = frodobots_bridge.frodobots_adapter:main",
        ],
    },
)
