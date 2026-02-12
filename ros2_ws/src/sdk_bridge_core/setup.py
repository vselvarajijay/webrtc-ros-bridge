from setuptools import find_packages, setup
import os
from glob import glob

package_name = "sdk_bridge_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "interfaces", "msg"), glob("sdk_bridge_core/interfaces/msg/*.msg")),
        (os.path.join("share", package_name, "interfaces", "srv"), glob("sdk_bridge_core/interfaces/srv/*.srv")),
    ],
    install_requires=["setuptools"],
    extras_require={
        "webrtc": ["aiortc", "aiohttp", "numpy"],
    },
    zip_safe=True,
    maintainer="dgx-ros2",
    maintainer_email="user@example.com",
    description="Standardized ROS2 bridge core with base interfaces for SDK adapters",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bridge_service = sdk_bridge_core.service.bridge_service:main",
        ],
    },
)
