from setuptools import find_packages, setup

package_name = "botix_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bringup.launch.py"]),
        ("share/" + package_name + "/config", ["config/botix.yaml"]),
        ("share/" + package_name + "/rviz", ["rviz/botix.rviz"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="dark516",
    maintainer_email="sashakulagin2007@gmail.com",
    description="ROS 2 bridge for the Botix ESP32 rover.",
    license="GPL-3.0-or-later",
    entry_points={
        "console_scripts": [
            "bridge = botix_driver.bridge_node:main",
        ],
    },
)
