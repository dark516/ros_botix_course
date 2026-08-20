from glob import glob
import os

from setuptools import find_packages, setup


package_name = "botix_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *[
            (os.path.join("share", package_name, folder), glob(folder + "/*"))
            for folder in ("config", "launch", "maps", "rviz")
        ],
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest", "PyYAML"]},
    zip_safe=True,
    maintainer="dark516",
    maintainer_email="sashakulagin2007@gmail.com",
    description="SLAM and Navigation2 bringup for Botix.",
    license="GPL-3.0-or-later",
    entry_points={
        "console_scripts": [
            "cmd_mux = botix_navigation.cmd_mux:main",
            "save_map = botix_navigation.save_map:main",
        ],
    },
)
