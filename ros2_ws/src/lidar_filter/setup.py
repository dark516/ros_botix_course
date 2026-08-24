from glob import glob
import os

from setuptools import find_packages, setup


package_name = "lidar_filter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        *[
            (os.path.join("share", package_name, folder), glob(folder + "/*"))
            for folder in ("config", "launch")
        ],
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest", "PyYAML"]},
    zip_safe=True,
    maintainer="dark516",
    maintainer_email="sashakulagin2007@gmail.com",
    description="Geometric lidar self-filter for Botix.",
    license="GPL-3.0-or-later",
)
