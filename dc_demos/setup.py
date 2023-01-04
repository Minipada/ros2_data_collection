from glob import glob
import os

from setuptools import setup


package_name = "dc_demos"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="steve",
    maintainer_email="stevenmacenski@gmail.com",
    description="An importable library for writing mobile robot applications in python3",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "qrcodes_waypoint_follower = dc_demos.qrcodes_waypoint_follower:main",
        ],
    },
)
