"""Setup file."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "dc_group"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*launch.py"),
        ),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="David Bensoussan",
    author_email="david.bensoussan@brisa.tech",
    maintainer="David Bensoussan",
    maintainer_email="david.bensoussan@brisa.tech",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Collect data and group it",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "group_server = dc_group.group_server:main",
        ],
    },
)
