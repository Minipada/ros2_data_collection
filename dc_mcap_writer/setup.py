from setuptools import setup

package_name = "dc_mcap_writer"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "mcap"],
    zip_safe=True,
    maintainer="David Bensoussan",
    maintainer_email="d.bensoussan@proton.me",
    description="Passthrough MCAP writer: dc.<tag> Records to rotated .mcap files (#210)",
    license="MPL-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dc_mcap_writer = dc_mcap_writer.cli:main",
        ],
    },
)
