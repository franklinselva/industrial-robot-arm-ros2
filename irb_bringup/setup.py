"""Build script for the irb_bringup package."""

from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "irb_bringup"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME, glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="franklinselva",
    maintainer_email="franklinselva10@gmail.com",
    description="Bringup package for the IRB task planning",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
