"""Setup for irb_action_manager package."""

from glob import glob

from setuptools import find_packages, setup  # type: ignore

PACKAGE_NAME = "irb_action_manager"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME + "/config", glob("config/*")),
        ("share/" + PACKAGE_NAME + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="franklinselva",
    maintainer_email="franklinselva10@gmail.com",
    description="Action Nodes Manager for IRB6640",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "goto_action_server = irb_action_manager.goto_action_server:main",
            "detect_screws_action_server = irb_action_manager.detect_screws_action_server:main",
            "change_tool_action_server = irb_action_manager.change_tool_action_server:main",
            "unscrew_action_server = irb_action_manager.unscrew_action_server:main",
        ],
    },
)
