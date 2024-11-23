"""Setup for the irb_task_planner package."""

from setuptools import find_packages, setup  # type: ignore

PACKAGE_NAME = "irb_task_planner"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="franklinselva",
    maintainer_email="franklinselva10@gmail.com",
    description="Task Planning for the IRB",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "change_tool_client = irb_task_planner.action_clients.change_tool:main",
            "unscrew_client = irb_task_planner.action_clients.unscrew:main",
            "detect_screw_client = irb_task_planner.action_clients.detect_screw:main",
            "goto_client = irb_task_planner.action_clients.goto:main",
        ],
    },
)
