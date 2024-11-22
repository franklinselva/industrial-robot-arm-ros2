from setuptools import find_packages, setup

package_name = "irb_task_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
        ],
    },
)
