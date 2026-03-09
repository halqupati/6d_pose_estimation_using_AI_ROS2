from glob import glob
import os
from setuptools import setup

package_name = "hexnut_6d_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="Planner package for 6D hex-nut pipeline.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose_fusion_node = hexnut_6d_planner.pose_fusion_node:main",
            "grasp_planner_node = hexnut_6d_planner.grasp_planner_node:main",
            "adaptive_compliance_node = hexnut_6d_planner.adaptive_compliance_node:main",
        ],
    },
)
