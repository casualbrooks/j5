from setuptools import find_packages, setup

package_name = "racetracker_perception_stack"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name] + find_packages(include=["perception", "perception.*"]),
    package_dir={package_name: "perception/ros2_node/racetracker_perception"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="J5 Maintainers",
    maintainer_email="opensource@j5.ai",
    description="ROS 2 perception node for Race Master Pro.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "perception_node = racetracker_perception_stack.perception_node:main",
        ],
    },
)
