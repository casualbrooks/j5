from setuptools import find_packages, setup

package_name = "racetracker_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Race Master Pro",
    maintainer_email="devnull@example.com",
    description="Race Master Pro ROS2 perception node package.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_node = racetracker_perception.perception_node:main",
        ],
    },
)
