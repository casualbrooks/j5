from setuptools import setup

package_name = "j5_voice"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="You",
    maintainer_email="you@example.com",
    description="Minimal voice node (placeholder)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hello = j5_voice.hello:main",
        ],
    },
)
