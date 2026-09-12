from glob import glob
from setuptools import find_packages, setup


package_name = "airsim_renderer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="ScaleNav",
    maintainer_email="puffy@example.com",
    description="AirSim external-physics renderer bridge for the UAV simulator.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "airsim_renderer_node = airsim_renderer.node:main",
        ],
    },
)
