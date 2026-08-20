from glob import glob
from setuptools import find_packages, setup


package_name = "openseek_airsim_renderer"

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
    maintainer="OpenSeek",
    maintainer_email="puffy@openseek.local",
    description="AirSim external-physics renderer bridge for the OpenSeek SO3 controller.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "airsim_renderer_node = openseek_airsim_renderer.node:main",
        ],
    },
)
