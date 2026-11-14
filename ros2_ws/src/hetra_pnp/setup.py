from glob import glob

from setuptools import setup

package_name = "hetra_pnp"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/scripts", glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="HETRA camera tracking: fiducial markers + SQPnP/IPPE pose estimation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pnp_node = hetra_pnp.pnp_node:main",
            "webcam_node = hetra_pnp.webcam_node:main",
            "pose_printer = hetra_pnp.pose_printer:main",
        ],
    },
)
