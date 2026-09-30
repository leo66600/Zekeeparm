from glob import glob

from setuptools import find_packages, setup

package_name = "zekeep_joystick"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zekeep Maintainers",
    maintainer_email="support@example.com",
    description="Gamepad control for the Zekeep arm servo stream.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joystick_control = zekeep_joystick.joystick_node:main",
            "cartesian_control = zekeep_joystick.cartesian_node:main",
        ],
    },
)
