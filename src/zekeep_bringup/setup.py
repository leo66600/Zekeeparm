from glob import glob
import os
from setuptools import find_packages, setup

package_name = "zekeep_bringup"
config_files = [
    "config/carry_params.yaml",
    "config/driver_params.yaml",
    "config/zekeep_hardware.yaml",
    "config/zekeep_hardware_withnogravity.yaml",
]
web_files = [
    path
    for path in glob("web/**/*", recursive=True)
    if os.path.isfile(path)
]
web_data_files = []
for path in web_files:
    relative_dir = os.path.dirname(path)
    web_data_files.append((f"share/{package_name}/{relative_dir}", [path]))

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", config_files),
        (f"share/{package_name}/description/urdf", glob("description/urdf/*.urdf")),
        (f"share/{package_name}/description/meshes", glob("description/meshes/*")),
        (f"share/{package_name}/description/collision", glob("description/collision/*")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        *web_data_files,
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zekeep Maintainers",
    maintainer_email="support@example.com",
    description="Launch, configuration, and description files for Zekeep.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "carry_node = zekeep_bringup.carry_node:main",
        ],
    },
)
