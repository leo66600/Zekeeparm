from setuptools import find_packages, setup

package_name = "zekeepcontroller"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zekeep Maintainers",
    maintainer_email="support@example.com",
    description="ROS 2 controller node for Zekeep.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ZekeepController = zekeepcontroller.zekeep_controller:main",
            "GravityCompensation = zekeepcontroller.examples.gravity_compensation:main",
            "GripperControl = zekeepcontroller.examples.gripper_control:main",
            "MoveTo = zekeepcontroller.examples.move_to:main",
            "MoveToPose = zekeepcontroller.examples.move_to_pose:main",
        ],
    },
)
