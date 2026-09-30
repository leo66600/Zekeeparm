from setuptools import find_packages, setup


package_name = "zekeep_teach"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/default.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zekeep Maintainers",
    maintainer_email="support@example.com",
    description="Supervised ROS 2 hand-guided teaching and replay for Zekeep.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "teach_replay = zekeep_teach.cli:main",
        ],
    },
)
