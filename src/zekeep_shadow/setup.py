from setuptools import setup

package_name = "zekeep_shadow"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zekeep Maintainers",
    maintainer_email="support@example.com",
    description="Web RGB-D perception and grasp backends.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "web_vision = zekeep_shadow.web_vision:main",
    ]},
)
