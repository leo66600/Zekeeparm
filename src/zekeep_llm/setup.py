from setuptools import find_packages, setup


package_name = "zekeep_llm"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zekeep Maintainers",
    maintainer_email="support@example.com",
    description="DeepSeek LLM terminal planner for allowlisted Zekeep ROS tools.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["terminal = zekeep_llm.terminal:main", "web_agent = zekeep_llm.web_agent:main"]},
)
