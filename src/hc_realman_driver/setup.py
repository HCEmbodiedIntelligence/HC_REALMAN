from setuptools import find_packages, setup

package_name = "hc_realman_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/hc_realman.launch.py"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=False,
    entry_points={"console_scripts": ["driver_node = hc_realman_driver.node:main"]},
)
