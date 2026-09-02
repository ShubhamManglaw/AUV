from setuptools import find_packages, setup

package_name = "ps11_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/nav.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Person A",
    maintainer_email="team@newtonbotics.local",
    description="Navigation sensor models and waypoint follower",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_noise = ps11_nav.odom_noise_node:main",
            "depth_sim = ps11_nav.depth_sim_node:main",
            "range_adapter = ps11_nav.range_adapter_node:main",
            "waypoint_follower = ps11_nav.waypoint_follower_node:main",
        ],
    },
)
