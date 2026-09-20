from setuptools import find_packages, setup


package_name = "robotics_nav"


setup(
    name=package_name,
    version="0.1.0",
    # The launch directory contains launch files, not a Python package. Exclude
    # it explicitly so it cannot shadow ROS 2's own `launch` package.
    packages=find_packages(exclude=["test", "launch"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bringup.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Portfolio",
    maintainer_email="portfolio@example.com",
    description="Minimal waypoint control and sensor-aware safety for a differential-drive robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "waypoint_controller = robotics_nav.waypoint_controller:main",
            "path_follower = robotics_nav.path_follower:main",
            "safety_supervisor = robotics_nav.safety_supervisor:main",
            "static_map_publisher = robotics_nav.static_map_publisher:main",
            "global_planner = robotics_nav.global_planner:main",
        ],
    },
)
