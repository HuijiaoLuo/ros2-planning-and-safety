from setuptools import find_packages, setup


package_name = "robotics_sim"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "launch"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sim.launch.py"]),
        ("share/" + package_name + "/worlds", ["worlds/differential_drive.sdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Portfolio",
    maintainer_email="portfolio@example.com",
    description="Gazebo Harmonic world and ROS 2 bridge for the robotics portfolio.",
    license="Apache-2.0",
)
