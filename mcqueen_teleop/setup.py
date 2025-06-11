from setuptools import find_packages, setup

package_name = "mcqueen_teleop"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "flask>=2.0,<3.0",
        "paho-mqtt>=1.6,<2.0",
        "spacy>=3.0,<4.0",
        "rclpy",
    ],
    zip_safe=True,
    maintainer="Halil İbrahim Şenaydın",
    maintainer_email="halilsenaydin@gmail.com",
    description=(
        "A ROS 2 package that provides a web-based and voice-assisted interface "
        "for remotely controlling mobile robots. Supports sensor feedback, actuator control, "
        "and velocity command publishing over a local network."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "web_server = mcqueen_teleop.web_server:main",
        ],
    },
)
