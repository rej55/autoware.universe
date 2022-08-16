#!/usr/bin/env python3

from setuptools import find_packages
from setuptools import setup

package_name = "tier4_rtc_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Fumiya Watanabe",
    author_email="fumiya.watanabe@tier4.jp",
    maintainer="Fumiya Watanabe",
    maintainer_email="fumiya.watanabe@tier4.jp",
    description="Tools for RTC control",
    license="Apache 2.0",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    entry_points={
        "console_scripts": [
            "tier4_rtc_controller = tier4_rtc_controller.tier4_rtc_controller:ros_main",
        ],
    },
)
