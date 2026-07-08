from setuptools import setup, find_packages
import os

package_name = "rs05_motor"

setup(
    name=package_name,
    version="0.0.1",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    # Tambahan data_files agar memproklamirkan diri sebagai package ament_python yang sah
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="salsa_hanaa",
    maintainer_email="salsabilahanaadniah25@gmail.com",
    description="RS05 motor driver library",
    license="Apache-2.0",
    tests_require=["pytest"],
)