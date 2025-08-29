#!/usr/bin/env python3

from setuptools import setup

package_name = 'spazzino_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='simenza',
    maintainer_email='simenza@example.com',
    description='Controller for SPAZZ.INO',
    license='MIT',
    entry_points={
        'console_scripts': [
            'esp_bridge = spazzino_control.esp_bridge:main',
            'esp_cmdvel_node = spazzino_control.esp_cmdvel_node:main',
            'esp_odom_node = spazzino_control.esp_odom_node:main'
        ],
    },
)
