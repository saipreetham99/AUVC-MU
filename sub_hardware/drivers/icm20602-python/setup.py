#!/usr/bin/env python3

from setuptools import setup

setup(
    name='icm20602',
    version='0.0.1',
    description='icm20602 driver',
    author='Blue Robotics',
    url='https://github.com/bluerobotics/icm20602-python',
    packages=['icm20602'],
    entry_points={
        'console_scripts': [
            'icm20602-test=icm20602.test:main',
            'icm20602-report=icm20602.report:main'
        ],
    },
    package_data={ "icm20602": ["icm20602.meta"]},
    install_requires=['spidev'],
)
