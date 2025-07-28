#!/usr/bin/env python3

from setuptools import setup

setup(
    name='bmp280',
    version='0.0.1',
    description='bmp280 driver',
    author='Blue Robotics',
    url='https://github.com/bluerobotics/bmp280-python',
    packages=['bmp280'],
    entry_points={
        'console_scripts': [
            'bmp280-test=bmp280.test:main',
            'bmp280-report=bmp280.report:main'
        ],
    },
    package_data={ "bmp280": ["bmp280.meta"]},
    install_requires=['smbus2'],
)
