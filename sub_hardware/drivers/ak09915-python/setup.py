#!/usr/bin/env python3

from setuptools import setup

setup(
    name='ak09915',
    version='0.0.1',
    description='ak09915 driver',
    author='Blue Robotics',
    url='https://github.com/bluerobotics/ak09915-python',
    packages=['ak09915'],
    entry_points={
        'console_scripts': [
            'ak09915-test=ak09915.test:main',
            'ak09915-report=ak09915.report:main'
        ],
    },
    package_data={ "ak09915": ["ak09915.meta"]},
    install_requires=['smbus2'],
)
