#!/usr/bin/env python3

from setuptools import setup

setup(
    name='mmc5983',
    version='0.0.1',
    description='mmc5983 driver',
    author='Blue Robotics',
    url='https://github.com/bluerobotics/mmc5983-python',
    packages=['mmc5983'],
    entry_points={
        'console_scripts': [
            'mmc5983-test=mmc5983.test:main',
            'mmc5983-report=mmc5983.report:main'
        ],
    },
    package_data={ "mmc5983": ["mmc5983.meta"]},
    install_requires=[
        'smbus2',
        'spidev'
    ],
)
