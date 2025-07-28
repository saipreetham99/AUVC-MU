#!/usr/bin/env python3

from setuptools import setup

setup(
    name='pca9685',
    version='0.0.1',
    description='pca9685 driver',
    author='Blue Robotics',
    url='https://github.com/bluerobotics/pca9685-python',
    packages=['pca9685'],
    entry_points={
        'console_scripts': [
            'pca9685-test=pca9685.test:main',
        ],
    },
    install_requires=[
        'smbus2',
        #'RPi.GPIO',
        # binary distributions are only available for the alpha release
        # use pip install RPi.GPIO --pre
    ],
)
