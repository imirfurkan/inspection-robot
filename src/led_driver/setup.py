# setup.py
from setuptools import setup
import os
from glob import glob

setup(
    name='led_driver',
    version='0.1.0',
    packages=['led_driver'],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/led_driver']),
        ('share/led_driver', ['package.xml']),
        ('share/led_driver/launch', glob('launch/*.py')),
        ('share/led_driver/config', glob('config/*.yaml')),
    ],
    entry_points={'console_scripts': ['led_node = led_driver.led_node:main']},
)