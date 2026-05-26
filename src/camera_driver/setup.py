from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'camera_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@todo.com',
    description='OAK-D Pro camera driver with web dashboard and H.265 encoding',
    license='MIT',
    entry_points={
        'console_scripts': [
            'oakd_node = camera_driver.oakd_node:main',
        ],
    },
)