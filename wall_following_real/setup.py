from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'wall_following_real'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.py')),
        #('share/' + package_name + '/rviz', glob('rviz/*.rviz')),  # copy rviz config files
        (os.path.join('share', package_name,'rviz'), glob('rviz/*.rviz')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carl',
    maintainer_email='carl.crane@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calc_steer = wall_following_real.calc_steering:main',
        ],
    },
)
