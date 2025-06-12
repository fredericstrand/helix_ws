import os
from glob import glob
from setuptools import setup

package_name = 'cone_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Frederic Strand',
    maintainer_email='strandfrederic@outlook.com',
    description='YOLO-based cone detector using ZED depth',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_detector_node = cone_detector.cone_detector_node:main',
        ],
    },
)