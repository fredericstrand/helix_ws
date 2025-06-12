from setuptools import setup

package_name = 'master_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eiklab',
    maintainer_email='you@domain.com',
    description='Cone distance visualizer',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_distance_visualizer = master_node.master_node:main',
        ],
    },
    data_files=[
        # install launch files to share/master_node/launch
        ('share/{}'.format(package_name), ['package.xml']),
        ('share/{}/launch'.format(package_name), ['launch/master_node.launch.py']),
    ],
)
