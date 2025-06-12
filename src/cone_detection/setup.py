from setuptools import setup

package_name = 'cone_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['best.pt']),
        ('share/' + package_name + '/resource', ['resource/cone_detection']),
    ],
    install_requires=['setuptools', 'ultralytics', 'numpy', 'opencv-python'],
    zip_safe=True,
    description='YOLO‐based ZED cone detection node',
    entry_points={
        'console_scripts': [
            'cone_detection = cone_detection.cone_detection:main',
        ],
    },
)
