from setuptools import setup
import os
from glob import glob

package_name = 'rear_tracking_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rear_camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='incsl-wego22',
    maintainer_email='incsl-wego22@todo.todo',
    description='Rear tracking node using YOLO, RealSense and DeepSORT',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rear_camera = rear_tracking_node.rear_camera:main',
        ],
    },
)

