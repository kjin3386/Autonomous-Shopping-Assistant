from setuptools import setup

package_name = 'path_planning_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Path planning and velocity publishing node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'path_planner_node = path_planning_node.path_planner_node:main',
        ],
    },
)
