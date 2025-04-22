from setuptools import setup

package_name = 'user_follow_decision_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='User following decision logic node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'user_decision_node = user_follow_decision_node.user_decision_node:main',
        ],
    },
)
