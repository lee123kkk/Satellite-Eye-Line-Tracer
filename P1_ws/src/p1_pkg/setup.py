from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'p1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='New Workspace',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_node = p1_pkg.waypoint_node:main',
            'waypoint_node2 = p1_pkg.waypoint_node2:main',
            'waypoint_node3 = p1_pkg.waypoint_node3:main',
        ],
    },
)
