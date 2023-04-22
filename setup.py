from setuptools import setup
import os 
from glob import glob

package_name = 'project4a'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch/'),
        glob('launch/*launch.[pxy][yma]*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhassmann',
    maintainer_email='jhassmann@todo.todo',
    description='CSCE 452 Project 4a',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "jimmy = project4a.jimmy_node:main"
        ],
    },

)