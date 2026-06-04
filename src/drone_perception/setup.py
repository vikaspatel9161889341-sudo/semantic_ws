import os
from glob import glob
from setuptools import setup

from setuptools import find_packages, setup

package_name = 'drone_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This maps the 'models' folder in your src to the install share directory
        ('share/' + package_name + '/models', glob('models/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanket',
    maintainer_email='sanket@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['segmentation_node = drone_perception.segmentation_node:main'        ],
    },
)
