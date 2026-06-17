from setuptools import find_packages, setup

package_name = 'nav_failsafe_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vikas',
    maintainer_email='vikas@todo.todo',
    description='Failsafe Core Navigation Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = nav_failsafe_core.navigation_node:main',
            'offboard_takeoff = nav_failsafe_core.offboard_takeoff:main',
        ],
    },
)
