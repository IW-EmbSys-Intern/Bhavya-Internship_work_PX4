from setuptools import find_packages, setup

package_name = 'drone_control'

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
    maintainer='indowings',
    maintainer_email='indowings@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'takeoff = drone_control.takeoff:main',
            'takeoff_NGCS = drone_control.takeoff_no_GCS:main',
            'square = drone_control.square_ros2:main',
            'square2 = drone_control.square_ros2_2:main',
            'circle = drone_control.circle_ros2:main'
        ],
    },
)
