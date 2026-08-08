from setuptools import find_packages, setup
from glob import glob

package_name = 'packages'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eeonpeton',
    maintainer_email='eeonpeton@eeonpeton.local',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder = packages.encoder:main',
            'wheelcontrol = packages.wheelcontrol:main',
            'wheelcontrolplus = packages.wheelcontrolplus:main',
            'driver = packages.driver:main',
            'odometry = packages.odometry:main',
            'odom_vertex = packages.odom_vertex:main', 
            'gyro = packages.gyro:main',
            'autodrive = packages.autodrive:main',
            'auto_vertex = packages.auto_vertex:main',
            'localize = packages.localize:main',
            'planner = packages.planner:main',
            'human = packages.human:main',
        ],
    },
)
