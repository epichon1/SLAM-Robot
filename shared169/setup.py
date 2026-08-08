from setuptools import setup
from glob import glob

package_name = 'shared169'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/maps',   glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='ME/CS/EE 169 Shared Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plotwheeldata = shared169.plotwheeldata:main',
            'rplidarfix    = shared169.rplidarfix:main',
            'detector      = shared169.detector:main'
        ],
    },
)
