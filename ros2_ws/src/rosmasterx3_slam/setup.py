from setuptools import setup, find_packages
from glob import glob

package_name = 'rosmasterx3_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.py'),
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Schagen',
    maintainer_email='your_email@example.com',
    description='ROSMASTERX3 SLAM research package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_monitor = rosmasterx3_slam.nodes.scan_monitor:main',
        ],
    },
)
