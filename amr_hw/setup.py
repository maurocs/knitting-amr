import os
from glob import glob
from setuptools import setup

package_name = 'amr_hw'
submodules = 'amr_hw/utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mauro Conde',
    maintainer_email='maurocondes@gmail.com',
    description='Base controller for AMR',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amr_hw = amr_hw.amr_hw:main',
            'amr_odometry = amr_hw.diff_drive_odometry:main',
            'amr_controller = amr_hw.diff_drive_controller:main',
        ],
    },
)
