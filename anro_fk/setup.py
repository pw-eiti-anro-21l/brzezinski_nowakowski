from setuptools import setup
import os
from glob import glob
package_name = 'anro_fk'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('rviz/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Brzeziński, Kacper Nowakowski',
    maintainer_email='gabrysbrzezinski@gmail.com, casperus99@wp.pl',
    description="Package with forwad kinematics calulations",
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nonkdl_dkin = anro_manipulator.nonkdl_dkin:main',
            'kdl_dkin = anro_manipulator.kdl_dkin:main'
        ],
    },
)
