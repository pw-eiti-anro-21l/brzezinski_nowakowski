from setuptools import setup
import os
from glob import glob
package_name = 'anro_interpolation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
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
            'jint_control_srv = anro_interpolation.jint_control_srv:main',
            'jint = anro_interpolation.jint:main',
            'oint_control_srv = anro_interpolation.oint_control_srv:main',
            'oint = anro_interpolation.oint:main',
        ],
    },
)
