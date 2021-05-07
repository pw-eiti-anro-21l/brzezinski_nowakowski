from setuptools import setup

package_name = 'anro_interpolation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='gabrysbrzezinski@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'iservice = anro_interpolation.jint_control_srv:main',
            'iclient = anro_interpolation.jint:main',
            'oservice = anro_interpolation.oint_control_srv:main',
            'oclient = anro_interpolation.oint:main',
        ],
    },
)
