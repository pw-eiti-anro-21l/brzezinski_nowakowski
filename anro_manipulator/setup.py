import os
from glob import glob
from setuptools import setup
from setuptools import find_packages
import setuptools.command.build_py
import distutils.cmd
import distutils.log
import setuptools
import subprocess
from ament_index_python.packages import get_package_share_directory

package_name = 'anro_manipulator'

urdf_file_name = 'manipulator_fixed.urdf.xml'
xacro_file_name = 'manipulator_fixed.xacro.xml'
global urdf
global xacro
class PylintCommand(distutils.cmd.Command):
  """A custom command to run Pylint on all Python source files."""

  description = 'run Pylint on Python source files'
  user_options = [
      # The format is (long option, short option, description).
    #   ('pylint-rcfile=', None, 'path to Pylint config file'),
  ]

  def initialize_options(self):
    """Set default values for options."""
    # Each user option must be listed here with their default value.
    # self.pylint_rcfile = ''

  def finalize_options(self):
    """Post-process options."""
    # if self.pylint_rcfile:
    #   assert os.path.exists(self.pylint_rcfile), (
    #       'Pylint config file %s does not exist.' % self.pylint_rcfile)

  def run(self):
    """Run command."""
    global urdf
    global xacro
    command = ['/bin/bash', '-c', 'xacro ' + os.getcwd() +"/urdf/"+ xacro + ' -o ' + os.getcwd() + "/urdf/" + urdf]#'xacro ' + xacro + ' -o ' + urdf]
    # if self.pylint_rcfile:
    #   command.append('--rcfile=%s' % self.pylint_rcfile)
    # command.append(os.getcwd())
    self.announce(
        'Running command: %s' % str(command),
        level=distutils.log.INFO)
    subprocess.check_call(command)

class BuildPyCommand(setuptools.command.build_py.build_py):
  """Custom build command."""

  def run(self):
    global urdf
    global xacro
    urdf = urdf_file_name
    xacro = xacro_file_name
    self.run_command('pylint')
    setuptools.command.build_py.build_py.run(self)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('urdf/*'))
    ],
    cmdclass={
        'pylint': PylintCommand,
        'build_py': BuildPyCommand,
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='gabrysbrzezinski@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = anro_manipulator.state_publisher:main'
        ],
    },
)
