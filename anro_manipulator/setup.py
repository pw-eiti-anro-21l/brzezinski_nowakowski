import os
import sys
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

#Finds installation path
#First by checking COLCON_PREFIX_PATH env variable then fallbacks to getting value of --prefix parameter
install_base = os.environ['COLCON_PREFIX_PATH']  + "/" + package_name if "COLCON_PREFIX_PATH" in os.environ else "".join([arg for arg_id, arg in enumerate(sys.argv) if  arg_id > 1 and sys.argv[arg_id - 1].startswith("--prefix")])

class XacroCommand(distutils.cmd.Command):
  """A custom command to compile xarco files."""

  description = 'compile xarco files'

  def initialize_options(self):
    """Set default values for options."""
    pass
  def finalize_options(self):
    """Post-process options."""
    pass

  def run(self):
    """Run command."""
    command = ['/bin/bash', '-c', 'for f in '+os.getcwd()+'/urdf/*.xacro.xml; do xacro ${f} -o ' + install_base + '/share/' + package_name + "/" + '$(basename "$f" ".xacro.xml").urdf.xml fixed:=true; if [[ "$f" == *.fixed.xacro.xml ]]; then xacro ${f} -o ' + install_base + '/share/' + package_name + "/" + '$(basename "$f" ".fixed.xacro.xml").urdf.xml fixed:=false; fi; done']
    subprocess.check_call(command)

class BuildPyCommand(setuptools.command.build_py.build_py):
  """Custom build command."""

  def run(self):
    self.run_command('xacro')
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
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('meshes/*'))
    ],
    cmdclass={
        'xacro': XacroCommand,
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
