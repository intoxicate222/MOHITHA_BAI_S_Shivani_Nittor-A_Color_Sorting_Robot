from setuptools import setup
import os
from glob import glob

package_name = 'simple_sorter_sim'

setup(
    name=package_name,
    version='0.0.1',
    # If you have Python modules directly under simple_sorter_sim/ (e.g. simple_sorter_sim/my_module.py)
    # packages=[package_name], # This would expect simple_sorter_sim/simple_sorter_sim/__init__.py
    # For scripts, we install them to lib
    packages=[], # No top-level Python package if scripts are standalone
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # This makes scripts available in install/simple_sorter_sim/lib/simple_sorter_sim/
        # The launch file can then find them.
        (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohitha',
    maintainer_email='mohithabai12@gmail.com',
    description='Simple color sorter simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             # This allows 'ros2 run simple_sorter_sim sorter_logic_executable'
             'sorter_logic_executable = simple_sorter_sim.sorter_logic:main',
        ],
    },
)
