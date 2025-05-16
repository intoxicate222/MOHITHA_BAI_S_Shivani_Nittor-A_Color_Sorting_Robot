from setuptools import setup
import os
from glob import glob

package_name = 'simple_sorter_sim'

setup(
    name=package_name,
    version='0.0.1',
    
    packages=[], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        
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
             
             'sorter_logic_executable = simple_sorter_sim.sorter_logic:main',
        ],
    },
)
