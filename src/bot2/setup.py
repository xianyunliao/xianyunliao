from setuptools import setup
from glob import glob
import os

package_name = 'bot2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')
        ),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xianyun',
    maintainer_email='xianyun@todo.todo',
    description='SLAM mapping task for TurtleBot3',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'frontier_explorer = bot2.frontier_explorer:main',
        ],
    },
)
