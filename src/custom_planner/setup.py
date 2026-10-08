from setuptools import find_packages, setup

package_name = 'custom_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xianyun',
    maintainer_email='xianyun@example.com',
    description='Custom A* path planner for TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_planner = custom_planner.astar_planner:main',
            'path_follower = custom_planner.path_follower:main',
        ],
    },
)
