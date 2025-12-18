from setuptools import find_packages, setup

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dawid',
    maintainer_email='146015090+LegendaryRover@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = obstacle_avoidance.obstacle_avoidance:main', 
            'FuzzyTS_obstacle_avoidance = obstacle_avoidance.FuzzyTS_obstacle_avoidance:main',
            'FuzzyTS_GoTo = obstacle_avoidance.FuzzyTS_GoTo:main',
            'FuzzyTS_GoTo_V2 = obstacle_avoidance.FuzzyTS_GoTo_V2:main',
            'FuzzyTS_between_walls = obstacle_avoidance.FuzzyTS_between_walls:main',
        ],
    },
)
