from setuptools import find_packages, setup

package_name = 'keyboard_teleop'

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
    entry_points={
        'console_scripts': [
            'keyboard_teleop = keyboard_teleop.keyboard_teleop:main'
        ],
    },
)
