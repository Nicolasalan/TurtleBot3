from setuptools import find_packages, setup

package_name = 'robot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/checkpoints.launch.py']),
        ('share/' + package_name + '/launch', ['launch/checkpoints.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'localization = robot_localization.localization:main',
             'bresenham = robot_localization.bresenham:main',
             'checkpoints = robot_localization.checkpoints:main',
        ],
    },
)
