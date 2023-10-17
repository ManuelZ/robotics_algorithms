from setuptools import setup

package_name = 'slam_algorithms'

data_files = []

data_files.append(('share/ament_index/resource_index/packages', [
    'resource/' + package_name
]))

data_files.append(('share/' + package_name, [
    'launch/occupancy_grid.launch.py'
]))

data_files.append(('share/' + package_name + '/resource', [
    'resource/webots_robot_description.urdf',
    'resource/ros2control.yaml',
    'resource/config.rviz'
]))

data_files.append(('share/' + package_name + '/worlds', [
    'worlds/webots_world_file.wbt'
]))

data_files.append(('share/' + package_name + '/protos', [
    'protos/E-puck_enu.proto',
    'protos/E-puckDistanceSensor_enu.proto'
]))

data_files.append(('share/' + package_name, [
    'package.xml'
]))

setup(
    name=package_name,
    version          = '0.0.1',
    packages         = [package_name],
    data_files       = data_files,
    install_requires = ['setuptools', 'launch'],
    zip_safe         = True,
    maintainer       = 'manuelz',
    maintainer_email = '',
    description      = 'TODO: Package description',
    license          = 'TODO: License declaration',
    tests_require    = ['pytest'],
    entry_points     = {
        'console_scripts': [
            'occupancy_grid = slam_algorithms.occupancy_grid:main'
        ],
    },
)