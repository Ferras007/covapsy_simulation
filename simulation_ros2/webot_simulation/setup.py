from setuptools import find_packages, setup

package_name = 'webot_simulation'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch',  ['launch/webot_launch.py']))# TO DO CHANGE LAUNCH FILE
data_files.append(('share/' + package_name + '/resource', [
    'resource/voiture_webots.urdf',
    'resource/ros2control.yml',
])) # TO DO CHANGE RESOURCE FILE
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/piste_ens.wbt',])) # TO DO CHANGE WORLDS FILE
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/piste_ens2025.wbt',])) # TO DO CHANGE WORLDS FILE

data_files.append(('share/' + package_name + '/protos', [
    'protos/TT02_2023b.proto',
    'protos/TT02Wheel.proto',
    'protos/RpLidarA2.proto',
    'protos/Carrosserie_Audi_Light.stl',
    'protos/Carrosserie_BMW_light.stl',
    'protos/ChevroletCamaroLight.stl',

]))

data_files.append(('share/' + package_name + '/worlds', [
    'worlds/ImageToStl_virage.obj',
    'worlds/ImageToStl.com_virage.obj',
]))

data_files.append(('share/' + package_name + '/worlds', [
    'worlds/circular_piste.wbt',])) # TO DO CHANGE WORLDS FILE
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/piste_enscopy.wbt',])) # TO DO CHANGE WORLDS FILE
data_files.append(('share/' + package_name, ['package.xml']))
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='imad',
    maintainer_email='imadeddine.ghomari@yahoo.com',
    description='webots simulation for our bolide',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [    
            'teleop = webot_simulation.teleop:main',
            'obstacle_avoider = webot_simulation.obstacle_avoider:main',
            'vision_lane_follower = webot_simulation.vision_lane_follower:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
