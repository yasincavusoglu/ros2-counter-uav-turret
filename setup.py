from setuptools import find_packages, setup

package_name = 'counter_uav_turret'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yasin Çavuşoğlu',
    maintainer_email='yasincvsoglu002@gmail.com',
    description='Autonomous counter-UAS turret simulation in ROS 2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_sim   = counter_uav_turret.scenario_sim:main',
            'detector       = counter_uav_turret.detector:main',
            'tracker        = counter_uav_turret.tracker:main',
            'estimator      = counter_uav_turret.estimator:main',
            'lead_solver    = counter_uav_turret.lead_solver:main',
            'gimbal_control = counter_uav_turret.gimbal_control:main',
            'turret_plant   = counter_uav_turret.turret_plant:main',
            'engagement_fsm = counter_uav_turret.engagement_fsm:main',
            'hmi_telemetry  = counter_uav_turret.hmi_telemetry:main',
        ],
    },
)
