from setuptools import find_packages, setup

package_name = 'skycam_control'

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
    maintainer='mapy',
    maintainer_email='mapy7772004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'skycam_dynamics = skycam_control.skycam_dynamics:main',
          'skycam_controller = skycam_control.skycam_controller:main',
          'skycam_goal_sender = skycam_control.skycam_goal_sender:main',
          'skycam_force_test = skycam_control.skycam_force_test:main',
          'skycam_cable_1 = skycam_control.skycam_cable_1:main',
          'skycam_four_cables = skycam_control.skycam_four_cables:main',
          'skycam_commander = skycam_control.skycam_commander:main',
          'skycam_trajectory_planner=skycam_control.skycam_trajectory_planner:main',
          'skycam_ai_tracker=skycam_control.skycam_ai_tracker:main',
          'ball_mover=skycam_control.ball_mover:main',
          'tension_logger=skycam_control.tension_logger:main',
          'plot_tensiones=skycam_control.plot_tensiones:main',
          'plot_error=skycam_control.plot_error:main',
          'plot_3d=skycam_control.plot_3d:main',
                ],
    },
)
