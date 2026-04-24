from setuptools import setup, find_packages

setup(
    name='coverage_robot',
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    install_requires=[
        'rospy',
        'std_msgs',
        'geometry_msgs',
        'nav_msgs',
        'tf',
        'move_base_msgs'
    ],
    entry_points={
        'console_scripts': [
            'coverage_path_planner = coverage_robot.coverage_path_planner:main',
            'goal_publisher = coverage_robot.goal_publisher:main',
            'coordinator = coverage_robot.coordinator:main',
        ],
    },
)

