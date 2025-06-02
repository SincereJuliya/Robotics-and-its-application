from setuptools import find_packages
from setuptools import setup

setup(
    name='motion_planner_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('motion_planner_msgs', 'motion_planner_msgs.*')),
)
