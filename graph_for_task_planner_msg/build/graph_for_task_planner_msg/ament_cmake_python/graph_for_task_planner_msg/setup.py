from setuptools import find_packages
from setuptools import setup

setup(
    name='graph_for_task_planner_msg',
    version='0.0.0',
    packages=find_packages(
        include=('graph_for_task_planner_msg', 'graph_for_task_planner_msg.*')),
)
