from setuptools import find_packages
from setuptools import setup

setup(
    name='obstacles_msgs',
    version='0.1.2',
    packages=find_packages(
        include=('obstacles_msgs', 'obstacles_msgs.*')),
)
