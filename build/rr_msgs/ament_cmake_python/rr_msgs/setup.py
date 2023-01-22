from setuptools import find_packages
from setuptools import setup

setup(
    name='rr_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('rr_msgs', 'rr_msgs.*')),
)
