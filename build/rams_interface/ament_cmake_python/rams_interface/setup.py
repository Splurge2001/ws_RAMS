from setuptools import find_packages
from setuptools import setup

setup(
    name='rams_interface',
    version='0.0.1',
    packages=find_packages(
        include=('rams_interface', 'rams_interface.*')),
)
