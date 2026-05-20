from setuptools import find_packages
from setuptools import setup

setup(
    name='clearpath_diagnostics',
    version='1.3.6',
    packages=find_packages(
        include=('clearpath_diagnostics', 'clearpath_diagnostics.*')),
)
