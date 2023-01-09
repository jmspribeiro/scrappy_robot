#!/usr/bin/env python

print(" executing setup.py")
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=['scrappy_base'],
    package_dir={'': 'src'},
))
