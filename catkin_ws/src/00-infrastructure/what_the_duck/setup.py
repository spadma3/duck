# # ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        'what_the_duck', 
        'what_the_duck_tests',
    ],
    install_requires=['termcolor'], ### unused in any case
    package_dir={'': 'include'},
)
setup(**setup_args)
