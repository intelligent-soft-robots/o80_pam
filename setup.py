from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["o80_pam"],
    package_dir={"": "python"},
)

setup(**setup_args)
