from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["abb_omnicore_web_service"],
    package_dir={"": "src"},
)

setup(**setup_args)
