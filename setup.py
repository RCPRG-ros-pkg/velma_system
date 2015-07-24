from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['headkinematics', 'velma', 'velma_fake', 'velma_fk_ik'],
    scripts=[],
    package_dir={'': 'scripts'}
)

setup(**d)
