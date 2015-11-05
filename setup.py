from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[''],
    scripts=['scripts/headkinematics.py', 'scripts/velma.py', 'scripts/velma_fake.py', 'scripts/velma_fk_ik.py', 'scripts/velmautils.py', 'scripts/dijkstra.py'],
    package_dir={'': ''}
)

setup(**d)
