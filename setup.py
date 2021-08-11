from setuptools import setup

setup(
    name='utm_simulator',
    version='1.0',
    description='A tool to simulate different architectures for Unmanned Traffic Management',
    author='Coline Ramee',
    author_email='coline.ramee@gatech.edu',
    packages=['utm_simulator'],
    install_requires=['numpy', 'scikit-learn', 'gurobi', 'Pillow', 'matplotlib']
)
# If installing from source the package name is gurobipy, if installing with conda it's gurobi, but when importing it's still gurobipy
