from setuptools import setup, find_packages
import joint_calc.__init__

setup(
    name="joint_calc",
    version=joint_calc.__init__.__version__,
    description=joint_calc.__init__.__doc__,
    author=joint_calc.__init__.__author__,
    packages=find_packages(),
    install_requires=[],
    python_requires=">=3.6",
)