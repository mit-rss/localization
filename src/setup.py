from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension
import numpy as np

extensions = [
        Extension(
            "scan_simulator_2d",
            ["scan_simulator_2d.pyx"],
            libraries=["racecar_simulator"],
            include_dirs=["/usr/local/include/racecar_simulator", np.get_include()],
            language='c++'
            )]

setup(
    ext_modules=cythonize(extensions)
)
