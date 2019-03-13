from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension
import numpy as np

extensions = [
        Extension(
            "scan_simulator_2d",
            ["scan_simulator_2d.pyx"],
            libraries=["racecar_simulator"],
            include_dirs=["/home/racecar/racecar_ws/install/include", "/usr/local/include", np.get_include()],
            library_dirs=["/home/racecar/racecar_ws/devel/lib", "/usr/local/lib"],
            language='c++',
            extra_compile_args=["-std=c++11", "-O2", "-O3"],
            extra_link_args=["-std=c++11"]
            )]

setup(
    ext_modules=cythonize(extensions)
)
