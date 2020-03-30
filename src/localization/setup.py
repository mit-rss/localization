from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension
import numpy as np
import os

# Add non-ROS install locations
include_dirs = ["/usr/local/include", np.get_include()]
library_dirs = ["/usr/local/lib"]

# Get a list of all package locations
ros_package_paths = os.environ['ROS_PACKAGE_PATH']
for path in ros_package_paths.split(":"):
    # One of these will be
    # .../catkin_ws/src/
    include_dir1 = os.path.abspath(os.path.join(path, "../install/include"))
    library_dir1 = os.path.abspath(os.path.join(path, "../devel/lib"))
    include_dir2 = os.path.abspath(os.path.join(path, "../../install/include"))
    library_dir2 = os.path.abspath(os.path.join(path, "../../devel/lib"))
    include_dirs.append(include_dir1)
    library_dirs.append(library_dir1)
    include_dirs.append(include_dir2)
    library_dirs.append(library_dir2)

extensions = [
        Extension(
            "scan_simulator_2d",
            ["scan_simulator_2d.pyx"],
            libraries=["racecar_simulator"],
            include_dirs=include_dirs,
            library_dirs=library_dirs,
            language='c++',
            extra_compile_args=["-std=c++11", "-O2", "-O3"],
            extra_link_args=["-std=c++11"]
            )]

setup(
    ext_modules=cythonize(extensions)
)
