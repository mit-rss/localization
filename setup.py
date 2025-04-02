import glob
import os
from Cython.Build import cythonize
from setuptools import setup, find_packages, Extension

package_name = 'localization'

RACECAR_SIMULATOR_PREFIX = os.path.join(os.environ["SIM_WS"], "install", "racecar_simulator")

extensions = Extension(
    "scan_simulator_2d",
    [package_name + "/scan_simulator_2d.pyx"],
    language="c++",
    libraries=["racecar_simulator"],
    include_dirs=[os.path.join(RACECAR_SIMULATOR_PREFIX, "include")],
    library_dirs=[os.path.join(RACECAR_SIMULATOR_PREFIX, "lib")],
    extra_compile_args=['-Wno-cpp', '-g', '-Wno-maybe-uninitialized'],  # Added '-Wno-maybe-uninitialized'
)
setup(
    ext_modules=cythonize(extensions, force=True, quiet=True),
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'localization/params.yaml', 'localization/test/test_params.yaml', 'localization/real_params.yaml']),
        ('share/localization/launch',
         glob.glob(os.path.join('launch', '*launch.*')) + glob.glob(os.path.join('launch/unit_tests', '*launch.*'))),
        ('share/localization/test_map', glob.glob(os.path.join('test_map', '*'))),
    ],
    install_requires=['setuptools', "Cython"],
    zip_safe=True,
    maintainer='alanyu',
    maintainer_email='alanyu@csail.mit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'particle_filter = localization.particle_filter:main',
            'sensor_model_test = localization.test.sensor_model_test:main',
            'motion_model_test = localization.test.motion_model_test:main',
        ],
    },

)
