from setuptools import setup, Extension

eigen_include = '/usr/include/eigen3'

module = Extension(
    'forward_kinematics',
    sources=['forward_kinematics.cpp'],
    include_dirs=[eigen_include],
    extra_compile_args=['-O3', '-march=native']
)

setup(
    name='forward_kinematics',
    version='1.0',
    ext_modules=[module]
)
