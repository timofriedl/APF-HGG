from setuptools import setup, Extension

module = Extension(
    'apf',
    sources=['src/gateway.h'],
    extra_compile_args=['-O3', '-march=native']
)

setup(
    name='apf',
    version='1.0',
    ext_modules=[module]
)
