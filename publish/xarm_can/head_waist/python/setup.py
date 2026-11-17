from pathlib import Path
import platform, sys
from setuptools import setup, Distribution

if sys.version_info[:2] != (3, 10):
    raise RuntimeError('K1 head/waist prebuilt SDK requires CPython 3.10')
arch = {'amd64':'x86_64','arm64':'aarch64'}.get(platform.machine().lower(), platform.machine().lower())
if sys.platform != 'linux' or arch not in ('x86_64','aarch64'):
    raise RuntimeError('Supported platforms: Linux x86_64 and aarch64')
class BinaryDistribution(Distribution):
    def has_ext_modules(self): return True
setup(name='k1-head-waist-can', version='1.0.0',
      description='Prebuilt K1 head/waist motor bindings',
      packages=['k1_head_waist_can'],
      package_dir={'k1_head_waist_can':'prebuilt/linux-'+arch+'/k1_head_waist_can'},
      package_data={'k1_head_waist_can':['*.so']},
      distclass=BinaryDistribution, python_requires='>=3.10,<3.11', zip_safe=False)
