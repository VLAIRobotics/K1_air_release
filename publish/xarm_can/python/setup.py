"""Build a platform-tagged wheel from the SDK's verified native artifact."""
import platform
import shutil
import sys
from pathlib import Path
from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


class InstallNativeArtifact(build_ext):
    def build_extension(self, ext):
        if sys.implementation.name != "cpython" or sys.version_info[:2] != (3, 10):
            raise RuntimeError("This SDK requires CPython 3.10; use a matching environment.")
        if sys.platform != "linux":
            raise RuntimeError("The CAN Python SDK supports Linux only.")
        machine = platform.machine().lower()
        arch = {"amd64": "x86_64", "arm64": "aarch64"}.get(machine, machine)
        if arch not in {"x86_64", "aarch64"}:
            raise RuntimeError(f"Unsupported CPU architecture: {machine}")
        matches = list((Path(__file__).parent / "prebuilt" / f"linux-{arch}").glob("xarm_can.cpython-310-*.so"))
        if len(matches) != 1:
            raise RuntimeError(f"Expected one CPython 3.10 artifact for {arch}, found {len(matches)}. Download the complete SDK Release attachment, not GitHub Source code.zip.")
        destination = Path(self.get_ext_fullpath(ext.name))
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(matches[0], destination)


setup(packages=[], include_package_data=False,
      ext_modules=[Extension("xarm_can", sources=[])],
      cmdclass={"build_ext": InstallNativeArtifact})
