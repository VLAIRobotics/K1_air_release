#!/usr/bin/env bash
# Install the ABI required by the shipped teleoperation library.
set -euo pipefail

if /sbin/ldconfig -p 2>/dev/null | grep 'libyaml-cpp\.so\.0\.8 ' >/dev/null; then
    echo 'yaml-cpp 0.8 shared library is already installed.'
    exit 0
fi
for tool in git cmake; do
    command -v "$tool" >/dev/null || { echo "Missing build tool: $tool" >&2; exit 1; }
done
installer_tmp=$(mktemp -d)
trap 'rm -rf "$installer_tmp"' EXIT

git clone --depth 1 --branch 0.8.0 https://github.com/jbeder/yaml-cpp.git "$installer_tmp/source"
expected_commit=f7320141120f720aecc4c32be25586e7da9eb978
[[ "$(git -C "$installer_tmp/source" rev-parse HEAD)" == "$expected_commit" ]] || {
    echo 'yaml-cpp source identity does not match the verified release.' >&2
    exit 1
}
cmake -S "$installer_tmp/source" -B "$installer_tmp/build" \
    -DYAML_BUILD_SHARED_LIBS=ON -DYAML_CPP_BUILD_TESTS=OFF \
    -DYAML_CPP_BUILD_TOOLS=OFF -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_INSTALL_LIBDIR=lib
cmake --build "$installer_tmp/build" --parallel 2
if [[ $EUID -eq 0 ]]; then
    cmake --install "$installer_tmp/build"
    /sbin/ldconfig
else
    sudo cmake --install "$installer_tmp/build"
    sudo /sbin/ldconfig
fi
/sbin/ldconfig -p | grep 'libyaml-cpp\.so\.0\.8 '
