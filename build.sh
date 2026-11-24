#!/bin/bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Builds the adapter at build/occt-adapter. OpenCascade's libraries come from the package
# pyproject.toml pins, installed by uv; the headers of the same release, CMake and nlohmann/json
# from conda-forge, installed by micromamba into build/. On macOS the compiler comes from
# conda-forge too; on Linux it is the system's.
#
# BREP_CONFORMANCE: a checkout of the suite whose C++ binding to build against, instead of the
# release's. BUILD_ENV: where to make the conda-forge environment, instead of build/env.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
build=$here/build
env=${BUILD_ENV:-$build/env}
mkdir -p "$build"

uv sync -q --frozen --project "$here"
python=$here/.venv/bin/python
version=$("$python" -I -c '
from importlib.metadata import version
print(version("cadquery-ocp-novtk"))
')
release=$(echo "$version" | cut -d. -f1-3)
libs=$("$python" -I -c '
import OCP, pathlib
package = pathlib.Path(OCP.__file__).parent
for folder in (package / ".dylibs", package.parent / "cadquery_ocp_novtk.libs"):
    if folder.is_dir():
        print(folder)
        break
')
[ -n "$libs" ] || { echo "build.sh: no OpenCascade libraries in the package" >&2; exit 1; }

case "$(uname -s)-$(uname -m)" in
    Darwin-arm64) platform=osx-arm64 compiler=clangxx ;;
    Darwin-x86_64) platform=osx-64 compiler=clangxx ;;
    Linux-x86_64) platform=linux-64 compiler= ;;
    Linux-aarch64) platform=linux-aarch64 compiler= ;;
    *) echo "build.sh: no build for $(uname -s) on $(uname -m)" >&2; exit 1 ;;
esac
mamba=$build/bin/micromamba
if [ ! -x "$mamba" ]; then
    curl -sSfL "https://micro.mamba.pm/api/micromamba/$platform/latest" |
        tar -xj -C "$build" bin/micromamba
fi
if [ ! -d "$env/include/opencascade" ]; then
    "$mamba" create -q -y -p "$env" -c conda-forge "occt=$release=novtk*" nlohmann_json cmake \
        ninja $compiler > "$build/env.log" 2>&1 || { tail -20 "$build/env.log" >&2; exit 1; }
fi

args=(-G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_MAKE_PROGRAM="$env/bin/ninja"
    -DCMAKE_PREFIX_PATH="$env" -DOCCT_INCLUDE_DIR="$env/include/opencascade"
    -DOCCT_LIBRARY_DIR="$libs" -DOCCT_PACKAGE_VERSION="$version")
if [ -n "$compiler" ]; then
    args+=(-DCMAKE_CXX_COMPILER="$env/bin/clang++" -DCMAKE_OSX_SYSROOT="$(xcrun --show-sdk-path)")
fi
if [ -n "${BREP_CONFORMANCE:-}" ]; then
    args+=(-DFETCHCONTENT_SOURCE_DIR_BREP_PROTOCOL="$(cd "$BREP_CONFORMANCE" && pwd)/protocol/cpp")
fi
"$env/bin/cmake" -S "$here" -B "$build/cmake" "${args[@]}" > "$build/cmake.log" 2>&1 ||
    { tail -30 "$build/cmake.log" >&2; exit 1; }
"$env/bin/cmake" --build "$build/cmake"
# A new file, not one copied over: macOS keeps the signature it checked for the old one.
rm -f "$build/occt-adapter"
cp "$build/cmake/occt-adapter" "$build/occt-adapter"
echo '{"cmd": "hello"}' | "$build/occt-adapter" > /dev/null
echo "built $build/occt-adapter against OpenCascade $version"
