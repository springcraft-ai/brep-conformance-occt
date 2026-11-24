# brep-conformance-occt

The [brep-conformance](https://github.com/springcraft-ai/brep-conformance) adapter for
[OpenCascade Technology](https://dev.opencascade.org/) (OCCT): it puts OpenCascade behind the
suite's protocol, so that the suite can score it.

## Who maintains this

brep-conformance scores a B-rep kernel through an adapter: a small program that answers the suite's
requests with the kernel's own calls. Springcraft, which maintains the suite, wrote this adapter and
maintains it.

- **It is not OpenCascade's.** OpenCascade's maintainers did not write it or choose the tests it
  runs.
- **A failure can be the adapter's.** A case OpenCascade fails in the results may be this adapter
  driving it wrongly. Corrections are welcome: open an issue or a pull request.
- **It adds nothing to the kernel.** The adapter is pass-through, as the suite's
  [contract](https://github.com/springcraft-ai/brep-conformance/blob/main/doc/contract.md#what-an-adapter-may-do)
  asks: one call of the kernel for each function, and anything the kernel has no one call for is
  listed as unsupported. What is scored is OpenCascade, not this code.
- **OpenCascade's maintainers are welcome here.** They know best how to drive OpenCascade, so their
  review and corrections carry the most weight. Ask in an issue to be added as a reviewer.
- **The suite decides what is scored.** An official run uses only a commit of this adapter that the
  suite's maintainers have reviewed as pass-through, whoever wrote it.

## Overview

[`adapter.cpp`](adapter.cpp) answers the suite's requests on stdin and stdout, one JSON request and
one reply per line, as the suite's
[api.md](https://github.com/springcraft-ai/brep-conformance/blob/main/doc/api.md#calling-a-kernel)
describes, and keeps solids between requests under integer handles. It answers each function with
OpenCascade's one call for it, as
[the contract](https://github.com/springcraft-ai/brep-conformance/blob/main/doc/contract.md#what-an-adapter-may-do)
asks, and lists only those functions as the ones it supports. It is written on the suite's
[C++ binding](https://github.com/springcraft-ai/brep-conformance/tree/main/protocol/cpp) of the
protocol, and links OpenCascade's own libraries, at the release [`pyproject.toml`](pyproject.toml)
pins.

The few numbers the adapter computes itself, such as a profile's points placed in its frame, are in
[`arithmetic.cpp`](arithmetic.cpp), which is compiled without fused multiply-adds, so they are the
same on every compiler and platform.

A display is OpenCascade's mesher read back the way its glTF writer reads it: each face through
`RWMesh_FaceIterator`, with OpenCascade's normals and its triangles turned outward, and the polyline
of each edge the mesher draws. A degenerate edge is left out. A seam, closed on its one face as
`BRep_Tool` reports, goes to `seams`. Each other edge goes to `edges`, with the faces it bounds and,
between two faces, the turn: the least and greatest signed angle between OpenCascade's outward
normals of the two faces at the polyline's vertices, each read at the point of the edge's curve the
mesher placed there. An edge is flagged as a split where its two faces lie on one surface: the same
surface, or, for planes, cylinders, cones, spheres and tori, surfaces whose axes, positions and radii
agree to within OpenCascade's `Precision::Confusion` and `Precision::Angular`. At a degenerate edge,
such as a cone's apex or a ball's pole, each triangle has its own vertex, as the protocol's display
asks, with OpenCascade's normal there along the triangle's middle.

## Setup

[uv](https://docs.astral.sh/uv/) and curl, and on Linux a C++17 compiler. [`build.sh`](build.sh)
installs the rest on its first run:

- OpenCascade's libraries, from the `cadquery-ocp-novtk` package [`pyproject.toml`](pyproject.toml)
  pins, which uv installs with Python 3.12.
- The headers of the same OpenCascade release, CMake and nlohmann/json, from conda-forge, which
  micromamba installs into `build/`. On macOS the compiler comes from conda-forge as well, so the
  adapter shares the libc++ that the package's libraries load.
- The suite's C++ binding, from its own release, pinned by its hash.

## Usage

The suite runs the adapter as a separate process. Build the adapter here, then run the suite from a
checkout of [brep-conformance](https://github.com/springcraft-ai/brep-conformance) beside this one:

```bash
./build.sh
cd ../brep-conformance
cargo build --release -p brep-conformance
target/release/brep-conformance --adapter ../brep-conformance-occt/build/occt-adapter \
    --out occt.json
```

`BREP_CONFORMANCE=<checkout> ./build.sh` builds against a checkout's binding in place of the
release's.

## Licences

The adapter in this repository is under either the MIT licence
([LICENSE-MIT](LICENSE-MIT)) or the Apache License, Version 2.0
([LICENSE-APACHE](LICENSE-APACHE)), at your option.

Either licence covers the adapter and nothing else. It does not cover the kernel, and nothing here
relicenses the kernel.

OpenCascade Technology is a separate work under its own licence: the GNU Lesser General Public
License 2.1 with the Open CASCADE exception. Each project's own repository is the authority on its
licence, not this page.

The adapter links OpenCascade's shared libraries dynamically: the ones in the `cadquery-ocp-novtk`
package of the [OCP](https://github.com/CadQuery/OCP) project, whose package declares `Apache-2.0`,
built against OpenCascade's headers from conda-forge's `occt` package. This repository holds no
OpenCascade source and no OpenCascade binaries; the build downloads them. Anything you redistribute
that includes them is subject to their terms as well as this repository's. The adapter also uses the
suite's C++ binding, under the suite's licence, and nlohmann/json, under the MIT licence.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).
