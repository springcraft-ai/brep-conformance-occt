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

[`adapter.py`](adapter.py) answers the suite's requests on stdin and stdout, one JSON request and
one reply per line, as the suite's
[api.md](https://github.com/springcraft-ai/brep-conformance/blob/main/doc/api.md#calling-a-kernel)
describes, and keeps solids between requests under integer handles. It answers each function with
OpenCascade's one call for it, as
[the contract](https://github.com/springcraft-ai/brep-conformance/blob/main/doc/contract.md#what-an-adapter-may-do)
asks, and lists the rest as unsupported. It reaches OpenCascade through the OCP bindings, at the
release [`pyproject.toml`](pyproject.toml) pins.

A display is OpenCascade's mesher read back the way its glTF writer reads it: each face through
`RWMesh_FaceIterator`, with OpenCascade's normals and its triangles turned outward, and the polyline
of every edge the mesher draws. That includes seams and the edges on a ball, which the protocol's
display leaves out.

## Setup

[uv](https://docs.astral.sh/uv/), which installs Python 3.12 and the pinned OCP package on the first
run.

## Usage

The suite runs the adapter as a separate process. Build the adapter here, then run the suite from a
checkout of [brep-conformance](https://github.com/springcraft-ai/brep-conformance) beside this one:

```bash
uv sync --project .
cd ../brep-conformance
cargo build --release -p brep-conformance
target/release/brep-conformance --adapter "uv run -q --project ../brep-conformance-occt \
    python -I ../brep-conformance-occt/adapter.py" \
    --out occt.json
```

## Licences

The adapter in this repository is under the MIT licence: see [LICENSE](LICENSE).

That licence covers the adapter and nothing else. It does not cover the kernel, and nothing here
relicenses the kernel.

OpenCascade Technology is a separate work under its own licence: the GNU Lesser General Public
License 2.1 with the Open CASCADE exception. The adapter reaches it through the
[OCP](https://github.com/CadQuery/OCP) Python bindings, whose package declares `Apache-2.0`. Each
project's own repository is the authority on its licence, not this page.

The adapter is a Python script that imports those bindings. This repository holds no OpenCascade
source and no OpenCascade binaries; `uv` downloads them when the adapter runs. Anything you
redistribute that includes them is subject to their terms as well as this repository's.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).
