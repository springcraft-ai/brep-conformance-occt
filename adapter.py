"""The OpenCascade adapter for the B-rep protocol (doc/api.md): one JSON request per line on stdin,
one reply per line on stdout, solids kept here between requests under integer handles.
"""

import contextlib
import json
import math
import os
import sys
import tempfile
from importlib.metadata import version

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepAlgoAPI import (
    BRepAlgoAPI_Common,
    BRepAlgoAPI_Cut,
    BRepAlgoAPI_Defeaturing,
    BRepAlgoAPI_Fuse,
    BRepAlgoAPI_Section,
    BRepAlgoAPI_Splitter,
)
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_GTransform,
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeVertex,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_Transform,
)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepFilletAPI import BRepFilletAPI_MakeChamfer, BRepFilletAPI_MakeFillet
from OCP.BRepGProp import BRepGProp
from OCP.BRepLib import BRepLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepOffset import BRepOffset_MakeOffset, BRepOffset_Skin
from OCP.BRepOffsetAPI import (
    BRepOffsetAPI_DraftAngle,
    BRepOffsetAPI_MakeOffsetShape,
    BRepOffsetAPI_MakePipe,
    BRepOffsetAPI_MakePipeShell,
    BRepOffsetAPI_MakeThickSolid,
    BRepOffsetAPI_ThruSections,
)
from OCP.BRepPrimAPI import (
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeCone,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakePrism,
    BRepPrimAPI_MakeRevol,
    BRepPrimAPI_MakeSphere,
    BRepPrimAPI_MakeTorus,
)
from OCP.GC import GC_MakeArcOfCircle
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.Geom import (
    Geom_BezierCurve,
    Geom_BSplineCurve,
    Geom_CylindricalSurface,
)
from OCP.Geom2d import Geom2d_Line
from OCP.GeomAbs import GeomAbs_Arc, GeomAbs_Intersection
from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
from OCP.GeomLProp import GeomLProp_SLProps
from OCP.gp import (
    gp_Ax1,
    gp_Ax2,
    gp_Ax3,
    gp_Circ,
    gp_Dir,
    gp_Dir2d,
    gp_GTrsf,
    gp_Lin,
    gp_Mat,
    gp_Pln,
    gp_Pnt,
    gp_Pnt2d,
    gp_Trsf,
    gp_Vec,
    gp_XYZ,
)
from OCP.GProp import GProp_GProps
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.IFSelect import IFSelect_RetDone
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.OCP.collections import (
    Array1_double,
    Array1_gp_Pnt,
    Array1_gp_Pnt2d,
    Array1_int,
    List_TopoDS_Shape,
)
from OCP.RWMesh import RWMesh_FaceIterator
from OCP.Standard import Standard_Failure
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Reader, STEPControl_Writer
from OCP.TopAbs import (
    TopAbs_EDGE,
    TopAbs_FACE,
    TopAbs_IN,
    TopAbs_ON,
    TopAbs_OUT,
    TopAbs_REVERSED,
    TopAbs_SHELL,
    TopAbs_SOLID,
)
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Compound

solids = {}


class UnsupportedError(Exception):
    """A request OpenCascade has no one call for, with these arguments."""


def frame(f):
    return gp_Ax2(gp_Pnt(*f["origin"]), gp_Dir(*f["z"]), gp_Dir(*f["x"]))


def make(p):
    ax = frame(p["frame"])
    kind = p["cmd"]
    if kind == "box":
        s = p["size"]
        return BRepPrimAPI_MakeBox(ax, s[0], s[1], s[2]).Shape()
    if kind == "cylinder":
        return BRepPrimAPI_MakeCylinder(ax, p["radius"], p["height"]).Shape()
    if kind == "sphere":
        return BRepPrimAPI_MakeSphere(ax, p["radius"]).Shape()
    if kind == "torus":
        return BRepPrimAPI_MakeTorus(ax, p["major"], p["minor"]).Shape()
    if kind == "cone":
        return BRepPrimAPI_MakeCone(ax, p["bottom"], p["top"], p["height"]).Shape()
    f = p["frame"]
    o, x, y, z = (f[k] for k in ("origin", "x", "y", "z"))
    if kind == "extrude":
        face = profile_face(p["profile"], o, x, y)
        return BRepPrimAPI_MakePrism(face, gp_Vec(*z) * p["height"]).Shape()
    if kind == "revolve":
        face = profile_face(p["profile"], o, x, z)
        return BRepPrimAPI_MakeRevol(
            face, gp_Ax1(gp_Pnt(*o), gp_Dir(*z)), p["angle"]
        ).Shape()
    if kind == "screw":
        return helix(p, o, x, z)
    if kind == "sweep":
        return sweep(p, o, x, y, z)
    if kind == "sweep_path":
        return sweep_path(p, o, x, y, z)
    if kind == "loft":
        top = [o[k] + z[k] * p["height"] for k in range(3)]
        loft = BRepOffsetAPI_ThruSections(True, True)
        loft.AddWire(loop_wire(one_loop(p["profile"]), o, x, y, 1.0))
        loft.AddWire(loop_wire(one_loop(p["top"]), top, x, y, 1.0))
        loft.Build()
        return done(loft, "loft")
    raise UnsupportedError(f"unknown constructor {kind}")


def done(algo, name):
    """The algorithm's shape, once it says it finished."""
    if not algo.IsDone():
        raise RuntimeError(f"{name} failed")
    return algo.Shape()


def one_loop(profile):
    """The profile's one loop, for a call that takes a wire: holes are refused, not dropped."""
    if len(profile) != 1:
        raise UnsupportedError("a profile with holes is not supported here")
    return profile[0]


def loop_wire(loop, o, x, y, scale):
    """A wire through the loop's segments, drawn in the plane through `o` along `x` and `y` and
    scaled by `scale` about `o`."""

    def at(u, v):
        return gp_Pnt(*(o[k] + scale * (x[k] * u + y[k] * v) for k in range(3)))

    normal = gp_Dir(
        *(
            x[1] * y[2] - x[2] * y[1],
            x[2] * y[0] - x[0] * y[2],
            x[0] * y[1] - x[1] * y[0],
        )
    )
    wire = BRepBuilderAPI_MakeWire()
    for seg in loop:
        if "line" in seg:
            a, b = seg["line"]
            wire.Add(BRepBuilderAPI_MakeEdge(at(*a), at(*b)).Edge())
            continue
        if "spline" in seg:
            wire.Add(BRepBuilderAPI_MakeEdge(periodic_spline(seg["spline"], at)).Edge())
            continue
        if "conic" in seg:
            cn = seg["conic"]
            poles = Array1_gp_Pnt(1, 3)
            for i, key in enumerate(("a", "apex", "b"), 1):
                poles.SetValue(i, at(*cn[key]))
            weights = Array1_double(1, 3)
            for i, wt in enumerate((1.0, cn["w"], 1.0), 1):
                weights.SetValue(i, wt)
            wire.Add(BRepBuilderAPI_MakeEdge(Geom_BezierCurve(poles, weights)).Edge())
            continue
        if "arc" not in seg:
            raise UnsupportedError(f"a {next(iter(seg))} segment is not supported")
        arc = seg["arc"]
        c, r, s, w = arc["center"], arc["radius"], arc["start"], arc["sweep"]
        if abs(w) >= 2 * math.pi:
            # A whole circle, its direction set by the axis it turns about.
            n = normal if w > 0 else normal.Reversed()
            circ = gp_Circ(gp_Ax2(at(*c), n), r * scale)
            wire.Add(BRepBuilderAPI_MakeEdge(circ).Edge())
            continue
        pts = [
            at(c[0] + r * math.cos(t), c[1] + r * math.sin(t))
            for t in (s, s + w / 2, s + w)
        ]
        wire.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(*pts).Value()).Edge())
    return wire.Wire()


def profile_face(loops, o, x, y):
    """The planar face the loops bound, the first outer and the rest holes. A profile's outer loop
    runs counter-clockwise and its holes clockwise, which is how the face builder takes them."""
    make = BRepBuilderAPI_MakeFace(loop_wire(loops[0], o, x, y, 1.0), True)
    for hole in loops[1:]:
        make.Add(loop_wire(hole, o, x, y, 1.0))
    return make.Face()


def helix(p, o, x, z):
    """The profile screwed about z by a pipe along a helix with its binormal held along z: each
    section is the start one turned about the axis and lifted, which is the screw motion."""
    axis = gp_Ax3(gp_Pnt(*o), gp_Dir(*z), gp_Dir(*x))
    cyl = Geom_CylindricalSurface(axis, 1.0)
    rise, angle = p["rise"], p["angle"]
    # On the cylinder (angle, height): the line height = rise·angle, by arc length in that plane.
    line = Geom2d_Line(gp_Pnt2d(0.0, 0.0), gp_Dir2d(1.0, rise))
    edge = BRepBuilderAPI_MakeEdge(line, cyl, 0.0, angle * math.hypot(1.0, rise)).Edge()
    BRepLib.BuildCurves3d_s(edge)
    pipe = BRepOffsetAPI_MakePipeShell(BRepBuilderAPI_MakeWire(edge).Wire())
    pipe.SetMode(gp_Dir(*z))
    # Placed at the spine's start: left to choose, the pipe puts the profile at the nearest point
    # of the spine, a turn up where the profile sits nearer one pitch than none.
    profile = loop_wire(one_loop(p["profile"]), o, x, z, 1.0)
    pipe.Add(profile, TopExp.FirstVertex_s(edge), False, False)
    pipe.Build()
    if not pipe.MakeSolid():
        raise RuntimeError("pipe shell did not close into a solid")
    return pipe.Shape()


def sweep_path(p, o, x, y, z):
    """The profile carried along the B-spline by a pipe shell in its discrete mode, whose frame is
    the rotation-minimizing one the function asks for. OpenCascade's pipe has no twist law, so a
    twisted sweep is one it has no call for."""
    if p["twist"] != 0:
        raise UnsupportedError("a twisted sweep: the pipe shell has no twist law")
    path = p["path"]
    ctrl, knots, deg = path["ctrl"], path["knots"], path["degree"]
    poles = Array1_gp_Pnt(1, len(ctrl))
    for i, c in enumerate(ctrl):
        poles.SetValue(
            i + 1,
            gp_Pnt(*(o[k] + x[k] * c[0] + y[k] * c[1] + z[k] * c[2] for k in range(3))),
        )
    values, mults = [], []
    for t in knots:
        if values and t == values[-1]:
            mults[-1] += 1
        else:
            values.append(t)
            mults.append(1)
    va, ma = Array1_double(1, len(values)), Array1_int(1, len(values))
    for i, (t, m) in enumerate(zip(values, mults)):
        va.SetValue(i + 1, t)
        ma.SetValue(i + 1, m)
    curve = Geom_BSplineCurve(poles, va, ma, deg)
    spine = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(curve).Edge()).Wire()
    pipe = BRepOffsetAPI_MakePipeShell(spine)
    pipe.SetDiscreteMode()
    pipe.SetTolerance(1e-7, 1e-7, 1e-6)
    pipe.Add(loop_wire(one_loop(p["profile"]), o, x, y, 1.0))
    pipe.Build()
    if not pipe.IsDone() or not pipe.MakeSolid():
        raise RuntimeError("sweep_path failed")
    return pipe.Shape()


def periodic_spline(sp, at):
    """A periodic B-spline as the protocol sends it: n + degree points, the first `degree` repeated,
    and knots running `degree` past each end. OpenCascade takes the n points and the n + 1 knots of
    one period."""
    deg = sp["degree"]
    n = len(sp["ctrl"]) - deg
    poles = Array1_gp_Pnt(1, n)
    for i, (u, v) in enumerate(sp["ctrl"][:n], 1):
        poles.SetValue(i, at(u, v))
    values, counts = [], []
    for t in sp["knots"][deg : deg + n + 1]:
        if values and t == values[-1]:
            counts[-1] += 1
        else:
            values.append(t)
            counts.append(1)
    knots = Array1_double(1, len(values))
    mults = Array1_int(1, len(values))
    for i, (t, m) in enumerate(zip(values, counts), 1):
        knots.SetValue(i, t)
        mults.SetValue(i, m)
    return Geom_BSplineCurve(poles, knots, mults, deg, True)


def boolean(op, a, b):
    algo = {
        "union": BRepAlgoAPI_Fuse,
        "intersection": BRepAlgoAPI_Common,
        "difference": BRepAlgoAPI_Cut,
    }[op](a, b)
    return done(algo, op)


# In this suite's runs of the release pinned here, the fixed rule measured up to 4e-4 of the volume
# off on crossing cylinders' spline faces, and the adaptive one 5% off on one periodic spline prism
# while reporting an error estimate of 1e-17. The adapter uses the adaptive rule;
# BREP_CONFORMANCE_OCCT_GPROP=fixed picks the fixed one.
ADAPTIVE = os.environ.get("BREP_CONFORMANCE_OCCT_GPROP", "adaptive") != "fixed"


def volume(s):
    v = GProp_GProps()
    if ADAPTIVE:
        BRepGProp.VolumeProperties_s(s, v, 1e-9, False)
    else:
        BRepGProp.VolumeProperties_s(s, v)
    return v.Mass()


def area(s):
    a = GProp_GProps()
    if ADAPTIVE:
        BRepGProp.SurfaceProperties_s(s, a, 1e-9)
    else:
        BRepGProp.SurfaceProperties_s(s, a)
    return a.Mass()


def check(s):
    """Whether OpenCascade's analyzer finds the shape valid. The analyzer keeps a list of statuses
    for each face, edge and vertex, which this binding does not expose, so an invalid shape is
    reported as that and no more."""
    return [] if BRepCheck_Analyzer(s).IsValid() else ["BRepCheck_Analyzer: invalid"]


def mass(s):
    props = GProp_GProps()
    if ADAPTIVE:
        BRepGProp.VolumeProperties_s(s, props, 1e-9, False)
    else:
        BRepGProp.VolumeProperties_s(s, props)
    return props


def has_solid(s):
    return TopExp_Explorer(s, TopAbs_SOLID).More()


def centroid(s):
    if not has_solid(s):
        return None
    c = mass(s).CentreOfMass()
    return [c.X(), c.Y(), c.Z()]


def inertia(s):
    """The inertia tensor about the centre of mass, which is how OpenCascade reports it."""
    if not has_solid(s):
        return [[0.0] * 3 for _ in range(3)]
    m = mass(s).MatrixOfInertia()
    return [[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]


def edges_at(s, points):
    """The edge of `s` each point lies on: the nearest, which must pass within 1e-6 mm."""
    found = []
    for x, y, z in points:
        vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(x, y, z)).Vertex()
        best, best_d = None, float("inf")
        explorer = TopExp_Explorer(s, TopAbs_EDGE)
        while explorer.More():
            edge = TopoDS.Edge(explorer.Current())
            d = BRepExtrema_DistShapeShape(edge, vertex).Value()
            if d < best_d:
                best, best_d = edge, d
            explorer.Next()
        if best is None or best_d > 1e-6:
            raise ValueError(f"no edge passes through {(x, y, z)}")
        found.append(best)
    return found


def blend(s, req):
    edges = edges_at(s, req["edges"])
    if req["cmd"] == "fillet_radii":
        algo, sizes = BRepFilletAPI_MakeFillet(s), req["radii"]
    elif req["cmd"] == "chamfer_distances":
        algo, sizes = BRepFilletAPI_MakeChamfer(s), req["distances"]
    elif req["cmd"] == "fillet":
        algo, sizes = BRepFilletAPI_MakeFillet(s), [req["radius"]] * len(edges)
    else:
        algo, sizes = BRepFilletAPI_MakeChamfer(s), [req["distance"]] * len(edges)
    if len(sizes) != len(edges):
        raise ValueError(f"{len(sizes)} sizes for {len(edges)} edges")
    for size, edge in zip(sizes, edges):
        algo.Add(size, edge)
    algo.Build()
    if not algo.IsDone():
        raise RuntimeError(f"{req['cmd']} failed")
    return algo.Shape()


@contextlib.contextmanager
def quiet():
    """Point file descriptor 1 at /dev/null for the duration: OpenCascade's STEP translators print
    progress there, which would corrupt the protocol's replies."""
    sys.stdout.flush()
    saved = os.dup(1)
    null = os.open(os.devnull, os.O_WRONLY)
    os.dup2(null, 1)
    try:
        yield
    finally:
        os.dup2(saved, 1)
        os.close(null)
        os.close(saved)


def write_step(s):
    """The solid as STEP text, through a temporary file: the writer only writes files."""
    with quiet(), tempfile.TemporaryDirectory() as d:
        writer = STEPControl_Writer()
        writer.Transfer(s, STEPControl_AsIs)
        path = os.path.join(d, "out.step")
        if writer.Write(path) != IFSelect_RetDone:
            raise RuntimeError("writing STEP failed")
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()


def read_step(text):
    with quiet(), tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "in.step")
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        reader = STEPControl_Reader()
        if reader.ReadFile(path) != IFSelect_RetDone:
            raise RuntimeError("reading STEP failed")
        if reader.TransferRoots() < 1:
            raise RuntimeError("the STEP file transferred no shape")
        return reader.OneShape()


def closest(s, points):
    """The point of the solid nearest each point: the point itself where the solid holds it."""
    out = []
    for x, y, z in points:
        d = BRepExtrema_DistShapeShape(
            BRepBuilderAPI_MakeVertex(gp_Pnt(x, y, z)).Vertex(), s
        )
        if not d.IsDone() or d.NbSolution() < 1:
            raise RuntimeError("closest failed")
        p = d.PointOnShape2(1)
        out.append([p.X(), p.Y(), p.Z()])
    return out


def distance(a, b):
    d = BRepExtrema_DistShapeShape(a, b)
    if not d.IsDone():
        raise RuntimeError("distance failed")
    return d.Value()


def sweep(p, o, x, y, z):
    """The profile carried along the path by a pipe: straight edges and circular arcs, each arc
    bending toward the current x about an axis along y, the frame turning with it."""

    def turn(v, k, a):
        c, s_ = math.cos(a), math.sin(a)
        kv = sum(k[i] * v[i] for i in range(3))
        kx = [
            k[1] * v[2] - k[2] * v[1],
            k[2] * v[0] - k[0] * v[2],
            k[0] * v[1] - k[1] * v[0],
        ]
        return [v[i] * c + kx[i] * s_ + k[i] * kv * (1 - c) for i in range(3)]

    face = profile_face(p["profile"], o, x, y)
    spine = BRepBuilderAPI_MakeWire()
    cur, fx, fy, fz = list(o), list(x), list(y), list(z)
    for seg in p["path"]:
        if "line" in seg:
            end = [cur[i] + fz[i] * seg["line"] for i in range(3)]
            spine.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*cur), gp_Pnt(*end)).Edge())
            cur = end
            continue
        r, a = seg["arc"]["radius"], seg["arc"]["angle"]
        centre = [cur[i] + fx[i] * r for i in range(3)]
        rel = [cur[i] - centre[i] for i in range(3)]
        end = [centre[i] + turn(rel, fy, a)[i] for i in range(3)]
        arc = GC_MakeArcOfCircle(gp_Pnt(*cur), gp_Vec(*fz), gp_Pnt(*end)).Value()
        spine.Add(BRepBuilderAPI_MakeEdge(arc).Edge())
        cur, fx, fz = end, turn(fx, fy, a), turn(fz, fy, a)
    return done(BRepOffsetAPI_MakePipe(spine.Wire(), face), "sweep")


def transform(s, m):
    t = gp_Trsf()
    t.SetValues(*m[0], *m[1], *m[2])
    return done(BRepBuilderAPI_Transform(s, t, True), "transform")


def affine(s, m):
    g = gp_GTrsf()
    g.SetVectorialPart(gp_Mat(*m[0][:3], *m[1][:3], *m[2][:3]))
    g.SetTranslationPart(gp_XYZ(m[0][3], m[1][3], m[2][3]))
    return done(BRepBuilderAPI_GTransform(s, g, True), "affine")


def delete_faces(s, points):
    """The faces taken out and the gap closed by OpenCascade's defeaturing."""
    algo = BRepAlgoAPI_Defeaturing()
    algo.SetShape(s)
    for f in faces_at(s, points):
        algo.AddFaceToRemove(f)
    algo.SetRunParallel(False)
    algo.Build()
    if not algo.IsDone():
        raise RuntimeError("delete_faces failed")
    return algo.Shape()


def thicken(s, points, t):
    """One face thickened by OpenCascade's offset in its thickening mode, which closes the face's
    edges with walls swept along the normals. Several faces would need a shell made of them."""
    faces = faces_at(s, points)
    if len(faces) != 1:
        raise UnsupportedError(
            f"thickening {len(faces)} faces in one call is not supported"
        )
    algo = BRepOffset_MakeOffset()
    algo.Initialize(faces[0], t, 1e-7, BRepOffset_Skin, False, False, GeomAbs_Arc, True)
    algo.MakeOffsetShape()
    if not algo.IsDone():
        raise RuntimeError("thicken failed")
    return algo.Shape()


def split(s, origin, normal):
    """The solid cut by OpenCascade's splitter with the plane as its tool, each resulting solid
    put with the pieces on its side by where its centre of mass lies."""
    splitter = BRepAlgoAPI_Splitter()
    args, tools = List_TopoDS_Shape(), List_TopoDS_Shape()
    args.Append(s)
    tools.Append(
        BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(*origin), gp_Dir(*normal))).Face()
    )
    splitter.SetArguments(args)
    splitter.SetTools(tools)
    splitter.Build()
    if not splitter.IsDone():
        raise RuntimeError("split failed")
    sides = (TopoDS_Compound(), TopoDS_Compound())
    builder = BRep_Builder()
    for c in sides:
        builder.MakeCompound(c)
    ex = TopExp_Explorer(splitter.Shape(), TopAbs_SOLID)
    while ex.More():
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(ex.Current(), props)
        c = props.CentreOfMass()
        ahead = (
            sum((p - o) * n for p, o, n in zip((c.X(), c.Y(), c.Z()), origin, normal))
            > 0
        )
        builder.Add(sides[0] if ahead else sides[1], ex.Current())
        ex.Next()
    return sides


def bounding_box(s):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(s, box, False, False)
    lo, hi = box.CornerMin(), box.CornerMax()
    return [[lo.X(), lo.Y(), lo.Z()], [hi.X(), hi.Y(), hi.Z()]]


def section(s, origin, normal, tolerance):
    """The section's edges, each sampled so every chord stays within the tolerance."""
    sec = BRepAlgoAPI_Section(s, gp_Pln(gp_Pnt(*origin), gp_Dir(*normal)))
    sec.Build()
    if not sec.IsDone():
        raise RuntimeError("section failed")
    pieces = []
    ex = TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
    while ex.More():
        curve = BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
        a, b = curve.FirstParameter(), curve.LastParameter()
        sampler = GCPnts_QuasiUniformDeflection(curve, tolerance, a, b)
        pts = []
        for k in range(1, sampler.NbPoints() + 1):
            p = curve.Value(sampler.Parameter(k))
            pts.append([p.X(), p.Y(), p.Z()])
        pieces.append(pts)
        ex.Next()
    return pieces


def sew(text, tolerance):
    """The file's faces sewn within the tolerance into one shell, made a solid, and the solid
    turned to face outward. Nothing is healed, and faces that sew into several shells are
    refused."""
    shape = read_step(text)
    sewing = BRepBuilderAPI_Sewing(tolerance)
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        sewing.Add(ex.Current())
        ex.Next()
    sewing.Perform()
    sewn = sewing.SewedShape()
    shells = []
    ex = TopExp_Explorer(sewn, TopAbs_SHELL)
    while ex.More():
        shells.append(TopoDS.Shell(ex.Current()))
        ex.Next()
    if len(shells) != 1:
        raise RuntimeError(f"sewing gave {len(shells)} shells")
    solid = BRepBuilderAPI_MakeSolid(shells[0]).Solid()
    BRepLib.OrientClosedSolid_s(solid)
    return solid


def store(shape):
    h = len(solids) + 1
    while h in solids:
        h += 1
    solids[h] = shape
    return h


def many(cmd, req):
    """A boolean of many solids in one operation: OpenCascade's arguments against its tools."""
    if cmd == "union_all":
        first, *rest = [solids[h] for h in req["solids"]]
        algo = BRepAlgoAPI_Fuse()
    else:
        first, rest = solids[req["target"]], [solids[h] for h in req["tools"]]
        algo = BRepAlgoAPI_Cut()
    args, tools = List_TopoDS_Shape(), List_TopoDS_Shape()
    args.Append(first)
    for t in rest:
        tools.Append(t)
    algo.SetArguments(args)
    algo.SetTools(tools)
    algo.Build()
    return done(algo, cmd)


def offset_faces(s, points, d):
    algo = BRepOffset_MakeOffset()
    algo.Initialize(
        s, 0.0, 1e-7, BRepOffset_Skin, True, False, GeomAbs_Intersection, False
    )
    for face in faces_at(s, points):
        algo.SetOffsetOnFace(face, d)
    algo.MakeOffsetShape()
    if not algo.IsDone():
        raise RuntimeError("offset_faces failed")
    return algo.Shape()


def place(s, point, within):
    """The one face of `s` the point is addressed to, and its surface parameters there: the face
    nearest the point, which must lie within `within` of it where a distance is given. OpenCascade
    answers a normal or a curvature on one face, so a point that two faces are equally near, on an
    edge or a vertex, is one it has no call for. `None` where no face is near enough."""
    at = gp_Pnt(*point)
    vertex = BRepBuilderAPI_MakeVertex(at).Vertex()
    faces = []
    ex = TopExp_Explorer(s, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face(ex.Current())
        faces.append((BRepExtrema_DistShapeShape(face, vertex).Value(), face))
        ex.Next()
    if not faces:
        return None
    nearest = min(gap for gap, _ in faces)
    if within is not None and nearest > within:
        return None
    near = [face for gap, face in faces if gap <= nearest + 1e-6]
    if len(near) != 1:
        raise UnsupportedError(
            "a point on an edge or a vertex, where several faces meet"
        )
    surface = BRep_Tool.Surface_s(near[0])
    u, v = GeomAPI_ProjectPointOnSurf(at, surface).LowerDistanceParameters()
    return near[0], surface, u, v


def normal(s, points):
    """The outward normal at each point of the boundary: the normal of the face's surface there,
    turned where the face is reversed. None off the boundary."""
    out = []
    for point in points:
        found = place(s, point, 1e-6)
        if found is None:
            out.append(None)
            continue
        face, surface, u, v = found
        props = GeomLProp_SLProps(surface, u, v, 1, 1e-9)
        if not props.IsNormalDefined():
            out.append(None)
            continue
        n = props.Normal()
        sign = -1.0 if face.Orientation() == TopAbs_REVERSED else 1.0
        out.append([sign * n.X(), sign * n.Y(), sign * n.Z()])
    return out


def curvature(s, points):
    """The principal curvatures at the boundary point nearest each point, from that face's surface.
    OpenCascade's are signed toward the surface normal, so a reversed face flips them, and the
    protocol's positive bends away from the outward normal."""
    out = []
    for point in points:
        found = place(s, point, None)
        if found is None:
            out.append(None)
            continue
        face, surface, u, v = found
        props = GeomLProp_SLProps(surface, u, v, 2, 1e-9)
        if not props.IsCurvatureDefined():
            out.append(None)
            continue
        sign = -1.0 if face.Orientation() == TopAbs_REVERSED else 1.0
        ks = sorted((-sign * props.MaxCurvature(), -sign * props.MinCurvature()))
        out.append(ks[::-1])
    return out


def view_axes(d):
    """The protocol's view plane looking along d: u = d × z made unit (d × x along z), and
    v = −d × u."""

    def cross(a, b):
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]

    def unit(a):
        n = math.sqrt(sum(x * x for x in a))
        return [x / n for x in a]

    d = unit(d)
    u = cross(d, [0.0, 0.0, 1.0])
    u = unit(cross(d, [1.0, 0.0, 0.0]) if sum(x * x for x in u) < 1e-12 else u)
    return d, u, cross([-x for x in d], u)


def project(s, direction, tolerance):
    """OpenCascade's exact hidden-line removal, its projector looking down its z, which is toward
    the viewer, with x across: sharp edges and outlines, seen and hidden, each edge sampled so
    every chord stays within the tolerance. Smooth and seam edges are not drawn."""
    d, u, _ = view_axes(direction)
    algo = HLRBRep_Algo()
    algo.Add(s)
    axes = gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(*(-x for x in d)), gp_Dir(*u))
    algo.Projector(HLRAlgo_Projector(axes))
    algo.Update()
    algo.Hide()
    shapes = HLRBRep_HLRToShape(algo)

    def drawn(compounds):
        lines = []
        for c in compounds:
            if c.IsNull():
                continue
            BRepLib.BuildCurves3d_s(c)
            ex = TopExp_Explorer(c, TopAbs_EDGE)
            while ex.More():
                curve = BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
                a, b = curve.FirstParameter(), curve.LastParameter()
                sampler = GCPnts_QuasiUniformDeflection(curve, tolerance, a, b)
                pts = []
                for k in range(1, sampler.NbPoints() + 1):
                    p = curve.Value(sampler.Parameter(k))
                    pts.append([p.X(), p.Y()])
                lines.append(pts)
                ex.Next()
        return lines

    return {
        "visible": drawn([shapes.VCompound(), shapes.OutLineVCompound()]),
        "hidden": drawn([shapes.HCompound(), shapes.OutLineHCompound()]),
    }


def fillet_variable(s, req):
    """One edge rounded by a radius running from r_from at the end nearest `from` to r_to at the
    other, by OpenCascade's fillet on radii given along the edge."""
    (edge,) = edges_at(s, [req["edge"]])
    start = BRep_Tool.Pnt_s(TopExp.FirstVertex_s(edge))
    end = BRep_Tool.Pnt_s(TopExp.LastVertex_s(edge))
    f = gp_Pnt(*req["from"])
    r1, r2 = req["r_from"], req["r_to"]
    if start.Distance(f) > end.Distance(f):
        r1, r2 = r2, r1
    # The function's radius runs evenly. OpenCascade's call on two radii runs them by an S-shaped
    # law, and its call on a law function fails, so the even run is given as radii at 129 points:
    # fewer leave the spline through them visibly off the line at its clamped ends.
    u0, u1 = BRep_Tool.Range_s(edge)
    marks = Array1_gp_Pnt2d(1, 129)
    for i in range(129):
        t = i / 128
        marks.SetValue(i + 1, gp_Pnt2d(u0 + t * (u1 - u0), r1 + t * (r2 - r1)))
    algo = BRepFilletAPI_MakeFillet(s)
    algo.Add(marks, edge)
    algo.Build()
    if not algo.IsDone():
        raise RuntimeError("fillet_variable failed")
    return algo.Shape()


def loft_through(req):
    """A smooth solid through the sections: OpenCascade's lofts, not ruled."""
    f = req["frame"]
    o, x, y, z = f["origin"], f["x"], f["y"], f["z"]
    if len(req["profiles"]) != len(req["heights"]):
        raise ValueError(
            f"{len(req['profiles'])} profiles at {len(req['heights'])} heights"
        )
    algo = BRepOffsetAPI_ThruSections(True, False)
    for prof, h in zip(req["profiles"], req["heights"]):
        at = [o[k] + z[k] * h for k in range(3)]
        algo.AddWire(loop_wire(one_loop(prof), at, x, y, 1.0))
    algo.Build()
    if not algo.IsDone():
        raise RuntimeError("loft_through failed")
    return algo.Shape()


def chamfer_two(s, req):
    """One edge cut d1 back along the named face and d2 along the other."""
    (edge,), (face,) = edges_at(s, [req["edge"]]), faces_at(s, [req["face"]])
    algo = BRepFilletAPI_MakeChamfer(s)
    algo.Add(req["d1"], req["d2"], edge, face)
    algo.Build()
    if not algo.IsDone():
        raise RuntimeError("chamfer_two failed")
    return algo.Shape()


def raycast(s, origin, direction):
    if not TopExp_Explorer(s, TopAbs_SOLID).More():
        return []
    line = gp_Lin(gp_Pnt(*origin), gp_Dir(*direction))
    hits = IntCurvesFace_ShapeIntersector()
    hits.Load(s, 1e-7)
    hits.Perform(line, 0.0, 1e9)
    return [hits.WParameter(i) for i in range(1, hits.NbPnt() + 1)]


def mesh(s, tolerance):
    """Each face's triangulation as OpenCascade gives it, the faces' vertices each their own, its
    triangles turned to face outward where the face is reversed."""
    BRepMesh_IncrementalMesh(s, tolerance, False, 0.5, True)
    positions, triangles = [], []

    def at(p):
        positions.append([p.X(), p.Y(), p.Z()])
        return len(positions) - 1

    fx = TopExp_Explorer(s, TopAbs_FACE)
    while fx.More():
        face = TopoDS.Face(fx.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None:
            raise RuntimeError("a face has no triangulation")
        trsf = loc.Transformation()
        nodes = [at(tri.Node(i).Transformed(trsf)) for i in range(1, tri.NbNodes() + 1)]
        reversed_ = face.Orientation() == TopAbs_REVERSED
        for i in range(1, tri.NbTriangles() + 1):
            a, b, c = tri.Triangle(i).Get()
            t = [nodes[a - 1], nodes[b - 1], nodes[c - 1]]
            triangles.append([t[0], t[2], t[1]] if reversed_ else t)
        fx.Next()
    return {"positions": positions, "triangles": triangles}


def display(s, tolerance):
    """OpenCascade's mesher, read back the way its glTF writer reads it: each face through
    RWMesh_FaceIterator, which gives the face's nodes, its normals and its triangles turned outward;
    and every edge as the mesher's polygon on a face it bounds, seams included."""
    BRepMesh_IncrementalMesh(s, tolerance, False, 0.5, True)

    def xyz(p):
        return [p.X(), p.Y(), p.Z()]

    faces = []
    it = RWMesh_FaceIterator(s)
    while it.More():
        lo, hi = it.NodeLower(), it.NodeUpper()
        nodes = range(lo, hi + 1)
        elems = range(it.ElemLower(), it.ElemUpper() + 1)
        faces.append({
            "positions": [xyz(it.NodeTransformed(i)) for i in nodes],
            "normals": [xyz(it.NormalTransformed(i)) for i in nodes],
            "triangles": [[v - lo for v in it.TriangleOriented(i).Get()] for i in elems],
        })
        it.Next()

    edges, seen = [], []
    fx = TopExp_Explorer(s, TopAbs_FACE)
    while fx.More():
        face = TopoDS.Face(fx.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        trsf = loc.Transformation()
        ex = TopExp_Explorer(face, TopAbs_EDGE)
        while ex.More():
            edge = TopoDS.Edge(ex.Current())
            if tri is not None and not any(edge.IsSame(e) for e in seen):
                seen.append(edge)
                poly = BRep_Tool.PolygonOnTriangulation_s(edge, tri, loc)
                if poly is not None:
                    ids = poly.Nodes()
                    edges.append([
                        xyz(tri.Node(ids.Value(j)).Transformed(trsf))
                        for j in range(ids.Lower(), ids.Upper() + 1)
                    ])
            ex.Next()
        fx.Next()
    return {"faces": faces, "edges": edges}


def faces_at(s, points):
    """The face each point lies on: the nearest, which must pass within 1e-6 mm."""
    found = []
    for x, y, z in points:
        vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(x, y, z)).Vertex()
        best, best_d = None, float("inf")
        ex = TopExp_Explorer(s, TopAbs_FACE)
        while ex.More():
            face = TopoDS.Face(ex.Current())
            d = BRepExtrema_DistShapeShape(face, vertex).Value()
            if d < best_d:
                best, best_d = face, d
            ex.Next()
        if best is None or best_d > 1e-6:
            raise ValueError(f"no face passes through {(x, y, z)}")
        found.append(best)
    return found


def offset(s, d):
    algo = BRepOffsetAPI_MakeOffsetShape()
    algo.PerformByJoin(s, d, 1e-7, BRepOffset_Skin, False, False, GeomAbs_Arc)
    if not algo.IsDone():
        raise RuntimeError("offset failed")
    return algo.Shape()


def shell(s, open_points, t):
    if not open_points:
        raise UnsupportedError("a shell with no open face is not supported")
    removed = List_TopoDS_Shape()
    for f in faces_at(s, open_points):
        removed.Append(f)
    algo = BRepOffsetAPI_MakeThickSolid()
    algo.MakeThickSolidByJoin(s, removed, -t, 1e-7)
    if not algo.IsDone():
        raise RuntimeError("shell failed")
    return algo.Shape()


def draft(s, points, origin, pull, angle):
    algo = BRepOffsetAPI_DraftAngle(s)
    neutral = gp_Pln(gp_Pnt(*origin), gp_Dir(*pull))
    for f in faces_at(s, points):
        algo.Add(f, gp_Dir(*pull), angle, neutral)
        if not algo.AddDone():
            raise RuntimeError("a face could not be drafted")
    algo.Build()
    if not algo.IsDone():
        raise RuntimeError("draft failed")
    return algo.Shape()


CONSTRUCTORS = {
    "box",
    "cylinder",
    "sphere",
    "torus",
    "cone",
    "extrude",
    "revolve",
    "screw",
    "loft",
    "sweep",
    "sweep_path",
}
QUERIES = {
    "volume": volume,
    "area": area,
    "check": check,
    "centroid": centroid,
    "inertia": inertia,
}
# Every function of the protocol version this adapter speaks.
FUNCTIONS = [
    "box", "cylinder", "sphere", "torus", "cone", "extrude", "revolve", "screw", "loft",
    "loft_through", "sweep", "sweep_path", "slab", "union", "intersection",
    "difference", "union_all", "difference_all", "fillet", "fillet_radii",
    "fillet_variable", "fillet_conic", "fillet_g2", "fillet_setback",
    "fillet_setback_g2", "fillet_faces", "full_round", "chamfer", "chamfer_two",
    "chamfer_distances", "offset", "offset_faces", "delete_faces", "move_faces",
    "thicken", "shell", "draft", "write_step", "sew", "read_step", "volume", "area",
    "classify", "closest", "normal", "curvature", "check", "centroid", "inertia",
    "distance", "interference", "betti", "section_area", "section", "project",
    "bounding_box", "split", "mesh", "display", "raycast", "transform", "affine",
]
UNSUPPORTED = [
    # Operations OpenCascade does not have.
    "move_faces",
    "fillet_g2",
    "fillet_conic",
    "fillet_setback",
    "fillet_setback_g2",
    "fillet_faces",
    "full_round",
    # Operations OpenCascade reaches in its own way, where the protocol asks for one call of another
    # shape. The adapter does not assemble them, so that what is scored is OpenCascade.
    # It builds such a solid from a surface, side faces and sewing; the protocol asks for one call.
    "slab",
    # It counts shapes of each kind; the protocol asks for pieces, tunnels and voids.
    "betti",
    # It gives a common solid and a distance; the protocol asks for one verdict on a pair.
    "interference",
    # Its section gives curves; the protocol asks for the area they bound.
    "section_area",
]
CONFIGURATION = (
    "fuzzy value and tolerances 1e-7 mm where a call takes one; "
    + (
        "volumes and areas integrated adaptively to 1e-9"
        if ADAPTIVE
        else "volumes and areas by the fixed Gauss rule"
    )
    + "; meshes at an angular deflection of 0.5 rad"
)


def handle(req):
    cmd = req["cmd"]
    if cmd == "hello":
        name = "opencascade" if ADAPTIVE else "opencascade-fixed-gprop"
        return {
            "name": name,
            "version": version("cadquery-ocp-novtk"),
            "tolerance": 1e-7,
            "configuration": CONFIGURATION,
            "protocol": "1.0.0",
            "supports": [f for f in FUNCTIONS if f not in UNSUPPORTED],
        }
    if cmd == "reset":
        solids.clear()
        return None
    if cmd in CONSTRUCTORS:
        shape = make(req)
    elif cmd in ("union", "intersection", "difference"):
        shape = boolean(cmd, solids[req["a"]], solids[req["b"]])
    elif cmd in ("union_all", "difference_all"):
        shape = many(cmd, req)
    elif cmd == "fillet_variable":
        shape = fillet_variable(solids[req["s"]], req)
    elif cmd == "loft_through":
        shape = loft_through(req)
    elif cmd == "chamfer_two":
        shape = chamfer_two(solids[req["s"]], req)
    elif cmd == "normal":
        return normal(solids[req["s"]], req["points"])
    elif cmd == "curvature":
        return curvature(solids[req["s"]], req["points"])
    elif cmd == "project":
        return project(solids[req["s"]], req["direction"], req["tolerance"])
    elif cmd == "offset_faces":
        shape = offset_faces(solids[req["s"]], req["faces"], req["distance"])
    elif cmd in QUERIES:
        return QUERIES[cmd](solids[req["s"]])
    elif cmd == "distance":
        return distance(solids[req["a"]], solids[req["b"]])
    elif cmd in ("fillet", "fillet_radii", "chamfer", "chamfer_distances"):
        shape = blend(solids[req["s"]], req)
    elif cmd == "write_step":
        return write_step(solids[req["s"]])
    elif cmd == "read_step":
        shape = read_step(req["text"])
    elif cmd == "transform":
        shape = transform(solids[req["s"]], req["matrix"])
    elif cmd == "affine":
        shape = affine(solids[req["s"]], req["matrix"])
    elif cmd == "delete_faces":
        shape = delete_faces(solids[req["s"]], req["faces"])
    elif cmd == "thicken":
        shape = thicken(solids[req["s"]], req["faces"], req["thickness"])
    elif cmd == "split":
        up, down = split(solids[req["s"]], req["origin"], req["normal"])
        out = []
        for piece in (up, down):
            out.append(store(piece))
        return out
    elif cmd == "bounding_box":
        return bounding_box(solids[req["s"]])
    elif cmd == "section":
        return section(solids[req["s"]], req["origin"], req["normal"], req["tolerance"])
    elif cmd == "sew":
        shape = sew(req["text"], req["tolerance"])
    elif cmd == "raycast":
        if "rays" in req:
            return [raycast(solids[req["s"]], o, d) for o, d in req["rays"]]
        return raycast(solids[req["s"]], req["origin"], req["direction"])
    elif cmd == "mesh":
        return mesh(solids[req["s"]], req["tolerance"])
    elif cmd == "display":
        return display(solids[req["s"]], req["tolerance"])
    elif cmd == "offset":
        shape = offset(solids[req["s"]], req["distance"])
    elif cmd == "shell":
        shape = shell(solids[req["s"]], req["open"], req["thickness"])
    elif cmd == "draft":
        shape = draft(
            solids[req["s"]], req["faces"], req["origin"], req["pull"], req["angle"]
        )
    elif cmd == "classify":
        s = solids[req["s"]]
        # The classifier places every point inside a shape with no solid, as an empty result is.
        if not TopExp_Explorer(s, TopAbs_SOLID).More():
            return ["out"] * len(req["points"])
        c = BRepClass3d_SolidClassifier(s)
        words = {TopAbs_IN: "in", TopAbs_ON: "on", TopAbs_OUT: "out"}
        out = []
        for x, y, z in req["points"]:
            c.Perform(gp_Pnt(x, y, z), 1e-7)
            if c.State() not in words:
                raise RuntimeError(f"the classifier could not place ({x}, {y}, {z})")
            out.append(words[c.State()])
        return out
    elif cmd == "closest":
        return closest(solids[req["s"]], req["points"])
    else:
        raise UnsupportedError(f"unknown command {cmd}")
    h = len(solids) + 1
    while h in solids:
        h += 1
    solids[h] = shape
    return h


for line in sys.stdin:
    # A refusal is the kernel's exception or one this adapter raises to refuse: `UnsupportedError`
    # where OpenCascade has no call for the request, `ValueError` where the request is at fault,
    # `RuntimeError` where a call did not finish. Any other error is a fault of the adapter: it
    # ends the worker, and the runner records a crash.
    try:
        reply = {"ok": handle(json.loads(line))}
    except UnsupportedError as e:
        reply = {"err": str(e), "kind": "unsupported"}
    except ValueError as e:
        reply = {"err": str(e), "kind": "invalid"}
    except KeyError as e:
        # A handle this worker never issued, or a field the request lacks.
        reply = {"err": f"the request names {e}, which is not here", "kind": "invalid"}
    except (Standard_Failure, RuntimeError) as e:
        reply = {"err": f"{type(e).__name__}: {e}", "kind": "failed"}
    except Exception as e:
        # The bindings give many of OpenCascade's exception classes a Python class of their own
        # that is not a Standard_Failure, and on Linux the kernel's exception arrives as that one.
        if not type(e).__module__.startswith("OCP."):
            raise
        reply = {"err": f"{type(e).__name__}: {e}", "kind": "failed"}
    sys.stdout.write(json.dumps(reply) + "\n")
    sys.stdout.flush()
