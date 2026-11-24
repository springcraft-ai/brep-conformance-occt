// SPDX-License-Identifier: MIT OR Apache-2.0
// The OpenCascade adapter for the B-rep protocol (doc/api.md), on the suite's C++ binding of it:
// one JSON request per line on stdin, one reply per line on stdout, solids kept between requests
// under integer handles, and each function answered with OpenCascade's one call for it.
#include "arithmetic.h"

#include <brep_protocol/brep_protocol.h>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Curve2d.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Defeaturing.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepOffset_MakeOffset.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GProp_GProps.hxx>
#include <Geom2d_Line.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <HLRAlgo_Projector.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRBRep_HLRToShape.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_PrinterOStream.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_List.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <Precision.hxx>
#include <RWMesh_FaceIterator.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir2d.hxx>
#include <gp_GTrsf.hxx>
#include <gp_Lin.hxx>
#include <gp_Mat.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <cmath>
#include <optional>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

using namespace brep_protocol;

namespace {

using Shape = TopoDS_Shape;

// In this suite's runs of the release pinned here, the fixed rule measured up to 4e-4 of the volume
// off on crossing cylinders' spline faces, and the adaptive one 5% off on one periodic spline prism
// while reporting an error estimate of 1e-17. The adapter uses the adaptive rule;
// BREP_CONFORMANCE_OCCT_GPROP=fixed picks the fixed one.
bool adaptive() {
    static const bool chosen = [] {
        const char* rule = std::getenv("BREP_CONFORMANCE_OCCT_GPROP");
        return rule == nullptr || std::string(rule) != "fixed";
    }();
    return chosen;
}

gp_Pnt pnt(const Vec3& p) {
    return gp_Pnt(p[0], p[1], p[2]);
}
gp_Dir dir(const Vec3& d) {
    return gp_Dir(d[0], d[1], d[2]);
}
Vec3 xyz(const gp_Pnt& p) {
    return {p.X(), p.Y(), p.Z()};
}
Vec3 xyz(const gp_Dir& d) {
    return {d.X(), d.Y(), d.Z()};
}

using arithmetic::along;
using arithmetic::cross;
using arithmetic::dot;
using arithmetic::turn;
Vec3 plus(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
Vec3 minus(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
Vec3 unit(const Vec3& a) {
    if (a[0] == 0.0 && a[1] == 0.0 && a[2] == 0.0) {
        throw Refusal::invalid("a direction of length zero");
    }
    return arithmetic::unit(a);
}

gp_Ax2 axes(const Frame& f) {
    return gp_Ax2(pnt(f.origin), dir(f.z), dir(f.x));
}

template <class Algo>
Shape done(Algo& algo, const std::string& name) {
    if (!algo.IsDone()) {
        throw Refusal::failed(name + " failed");
    }
    return algo.Shape();
}

// The profile's one loop, for a call that takes a wire: holes are refused, not dropped.
const Loop& one_loop(const Profile& profile) {
    if (profile.loops.size() != 1) {
        throw Refusal::unsupported("a profile with holes is not supported here");
    }
    return profile.loops[0];
}

// The sketch plane: a point (u, v) of a profile is o + scale·(u·x + v·y).
struct Plane {
    Vec3 o;
    Vec3 x;
    Vec3 y;
    double scale;

    gp_Pnt at(double u, double v) const {
        return pnt(arithmetic::in_plane(o, x, y, scale, u, v));
    }
    gp_Pnt at(const Vec2& uv) const {
        return at(uv[0], uv[1]);
    }
};

// Knot values with their multiplicities, from a list that repeats a knot once per multiplicity.
void knots_and_mults(const std::vector<double>& flat,
                     std::size_t first,
                     std::size_t last,
                     NCollection_Array1<double>& knots,
                     NCollection_Array1<int>& mults) {
    std::vector<double> values;
    std::vector<int> counts;
    for (std::size_t i = first; i < last && i < flat.size(); i++) {
        if (!values.empty() && flat[i] == values.back()) {
            counts.back() += 1;
        } else {
            values.push_back(flat[i]);
            counts.push_back(1);
        }
    }
    const int size = static_cast<int>(values.size());
    knots.Resize(1, size, false);
    mults.Resize(1, size, false);
    for (int i = 0; i < size; i++) {
        knots.SetValue(i + 1, values[i]);
        mults.SetValue(i + 1, counts[i]);
    }
}

// The periodic B-spline of one period: its first n = |ctrl| − degree points, and the knots of
// that period.
// A periodic B-spline as the protocol sends it: n + degree points, the first `degree` repeated,
// and knots running `degree` past each end. OpenCascade takes the n points and the n + 1 knots of
// one period.
occ::handle<Geom_BSplineCurve> periodic_spline(const Spline& sp, const Plane& plane) {
    const int degree = static_cast<int>(sp.degree);
    const int n = static_cast<int>(sp.ctrl.size()) - degree;
    if (n < 1) {
        throw Refusal::invalid("a periodic spline needs more points than its degree");
    }
    NCollection_Array1<gp_Pnt> poles(1, n);
    for (int i = 0; i < n; i++) {
        poles.SetValue(i + 1, plane.at(sp.ctrl[i]));
    }
    NCollection_Array1<double> knots;
    NCollection_Array1<int> mults;
    knots_and_mults(sp.knots, sp.degree, sp.degree + n + 1, knots, mults);
    return new Geom_BSplineCurve(poles, knots, mults, degree, true);
}

TopoDS_Wire loop_wire(const Loop& loop, const Plane& plane) {
    const Vec3 square = cross(plane.x, plane.y);
    const gp_Dir normal = dir(square);
    BRepBuilderAPI_MakeWire wire;
    for (const Segment& seg : loop) {
        if (const auto* line = std::get_if<Line>(&seg)) {
            wire.Add(BRepBuilderAPI_MakeEdge(plane.at(line->a), plane.at(line->b)).Edge());
        } else if (const auto* spline = std::get_if<Spline>(&seg)) {
            const occ::handle<Geom_Curve> curve = periodic_spline(*spline, plane);
            wire.Add(BRepBuilderAPI_MakeEdge(curve).Edge());
        } else if (const auto* conic = std::get_if<Conic>(&seg)) {
            NCollection_Array1<gp_Pnt> poles(1, 3);
            poles.SetValue(1, plane.at(conic->a));
            poles.SetValue(2, plane.at(conic->apex));
            poles.SetValue(3, plane.at(conic->b));
            NCollection_Array1<double> weights(1, 3);
            weights.SetValue(1, 1.0);
            weights.SetValue(2, conic->w);
            weights.SetValue(3, 1.0);
            const occ::handle<Geom_Curve> curve = new Geom_BezierCurve(poles, weights);
            wire.Add(BRepBuilderAPI_MakeEdge(curve).Edge());
        } else if (std::holds_alternative<Gn>(seg)) {
            throw Refusal::unsupported("a gn segment is not supported");
        } else {
            const Arc& arc = std::get<Arc>(seg);
            if (std::abs(arc.sweep) >= 2 * M_PI) {
                const gp_Dir n = arc.sweep > 0 ? normal : normal.Reversed();
                const gp_Circ circ(gp_Ax2(plane.at(arc.center), n), arc.radius * plane.scale);
                wire.Add(BRepBuilderAPI_MakeEdge(circ).Edge());
                continue;
            }
            gp_Pnt pts[3];
            int i = 0;
            for (const double t : {arc.start, arc.start + arc.sweep / 2, arc.start + arc.sweep}) {
                pts[i++] = plane.at(arithmetic::on_circle(arc.center, arc.radius, t));
            }
            const occ::handle<Geom_Curve> curve(GC_MakeArcOfCircle(pts[0], pts[1], pts[2]).Value());
            wire.Add(BRepBuilderAPI_MakeEdge(curve).Edge());
        }
    }
    return wire.Wire();
}

// The profile's face in the plane through `o` spanned by `x` and `y`.
// A profile's outer loop runs counter-clockwise and its holes clockwise, which is how the face
// builder takes them.
TopoDS_Face profile_face(const Profile& profile, const Vec3& o, const Vec3& x, const Vec3& y) {
    if (profile.loops.empty()) {
        throw Refusal::invalid("a profile with no loop");
    }
    const Plane plane{o, x, y, 1.0};
    BRepBuilderAPI_MakeFace make(loop_wire(profile.loops[0], plane), true);
    for (std::size_t i = 1; i < profile.loops.size(); i++) {
        make.Add(loop_wire(profile.loops[i], plane));
    }
    return make.Face();
}

// The profile screwed about z by a pipe along a helix with its binormal held along z: each
// section is the start one turned about the axis and lifted, which is the screw motion.
Shape helix(const Screw& p) {
    const Frame& f = p.frame;
    const gp_Ax3 axis(pnt(f.origin), dir(f.z), dir(f.x));
    const occ::handle<Geom_Surface> cyl = new Geom_CylindricalSurface(axis, 1.0);
    const occ::handle<Geom2d_Curve> line =
        new Geom2d_Line(gp_Pnt2d(0.0, 0.0), gp_Dir2d(1.0, p.rise));
    const TopoDS_Edge edge =
        // On the cylinder (angle, height): the line height = rise·angle, by arc length in that
        // plane.
        BRepBuilderAPI_MakeEdge(line, cyl, 0.0, p.angle * arithmetic::hypot(1.0, p.rise)).Edge();
    BRepLib::BuildCurves3d(edge);
    BRepOffsetAPI_MakePipeShell pipe(BRepBuilderAPI_MakeWire(edge).Wire());
    pipe.SetMode(dir(f.z));
    const TopoDS_Wire profile = loop_wire(one_loop(p.profile), Plane{f.origin, f.x, f.z, 1.0});
    // Placed at the spine's start: left to choose, the pipe puts the profile at the nearest point
    // of the spine, a turn up where the profile sits nearer one pitch than none.
    pipe.Add(profile, TopExp::FirstVertex(edge), false, false);
    pipe.Build();
    if (!pipe.MakeSolid()) {
        throw Refusal::failed("pipe shell did not close into a solid");
    }
    return pipe.Shape();
}

// The profile carried along the B-spline by a pipe shell in its discrete mode, whose frame is the
// rotation-minimizing one the function asks for. OpenCascade's pipe has no twist law, so a
// twisted sweep is one it has no call for.
Shape sweep_path(const SweepPath& p) {
    if (p.twist != 0) {
        throw Refusal::unsupported("a twisted sweep: the pipe shell has no twist law");
    }
    const Frame& f = p.frame;
    const int count = static_cast<int>(p.path.ctrl.size());
    NCollection_Array1<gp_Pnt> poles(1, count);
    for (int i = 0; i < count; i++) {
        poles.SetValue(i + 1, pnt(arithmetic::in_frame(f.origin, f.x, f.y, f.z, p.path.ctrl[i])));
    }
    NCollection_Array1<double> knots;
    NCollection_Array1<int> mults;
    knots_and_mults(p.path.knots, 0, p.path.knots.size(), knots, mults);
    const occ::handle<Geom_Curve> curve =
        new Geom_BSplineCurve(poles, knots, mults, static_cast<int>(p.path.degree));
    const TopoDS_Wire spine = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(curve).Edge()).Wire();
    BRepOffsetAPI_MakePipeShell pipe(spine);
    pipe.SetDiscreteMode();
    pipe.SetTolerance(1e-7, 1e-7, 1e-6);
    pipe.Add(loop_wire(one_loop(p.profile), Plane{f.origin, f.x, f.y, 1.0}));
    pipe.Build();
    if (!pipe.IsDone() || !pipe.MakeSolid()) {
        throw Refusal::failed("sweep_path failed");
    }
    return pipe.Shape();
}

// The profile carried along the path by a pipe: straight edges and circular arcs, each arc
// bending toward the current x about an axis along y, the frame turning with it.
Shape sweep(const Sweep& p) {
    const Frame& f = p.frame;
    const TopoDS_Face face = profile_face(p.profile, f.origin, f.x, f.y);
    BRepBuilderAPI_MakeWire spine;
    Vec3 cur = f.origin;
    Vec3 fx = f.x;
    const Vec3 fy = f.y;
    Vec3 fz = f.z;
    for (const PathPiece& piece : p.path) {
        if (const auto* line = std::get_if<PathLine>(&piece)) {
            const Vec3 end = along(cur, fz, line->length);
            spine.Add(BRepBuilderAPI_MakeEdge(pnt(cur), pnt(end)).Edge());
            cur = end;
            continue;
        }
        const PathArc& arc = std::get<PathArc>(piece);
        const Vec3 centre = along(cur, fx, arc.radius);
        const Vec3 end = plus(centre, turn(minus(cur, centre), fy, arc.angle));
        const gp_Vec tangent(fz[0], fz[1], fz[2]);
        const occ::handle<Geom_Curve> curve(
            GC_MakeArcOfCircle(pnt(cur), tangent, pnt(end)).Value());
        spine.Add(BRepBuilderAPI_MakeEdge(curve).Edge());
        cur = end;
        fx = turn(fx, fy, arc.angle);
        fz = turn(fz, fy, arc.angle);
    }
    BRepOffsetAPI_MakePipe pipe(spine.Wire(), face);
    return done(pipe, "sweep");
}

GProp_GProps mass(const Shape& s) {
    GProp_GProps props;
    if (adaptive()) {
        BRepGProp::VolumeProperties(s, props, 1e-9, false);
    } else {
        BRepGProp::VolumeProperties(s, props);
    }
    return props;
}

bool has_solid(const Shape& s) {
    return TopExp_Explorer(s, TopAbs_SOLID).More();
}

// The edge or face of `s` each point lies on: the nearest, which must pass within 1e-6 mm.
template <class Entity>
std::vector<Entity>
entities_at(const Shape& s, const std::vector<Vec3>& points, TopAbs_ShapeEnum kind) {
    std::vector<Entity> found;
    for (const Vec3& p : points) {
        const TopoDS_Vertex vertex = BRepBuilderAPI_MakeVertex(pnt(p)).Vertex();
        Entity best;
        bool any = false;
        double best_d = std::numeric_limits<double>::infinity();
        for (TopExp_Explorer explorer(s, kind); explorer.More(); explorer.Next()) {
            const TopoDS_Shape& entity = explorer.Current();
            const double d = BRepExtrema_DistShapeShape(entity, vertex).Value();
            if (d < best_d) {
                if constexpr (std::is_same_v<Entity, TopoDS_Edge>) {
                    best = TopoDS::Edge(entity);
                } else {
                    best = TopoDS::Face(entity);
                }
                best_d = d;
                any = true;
            }
        }
        if (!any || best_d > 1e-6) {
            std::ostringstream at;
            at << "(" << Json(p[0]).dump() << ", " << Json(p[1]).dump() << ", " << Json(p[2]).dump()
               << ")";
            const char* name = kind == TopAbs_EDGE ? "edge" : "face";
            throw Refusal::invalid(std::string("no ") + name + " passes through " + at.str());
        }
        found.push_back(best);
    }
    return found;
}

std::vector<TopoDS_Edge> edges_at(const Shape& s, const std::vector<Vec3>& points) {
    return entities_at<TopoDS_Edge>(s, points, TopAbs_EDGE);
}

std::vector<TopoDS_Face> faces_at(const Shape& s, const std::vector<Vec3>& points) {
    return entities_at<TopoDS_Face>(s, points, TopAbs_FACE);
}

template <class Algo>
Shape blend(Algo& algo,
            const std::vector<double>& sizes,
            const std::vector<TopoDS_Edge>& edges,
            const std::string& name) {
    for (std::size_t i = 0; i < edges.size(); i++) {
        algo.Add(sizes[i], edges[i]);
    }
    algo.Build();
    return done(algo, name);
}

// A temporary folder, removed with what it holds.
class TemporaryDirectory {
  public:
    TemporaryDirectory() {
        std::string pattern = (std::filesystem::temp_directory_path() / "occtXXXXXX").string();
        if (mkdtemp(pattern.data()) == nullptr) {
            throw std::runtime_error("no temporary directory");
        }
        path = pattern;
    }
    ~TemporaryDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
    TemporaryDirectory(const TemporaryDirectory&) = delete;
    TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;

    std::filesystem::path path;
};

Shape read_step_text(const std::string& text) {
    const TemporaryDirectory d;
    const std::string path = (d.path / "in.step").string();
    {
        std::ofstream file(path, std::ios::binary);
        file << text;
        if (!file) {
            throw std::runtime_error("the STEP file cannot be written");
        }
    }
    STEPControl_Reader reader;
    if (reader.ReadFile(path.c_str()) != IFSelect_RetDone) {
        throw Refusal::failed("reading STEP failed");
    }
    if (reader.TransferRoots() < 1) {
        throw Refusal::failed("the STEP file transferred no shape");
    }
    return reader.OneShape();
}

// The compound's edges, each sampled so every chord stays within the tolerance.
template <class P>
void sample_edges(const Shape& shape, double tolerance, std::vector<std::vector<P>>& lines) {
    for (TopExp_Explorer ex(shape, TopAbs_EDGE); ex.More(); ex.Next()) {
        const BRepAdaptor_Curve curve(TopoDS::Edge(ex.Current()));
        const GCPnts_QuasiUniformDeflection sampler(
            curve, tolerance, curve.FirstParameter(), curve.LastParameter());
        std::vector<P> pts;
        for (int k = 1; k <= sampler.NbPoints(); k++) {
            const gp_Pnt p = curve.Value(sampler.Parameter(k));
            if constexpr (std::is_same_v<P, Vec2>) {
                pts.push_back({p.X(), p.Y()});
            } else {
                pts.push_back(xyz(p));
            }
        }
        lines.push_back(std::move(pts));
    }
}

// The one face of `s` nearest the point, and its surface parameters there. `within` below zero
// sets no limit on how far the face may lie.
struct Placed {
    bool found = false;
    TopoDS_Face face;
    occ::handle<Geom_Surface> surface;
    double u = 0;
    double v = 0;
};

Placed place(const Shape& s, const Vec3& p, double within) {
    const gp_Pnt at = pnt(p);
    const TopoDS_Vertex vertex = BRepBuilderAPI_MakeVertex(at).Vertex();
    std::vector<std::pair<double, TopoDS_Face>> faces;
    for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face face = TopoDS::Face(ex.Current());
        faces.emplace_back(BRepExtrema_DistShapeShape(face, vertex).Value(), face);
    }
    if (faces.empty()) {
        return {};
    }
    double nearest = faces[0].first;
    for (const auto& entry : faces) {
        nearest = std::min(nearest, entry.first);
    }
    if (within >= 0 && nearest > within) {
        return {};
    }
    std::vector<TopoDS_Face> near;
    for (const auto& entry : faces) {
        if (entry.first <= nearest + 1e-6) {
            near.push_back(entry.second);
        }
    }
    if (near.size() != 1) {
        throw Refusal::unsupported("a point on an edge or a vertex, where several faces meet");
    }
    Placed out;
    out.found = true;
    out.face = near[0];
    out.surface = BRep_Tool::Surface(near[0]);
    GeomAPI_ProjectPointOnSurf(at, out.surface).LowerDistanceParameters(out.u, out.v);
    return out;
}

// Gives each triangle meeting a degenerate edge of the face, such as a cone's apex or a ball's
// pole, its own vertex there. Its normal is OpenCascade's at the point of the edge across from the
// triangle's far side, on the iso the triangle's middle runs along, turned to face the way
// OpenCascade's normals at the triangle's other corners do.
void split_apexes(const TopoDS_Face& face,
                  const occ::handle<Poly_Triangulation>& tri,
                  int lo,
                  DisplayFace& out) {
    struct Span {
        double u0;
        double v0;
        double du;
        double dv;
    };
    std::map<std::uint32_t, Span> line;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        if (!BRep_Tool::Degenerated(edge)) {
            continue;
        }
        TopLoc_Location loc;
        const occ::handle<Poly_PolygonOnTriangulation>& poly =
            BRep_Tool::PolygonOnTriangulation(edge, tri, loc);
        if (poly.IsNull()) {
            continue;
        }
        const NCollection_Array1<int>& nodes = poly->Nodes();
        const gp_Pnt2d first = tri->UVNode(nodes.Value(nodes.Lower()));
        const gp_Pnt2d last = tri->UVNode(nodes.Value(nodes.Upper()));
        for (int j = nodes.Lower(); j <= nodes.Upper(); j++) {
            line[static_cast<std::uint32_t>(nodes.Value(j) - lo)] =
                Span{first.X(), first.Y(), last.X() - first.X(), last.Y() - first.Y()};
        }
    }
    if (line.empty()) {
        return;
    }
    const BRepAdaptor_Surface surface(face);
    std::set<std::uint32_t> done;
    for (auto& t : out.triangles) {
        for (int k = 0; k < 3; k++) {
            const std::uint32_t a = t[k];
            const std::uint32_t others[2] = {t[(k + 1) % 3], t[(k + 2) % 3]};
            if (line.count(a) == 0 || line.count(others[0]) != 0 || line.count(others[1]) != 0) {
                continue;
            }
            const Span& span = line.at(a);
            const gp_Pnt2d m0 = tri->UVNode(static_cast<int>(others[0]) + lo);
            const gp_Pnt2d m1 = tri->UVNode(static_cast<int>(others[1]) + lo);
            const double mu = (m0.X() + m1.X()) / 2;
            const double mv = (m0.Y() + m1.Y()) / 2;
            const double length = span.du * span.du + span.dv * span.dv;
            const double t_along =
                length > 0 ? ((mu - span.u0) * span.du + (mv - span.v0) * span.dv) / length : 0.0;
            BRepLProp_SLProps props(surface,
                                    span.u0 + t_along * span.du,
                                    span.v0 + t_along * span.dv,
                                    2,
                                    Precision::Confusion());
            if (!props.IsNormalDefined()) {
                continue;
            }
            Vec3 n = xyz(props.Normal());
            const Vec3 way = plus(out.normals[others[0]], out.normals[others[1]]);
            if (dot(n, way) < 0) {
                n = {-n[0], -n[1], -n[2]};
            }
            if (done.count(a) != 0) {
                out.positions.push_back(out.positions[a]);
                out.normals.push_back(n);
                t[k] = static_cast<std::uint32_t>(out.positions.size() - 1);
            } else {
                done.insert(a);
                out.normals[a] = n;
            }
        }
    }
}

// Whether the two faces lie on one surface. A construction often gives each face a surface of its
// own, such as one plane for each side of an extruded profile, so the test compares the surfaces'
// geometry, for the elementary surfaces, and their handles otherwise.
bool same_surface(const TopoDS_Face& a, const TopoDS_Face& b) {
    if (BRep_Tool::Surface(a) == BRep_Tool::Surface(b)) {
        return true;
    }
    const BRepAdaptor_Surface sa(a);
    const BRepAdaptor_Surface sb(b);
    if (sa.GetType() != sb.GetType()) {
        return false;
    }
    const double d = Precision::Confusion();
    const double angle = Precision::Angular();
    // One axis line: parallel, either way, and through each other's location.
    const auto same_line = [&](const gp_Ax1& x, const gp_Ax1& y) {
        return x.IsParallel(y, angle) && gp_Lin(x).Distance(y.Location()) <= d;
    };
    switch (sa.GetType()) {
        case GeomAbs_Plane:
            return sa.Plane().Axis().IsParallel(sb.Plane().Axis(), angle) &&
                   sa.Plane().Distance(sb.Plane().Location()) <= d;
        case GeomAbs_Cylinder:
            return same_line(sa.Cylinder().Axis(), sb.Cylinder().Axis()) &&
                   std::abs(sa.Cylinder().Radius() - sb.Cylinder().Radius()) <= d;
        case GeomAbs_Cone:
            return same_line(sa.Cone().Axis(), sb.Cone().Axis()) &&
                   sa.Cone().Apex().Distance(sb.Cone().Apex()) <= d &&
                   std::abs(std::abs(sa.Cone().SemiAngle()) - std::abs(sb.Cone().SemiAngle())) <=
                       angle;
        case GeomAbs_Sphere:
            return sa.Sphere().Location().Distance(sb.Sphere().Location()) <= d &&
                   std::abs(sa.Sphere().Radius() - sb.Sphere().Radius()) <= d;
        case GeomAbs_Torus:
            return same_line(sa.Torus().Axis(), sb.Torus().Axis()) &&
                   sa.Torus().Location().Distance(sb.Torus().Location()) <= d &&
                   std::abs(sa.Torus().MajorRadius() - sb.Torus().MajorRadius()) <= d &&
                   std::abs(sa.Torus().MinorRadius() - sb.Torus().MinorRadius()) <= d;
        default:
            return false;
    }
}

// The outward unit normal of the face at the point of the edge at curve parameter t, or nothing
// where the surface has no one normal there.
std::optional<gp_Vec> outward_normal(const TopoDS_Edge& edge, const TopoDS_Face& face, double t) {
    const gp_Pnt2d uv = BRepAdaptor_Curve2d(edge, face).Value(t);
    BRepLProp_SLProps props(BRepAdaptor_Surface(face), uv.X(), uv.Y(), 1, Precision::Confusion());
    if (!props.IsNormalDefined()) {
        return std::nullopt;
    }
    gp_Vec n(props.Normal());
    if (face.Orientation() == TopAbs_REVERSED) {
        n.Reverse();
    }
    return n;
}

// The least and greatest signed angle between the outward normals of the two faces at the
// vertices of the edge's polyline, positive where the edge is convex. `edge` is oriented as `a`
// uses it, so `a` lies to the left of the edge's tangent seen against a's normal: the angle from
// a's normal to b's normal turns about that tangent, positive where b falls away from a.
std::optional<std::array<double, 2>> turn(const TopoDS_Edge& edge,
                                          const TopoDS_Face& a,
                                          const TopoDS_Face& b,
                                          const occ::handle<Poly_PolygonOnTriangulation>& poly) {
    if (!poly->HasParameters()) {
        return std::nullopt;
    }
    const BRepAdaptor_Curve curve(edge);
    std::optional<std::array<double, 2>> range;
    const NCollection_Array1<int>& ids = poly->Nodes();
    for (int j = ids.Lower(); j <= ids.Upper(); j++) {
        const double t = poly->Parameter(j);
        const auto na = outward_normal(edge, a, t);
        const auto nb = outward_normal(edge, b, t);
        gp_Pnt p;
        gp_Vec tangent;
        curve.D1(t, p, tangent);
        if (!na || !nb || tangent.Magnitude() <= Precision::Confusion()) {
            continue;
        }
        if (edge.Orientation() == TopAbs_REVERSED) {
            tangent.Reverse();
        }
        tangent.Normalize();
        const double angle = std::atan2(na->Crossed(*nb).Dot(tangent), na->Dot(*nb));
        if (!range) {
            range = std::array<double, 2>{angle, angle};
        } else {
            (*range)[0] = std::min((*range)[0], angle);
            (*range)[1] = std::max((*range)[1], angle);
        }
    }
    return range;
}

class Occt : public Kernel<Shape> {
  public:
    std::string name() const override {
        return adaptive() ? "opencascade" : "opencascade-fixed-gprop";
    }

    // The version and build of conda-forge's occt package this build links, which the build passes
    // in; the release of the headers otherwise.
    std::string version() const override {
#ifdef OCCT_PACKAGE
        return OCCT_PACKAGE;
#else
        return OCC_VERSION_COMPLETE;
#endif
    }

    double tolerance() const override {
        return 1e-7;
    }

    std::string configuration() const override {
        return std::string("fuzzy value and tolerances 1e-7 mm where a ") + "call takes one; " +
               (adaptive() ? "volumes and areas integrated adaptively to 1e-9"
                           : "volumes and areas by the fixed Gauss rule") +
               "; meshes at an angular deflection of 0.5 rad";
    }

    // Every function of the protocol except these, which the adapter does not offer:
    // - move_faces, fillet_g2, fillet_conic, fillet_setback, fillet_setback_g2, fillet_faces and
    //   full_round: OpenCascade has no such operation.
    // - slab, betti, interference and section_area: OpenCascade reaches each in its own way, where
    //   the protocol asks for one call of another shape. The adapter does not assemble them, so
    //   that what is scored is OpenCascade. It builds a slab from a surface, side faces and sewing;
    //   it counts shapes of each kind, where betti asks for pieces, tunnels and voids; it gives a
    //   common solid and a distance, where interference asks for one verdict on a pair; and its
    //   section gives curves, where section_area asks for the area they bound.
    std::vector<std::string> supports() const override {
        return {
            "box", "cylinder", "sphere", "torus", "cone",
            "extrude", "revolve", "screw", "loft", "loft_through", "sweep", "sweep_path",
            "union", "intersection", "difference", "union_all", "difference_all",
            "fillet", "fillet_radii", "fillet_variable",
            "chamfer", "chamfer_two", "chamfer_distances",
            "offset", "offset_faces", "delete_faces", "thicken", "shell", "draft",
            "write_step", "sew", "read_step",
            "volume", "area", "classify", "closest", "normal", "curvature", "check", "centroid",
            "inertia", "distance", "section", "project", "bounding_box", "split",
            "mesh", "display", "raycast", "transform", "affine",
        };
    }

    // OpenCascade's exceptions are refusals, and a standard exception that says the request is at
    // fault is refused as invalid. Running out of memory, an index out of range and an overflow are
    // faults of the adapter, so they stay crashes.
    std::optional<Refusal> refusal_for(std::exception_ptr e) const override {
        try {
            std::rethrow_exception(e);
        } catch (const Standard_Failure& f) {
            return Refusal::failed(std::string(f.ExceptionType()) + ": " + f.what());
        } catch (const std::bad_alloc&) {
            return std::nullopt;
        } catch (const std::out_of_range&) {
            return std::nullopt;
        } catch (const std::overflow_error&) {
            return std::nullopt;
        } catch (const std::invalid_argument& x) {
            return Refusal::invalid(x.what());
        } catch (const std::domain_error& x) {
            return Refusal::invalid(x.what());
        } catch (const std::length_error& x) {
            return Refusal::invalid(x.what());
        } catch (const std::range_error& x) {
            return Refusal::invalid(x.what());
        } catch (const std::exception& x) {
            return Refusal::failed(x.what());
        } catch (...) {
            return Refusal::failed("an unknown exception");
        }
    }

    Shape make(const Primitive& p) override {
        if (const auto* b = std::get_if<Box>(&p)) {
            return BRepPrimAPI_MakeBox(axes(b->frame), b->size[0], b->size[1], b->size[2]).Shape();
        }
        if (const auto* c = std::get_if<Cylinder>(&p)) {
            return BRepPrimAPI_MakeCylinder(axes(c->frame), c->radius, c->height).Shape();
        }
        if (const auto* s = std::get_if<Sphere>(&p)) {
            return BRepPrimAPI_MakeSphere(axes(s->frame), s->radius).Shape();
        }
        if (const auto* t = std::get_if<Torus>(&p)) {
            return BRepPrimAPI_MakeTorus(axes(t->frame), t->major, t->minor).Shape();
        }
        if (const auto* c = std::get_if<Cone>(&p)) {
            return BRepPrimAPI_MakeCone(axes(c->frame), c->bottom, c->top, c->height).Shape();
        }
        if (const auto* e = std::get_if<Extrude>(&p)) {
            const Frame& f = e->frame;
            const TopoDS_Face face = profile_face(e->profile, f.origin, f.x, f.y);
            return BRepPrimAPI_MakePrism(face, gp_Vec(f.z[0], f.z[1], f.z[2]) * e->height).Shape();
        }
        if (const auto* r = std::get_if<Revolve>(&p)) {
            const Frame& f = r->frame;
            const TopoDS_Face face = profile_face(r->profile, f.origin, f.x, f.z);
            return BRepPrimAPI_MakeRevol(face, gp_Ax1(pnt(f.origin), dir(f.z)), r->angle).Shape();
        }
        if (const auto* s = std::get_if<Screw>(&p)) {
            return helix(*s);
        }
        if (const auto* s = std::get_if<Sweep>(&p)) {
            return sweep(*s);
        }
        if (const auto* s = std::get_if<SweepPath>(&p)) {
            return sweep_path(*s);
        }
        if (const auto* l = std::get_if<Loft>(&p)) {
            const Frame& f = l->frame;
            BRepOffsetAPI_ThruSections loft(true, true);
            loft.AddWire(loop_wire(one_loop(l->profile), Plane{f.origin, f.x, f.y, 1.0}));
            const Vec3 top = along(f.origin, f.z, l->height);
            loft.AddWire(loop_wire(one_loop(l->top), Plane{top, f.x, f.y, 1.0}));
            loft.Build();
            return done(loft, "loft");
        }
        throw Refusal::unsupported("slab is not supported");
    }

    Shape unite(const Shape& a, const Shape& b) override {
        BRepAlgoAPI_Fuse algo(a, b);
        return done(algo, "union");
    }
    Shape intersection(const Shape& a, const Shape& b) override {
        BRepAlgoAPI_Common algo(a, b);
        return done(algo, "intersection");
    }
    Shape difference(const Shape& a, const Shape& b) override {
        BRepAlgoAPI_Cut algo(a, b);
        return done(algo, "difference");
    }

    Shape union_all(const Solids& solids) override {
        if (solids.empty()) {
            throw Refusal::invalid("union_all of no solids");
        }
        NCollection_List<Shape> args;
        NCollection_List<Shape> tools;
        args.Append(*solids[0]);
        for (std::size_t i = 1; i < solids.size(); i++) {
            tools.Append(*solids[i]);
        }
        BRepAlgoAPI_Fuse algo;
        algo.SetArguments(args);
        algo.SetTools(tools);
        algo.Build();
        return done(algo, "union_all");
    }

    Shape difference_all(const Shape& target, const Solids& rest) override {
        NCollection_List<Shape> args;
        NCollection_List<Shape> tools;
        args.Append(target);
        for (const Shape* t : rest) {
            tools.Append(*t);
        }
        BRepAlgoAPI_Cut algo;
        algo.SetArguments(args);
        algo.SetTools(tools);
        algo.Build();
        return done(algo, "difference_all");
    }

    double volume(const Shape& s) override {
        return mass(s).Mass();
    }

    double area(const Shape& s) override {
        GProp_GProps a;
        if (adaptive()) {
            BRepGProp::SurfaceProperties(s, a, 1e-9);
        } else {
            BRepGProp::SurfaceProperties(s, a);
        }
        return a.Mass();
    }

    std::vector<std::string> check(const Shape& s) override {
        if (!BRepCheck_Analyzer(s).IsValid()) {
            return {"BRepCheck_Analyzer: invalid"};
        }
        return {};
    }

    std::optional<Vec3> centroid(const Shape& s) override {
        if (!has_solid(s)) {
            return std::nullopt;
        }
        return xyz(mass(s).CentreOfMass());
    }

    std::array<Vec3, 3> inertia(const Shape& s) override {
        std::array<Vec3, 3> rows{};
        if (!has_solid(s)) {
            return rows;
        }
        const gp_Mat m = mass(s).MatrixOfInertia();
        for (int i = 0; i < 3; i++) {
            rows[i] = {m.Value(i + 1, 1), m.Value(i + 1, 2), m.Value(i + 1, 3)};
        }
        return rows;
    }

    Shape fillet(const Shape& s, const std::vector<Vec3>& edges, double radius) override {
        const std::vector<TopoDS_Edge> found = edges_at(s, edges);
        BRepFilletAPI_MakeFillet algo(s);
        return blend(algo, std::vector<double>(found.size(), radius), found, "fillet");
    }

    Shape fillet_radii(const Shape& s,
                       const std::vector<Vec3>& edges,
                       const std::vector<double>& radii) override {
        BRepFilletAPI_MakeFillet algo(s);
        return blend(algo, radii, edges_at(s, edges), "fillet_radii");
    }

    Shape chamfer(const Shape& s, const std::vector<Vec3>& edges, double d) override {
        const std::vector<TopoDS_Edge> found = edges_at(s, edges);
        BRepFilletAPI_MakeChamfer algo(s);
        return blend(algo, std::vector<double>(found.size(), d), found, "chamfer");
    }

    Shape chamfer_distances(const Shape& s,
                            const std::vector<Vec3>& edges,
                            const std::vector<double>& distances) override {
        BRepFilletAPI_MakeChamfer algo(s);
        return blend(algo, distances, edges_at(s, edges), "chamfer_distances");
    }

    Shape
    chamfer_two(const Shape& s, const Vec3& edge, const Vec3& face, double d1, double d2) override {
        const TopoDS_Edge e = edges_at(s, {edge})[0];
        const TopoDS_Face f = faces_at(s, {face})[0];
        BRepFilletAPI_MakeChamfer algo(s);
        algo.Add(d1, d2, e, f);
        algo.Build();
        return done(algo, "chamfer_two");
    }

    Shape fillet_variable(
        const Shape& s, const Vec3& edge, const Vec3& from, double r_from, double r_to) override {
        const TopoDS_Edge e = edges_at(s, {edge})[0];
        const gp_Pnt start = BRep_Tool::Pnt(TopExp::FirstVertex(e));
        const gp_Pnt end = BRep_Tool::Pnt(TopExp::LastVertex(e));
        const gp_Pnt f = pnt(from);
        double r1 = r_from;
        double r2 = r_to;
        if (start.Distance(f) > end.Distance(f)) {
            std::swap(r1, r2);
        }
        double u0 = 0;
        double u1 = 0;
        BRep_Tool::Range(e, u0, u1);
        // The function's radius runs evenly. OpenCascade's call on two radii runs them by an
        // S-shaped law, and its call on a law function fails, so the even run is given as radii at
        // 129 points: fewer leave the spline through them visibly off the line at its clamped ends.
        NCollection_Array1<gp_Pnt2d> marks(1, 129);
        for (int i = 0; i < 129; i++) {
            const double t = i / 128.0;
            marks.SetValue(
                i + 1, gp_Pnt2d(arithmetic::between(u0, u1, t), arithmetic::between(r1, r2, t)));
        }
        BRepFilletAPI_MakeFillet algo(s);
        algo.Add(marks, e);
        algo.Build();
        return done(algo, "fillet_variable");
    }

    Shape loft_through(const Frame& f,
                       const std::vector<Profile>& profiles,
                       const std::vector<double>& heights) override {
        BRepOffsetAPI_ThruSections algo(true, false);
        for (std::size_t i = 0; i < profiles.size(); i++) {
            const Plane plane{along(f.origin, f.z, heights[i]), f.x, f.y, 1.0};
            algo.AddWire(loop_wire(one_loop(profiles[i]), plane));
        }
        algo.Build();
        return done(algo, "loft_through");
    }

    std::string write_step(const Shape& s) override {
        const TemporaryDirectory d;
        STEPControl_Writer writer;
        writer.Transfer(s, STEPControl_AsIs);
        const std::string path = (d.path / "out.step").string();
        if (writer.Write(path.c_str()) != IFSelect_RetDone) {
            throw Refusal::failed("writing STEP failed");
        }
        std::ifstream file(path, std::ios::binary);
        std::stringstream text;
        text << file.rdbuf();
        return text.str();
    }

    Shape read_step(const std::string& text) override {
        return read_step_text(text);
    }

    Shape sew(const std::string& text, double tolerance) override {
        const Shape shape = read_step_text(text);
        BRepBuilderAPI_Sewing sewing(tolerance);
        for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
            sewing.Add(ex.Current());
        }
        sewing.Perform();
        const Shape sewn = sewing.SewedShape();
        std::vector<TopoDS_Shell> shells;
        for (TopExp_Explorer ex(sewn, TopAbs_SHELL); ex.More(); ex.Next()) {
            shells.push_back(TopoDS::Shell(ex.Current()));
        }
        if (shells.size() != 1) {
            throw Refusal::failed("sewing gave " + std::to_string(shells.size()) + " shells");
        }
        TopoDS_Solid solid = BRepBuilderAPI_MakeSolid(shells[0]).Solid();
        BRepLib::OrientClosedSolid(solid);
        return solid;
    }

    std::vector<Vec3> closest(const Shape& s, const std::vector<Vec3>& points) override {
        std::vector<Vec3> out;
        for (const Vec3& p : points) {
            const TopoDS_Vertex vertex = BRepBuilderAPI_MakeVertex(pnt(p)).Vertex();
            const BRepExtrema_DistShapeShape d(vertex, s);
            if (!d.IsDone() || d.NbSolution() < 1) {
                throw Refusal::failed("closest failed");
            }
            out.push_back(xyz(d.PointOnShape2(1)));
        }
        return out;
    }

    double distance(const Shape& a, const Shape& b) override {
        const BRepExtrema_DistShapeShape d(a, b);
        if (!d.IsDone()) {
            throw Refusal::failed("distance failed");
        }
        return d.Value();
    }

    Shape transform(const Shape& s, const Matrix& m) override {
        gp_Trsf t;
        t.SetValues(m[0][0],
                    m[0][1],
                    m[0][2],
                    m[0][3],
                    m[1][0],
                    m[1][1],
                    m[1][2],
                    m[1][3],
                    m[2][0],
                    m[2][1],
                    m[2][2],
                    m[2][3]);
        BRepBuilderAPI_Transform algo(s, t, true);
        return done(algo, "transform");
    }

    Shape affine(const Shape& s, const Matrix& m) override {
        gp_GTrsf g;
        g.SetVectorialPart(gp_Mat(
            m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]));
        g.SetTranslationPart(gp_XYZ(m[0][3], m[1][3], m[2][3]));
        BRepBuilderAPI_GTransform algo(s, g, true);
        return done(algo, "affine");
    }

    Shape delete_faces(const Shape& s, const std::vector<Vec3>& faces) override {
        BRepAlgoAPI_Defeaturing algo;
        algo.SetShape(s);
        for (const TopoDS_Face& f : faces_at(s, faces)) {
            algo.AddFaceToRemove(f);
        }
        algo.SetRunParallel(false);
        algo.Build();
        return done(algo, "delete_faces");
    }

    Shape thicken(const Shape& s, const std::vector<Vec3>& faces, double t) override {
        const std::vector<TopoDS_Face> found = faces_at(s, faces);
        if (found.size() != 1) {
            throw Refusal::unsupported("thickening " + std::to_string(found.size()) +
                                       " faces in one call is not supported");
        }
        BRepOffset_MakeOffset algo;
        algo.Initialize(found[0], t, 1e-7, BRepOffset_Skin, false, false, GeomAbs_Arc, true);
        algo.MakeOffsetShape();
        return done(algo, "thicken");
    }

    Shape offset_faces(const Shape& s, const std::vector<Vec3>& faces, double d) override {
        BRepOffset_MakeOffset algo;
        algo.Initialize(s, 0.0, 1e-7, BRepOffset_Skin, true, false, GeomAbs_Intersection, false);
        for (const TopoDS_Face& face : faces_at(s, faces)) {
            algo.SetOffsetOnFace(face, d);
        }
        algo.MakeOffsetShape();
        return done(algo, "offset_faces");
    }

    Shape offset(const Shape& s, double d) override {
        BRepOffsetAPI_MakeOffsetShape algo;
        algo.PerformByJoin(s, d, 1e-7, BRepOffset_Skin, false, false, GeomAbs_Arc);
        return done(algo, "offset");
    }

    Shape shell(const Shape& s, const std::vector<Vec3>& open, double t) override {
        if (open.empty()) {
            throw Refusal::unsupported("a shell with no open face is not supported");
        }
        NCollection_List<Shape> removed;
        for (const TopoDS_Face& f : faces_at(s, open)) {
            removed.Append(f);
        }
        BRepOffsetAPI_MakeThickSolid algo;
        algo.MakeThickSolidByJoin(s, removed, -t, 1e-7);
        return done(algo, "shell");
    }

    Shape draft(const Shape& s,
                const std::vector<Vec3>& faces,
                const Vec3& origin,
                const Vec3& pull,
                double angle) override {
        BRepOffsetAPI_DraftAngle algo(s);
        const gp_Pln neutral(pnt(origin), dir(pull));
        for (const TopoDS_Face& f : faces_at(s, faces)) {
            algo.Add(f, dir(pull), angle, neutral);
            if (!algo.AddDone()) {
                throw Refusal::failed("a face could not be drafted");
            }
        }
        algo.Build();
        return done(algo, "draft");
    }

    std::pair<Shape, Shape> split(const Shape& s, const Vec3& origin, const Vec3& normal) override {
        BRepAlgoAPI_Splitter splitter;
        NCollection_List<Shape> args;
        NCollection_List<Shape> tools;
        args.Append(s);
        tools.Append(BRepBuilderAPI_MakeFace(gp_Pln(pnt(origin), dir(normal))).Face());
        splitter.SetArguments(args);
        splitter.SetTools(tools);
        splitter.Build();
        if (!splitter.IsDone()) {
            throw Refusal::failed("split failed");
        }
        TopoDS_Compound above;
        TopoDS_Compound below;
        BRep_Builder builder;
        builder.MakeCompound(above);
        builder.MakeCompound(below);
        for (TopExp_Explorer ex(splitter.Shape(), TopAbs_SOLID); ex.More(); ex.Next()) {
            GProp_GProps props;
            BRepGProp::VolumeProperties(ex.Current(), props);
            const double side = arithmetic::height_above(xyz(props.CentreOfMass()), origin, normal);
            builder.Add(side > 0 ? above : below, ex.Current());
        }
        return {above, below};
    }

    std::array<Vec3, 2> bounding_box(const Shape& s) override {
        Bnd_Box box;
        BRepBndLib::AddOptimal(s, box, false, false);
        return {xyz(box.CornerMin()), xyz(box.CornerMax())};
    }

    std::vector<std::vector<Vec3>>
    section(const Shape& s, const Vec3& origin, const Vec3& normal, double tolerance) override {
        BRepAlgoAPI_Section sec(s, gp_Pln(pnt(origin), dir(normal)));
        sec.Build();
        if (!sec.IsDone()) {
            throw Refusal::failed("section failed");
        }
        std::vector<std::vector<Vec3>> pieces;
        sample_edges(sec.Shape(), tolerance, pieces);
        return pieces;
    }

    std::vector<std::optional<Vec3>> normal(const Shape& s,
                                            const std::vector<Vec3>& points) override {
        std::vector<std::optional<Vec3>> out;
        for (const Vec3& p : points) {
            const Placed found = place(s, p, 1e-6);
            if (!found.found) {
                out.emplace_back();
                continue;
            }
            GeomLProp_SLProps props(found.surface, found.u, found.v, 1, 1e-9);
            if (!props.IsNormalDefined()) {
                out.emplace_back();
                continue;
            }
            const gp_Dir n = props.Normal();
            const double sign = found.face.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
            out.emplace_back(Vec3{sign * n.X(), sign * n.Y(), sign * n.Z()});
        }
        return out;
    }

    std::vector<std::optional<Vec2>> curvature(const Shape& s,
                                               const std::vector<Vec3>& points) override {
        std::vector<std::optional<Vec2>> out;
        for (const Vec3& p : points) {
            const Placed found = place(s, p, -1.0);
            if (!found.found) {
                out.emplace_back();
                continue;
            }
            GeomLProp_SLProps props(found.surface, found.u, found.v, 2, 1e-9);
            if (!props.IsCurvatureDefined()) {
                out.emplace_back();
                continue;
            }
            const double sign = found.face.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
            double low = -sign * props.MaxCurvature();
            double high = -sign * props.MinCurvature();
            if (high < low) {
                std::swap(low, high);
            }
            out.emplace_back(Vec2{high, low});
        }
        return out;
    }

    Projection project(const Shape& s, const Vec3& direction, double tolerance) override {
        const Vec3 d = unit(direction);
        const Vec3 across_z = cross(d, {0.0, 0.0, 1.0});
        const Vec3 u = unit(dot(across_z, across_z) < 1e-12 ? cross(d, {1.0, 0.0, 0.0}) : across_z);
        occ::handle<HLRBRep_Algo> algo = new HLRBRep_Algo();
        algo->Add(s);
        const gp_Ax2 view(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(-d[0], -d[1], -d[2]), dir(u));
        algo->Projector(HLRAlgo_Projector(view));
        algo->Update();
        algo->Hide();
        HLRBRep_HLRToShape shapes(algo);
        const auto drawn = [&](const Shape& sharp, const Shape& outline) {
            std::vector<std::vector<Vec2>> lines;
            for (const Shape* c : {&sharp, &outline}) {
                if (c->IsNull()) {
                    continue;
                }
                BRepLib::BuildCurves3d(*c);
                sample_edges(*c, tolerance, lines);
            }
            return lines;
        };
        Projection out;
        out.visible = drawn(shapes.VCompound(), shapes.OutLineVCompound());
        out.hidden = drawn(shapes.HCompound(), shapes.OutLineHCompound());
        return out;
    }

    std::vector<double>
    raycast(const Shape& s, const Vec3& origin, const Vec3& direction) override {
        std::vector<double> out;
        if (!has_solid(s)) {
            return out;
        }
        IntCurvesFace_ShapeIntersector hits;
        hits.Load(s, 1e-7);
        hits.Perform(gp_Lin(pnt(origin), dir(direction)), 0.0, 1e9);
        for (int i = 1; i <= hits.NbPnt(); i++) {
            out.push_back(hits.WParameter(i));
        }
        return out;
    }

    Mesh mesh(const Shape& s, double tolerance) override {
        BRepMesh_IncrementalMesh(s, tolerance, false, 0.5, true);
        Mesh out;
        for (TopExp_Explorer fx(s, TopAbs_FACE); fx.More(); fx.Next()) {
            const TopoDS_Face face = TopoDS::Face(fx.Current());
            TopLoc_Location loc;
            const occ::handle<Poly_Triangulation> tri = BRep_Tool::Triangulation(face, loc);
            if (tri.IsNull()) {
                throw Refusal::failed("a face has no triangulation");
            }
            const gp_Trsf trsf = loc.Transformation();
            const auto first = static_cast<std::uint32_t>(out.positions.size());
            for (int i = 1; i <= tri->NbNodes(); i++) {
                out.positions.push_back(xyz(tri->Node(i).Transformed(trsf)));
            }
            const bool reversed = face.Orientation() == TopAbs_REVERSED;
            for (int i = 1; i <= tri->NbTriangles(); i++) {
                int a = 0;
                int b = 0;
                int c = 0;
                tri->Triangle(i).Get(a, b, c);
                const std::uint32_t t0 = first + a - 1;
                const std::uint32_t t1 = first + b - 1;
                const std::uint32_t t2 = first + c - 1;
                out.triangles.push_back(reversed ? std::array<std::uint32_t, 3>{t0, t2, t1}
                                                 : std::array<std::uint32_t, 3>{t0, t1, t2});
            }
        }
        return out;
    }

    Display display(const Shape& s, double tolerance) override {
        BRepMesh_IncrementalMesh(s, tolerance, false, 0.5, true);
        Display out;
        // The face of each entry of `out.faces`, in its order.
        std::vector<TopoDS_Face> drawn;
        for (RWMesh_FaceIterator it(s); it.More(); it.Next()) {
            const int lo = it.NodeLower();
            DisplayFace face;
            for (int i = lo; i <= it.NodeUpper(); i++) {
                face.positions.push_back(xyz(it.NodeTransformed(i)));
                face.normals.push_back(xyz(it.NormalTransformed(i)));
            }
            for (int i = it.ElemLower(); i <= it.ElemUpper(); i++) {
                int a = 0;
                int b = 0;
                int c = 0;
                it.TriangleOriented(i).Get(a, b, c);
                face.triangles.push_back({static_cast<std::uint32_t>(a - lo),
                                          static_cast<std::uint32_t>(b - lo),
                                          static_cast<std::uint32_t>(c - lo)});
            }
            split_apexes(it.Face(), it.Triangulation(), lo, face);
            out.faces.push_back(std::move(face));
            drawn.push_back(it.Face());
        }

        // Each edge once, with the faces it bounds.
        std::vector<std::pair<TopoDS_Edge, std::vector<TopoDS_Face>>> bounded;
        for (TopExp_Explorer fx(s, TopAbs_FACE); fx.More(); fx.Next()) {
            const TopoDS_Face face = TopoDS::Face(fx.Current());
            for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
                const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
                auto known = bounded.begin();
                while (known != bounded.end() && !known->first.IsSame(edge)) {
                    ++known;
                }
                if (known == bounded.end()) {
                    bounded.push_back({edge, {face}});
                    continue;
                }
                bool listed = false;
                for (const TopoDS_Face& f : known->second) {
                    listed = listed || f.IsSame(face);
                }
                if (!listed) {
                    known->second.push_back(face);
                }
            }
        }

        // A degenerate edge draws no line. A seam, closed on its one face, goes to `out.seams`;
        // every other edge goes to `out.edges`, with the faces it bounds and the turn between
        // them, and flagged as split where its two faces lie on one surface.
        for (const auto& [edge, faces] : bounded) {
            if (BRep_Tool::Degenerated(edge)) {
                continue;
            }
            const bool seam = BRep_Tool::IsClosed(edge, faces[0]);
            TopLoc_Location loc;
            const occ::handle<Poly_Triangulation> tri = BRep_Tool::Triangulation(faces[0], loc);
            if (tri.IsNull()) {
                continue;
            }
            const occ::handle<Poly_PolygonOnTriangulation>& poly =
                BRep_Tool::PolygonOnTriangulation(edge, tri, loc);
            if (poly.IsNull()) {
                continue;
            }
            const gp_Trsf trsf = loc.Transformation();
            const NCollection_Array1<int>& ids = poly->Nodes();
            DisplayEdge line;
            for (int j = ids.Lower(); j <= ids.Upper(); j++) {
                line.points.push_back(xyz(tri->Node(ids.Value(j)).Transformed(trsf)));
            }
            if (seam) {
                out.seams.push_back(std::move(line.points));
                continue;
            }
            for (const TopoDS_Face& f : faces) {
                for (std::size_t i = 0; i < drawn.size(); i++) {
                    if (drawn[i].IsSame(f)) {
                        line.faces.push_back(static_cast<std::uint32_t>(i));
                        break;
                    }
                }
            }
            if (faces.size() == 2) {
                line.turn = turn(edge, faces[0], faces[1], poly);
                line.split = same_surface(faces[0], faces[1]);
            }
            out.edges.push_back(std::move(line));
        }
        return out;
    }

    std::vector<Side> classify(const Shape& s, const std::vector<Vec3>& points) override {
        // The classifier places every point inside a shape with no solid, as an empty result is.
        if (!has_solid(s)) {
            return std::vector<Side>(points.size(), Side::Out);
        }
        BRepClass3d_SolidClassifier c(s);
        std::vector<Side> out;
        for (const Vec3& p : points) {
            c.Perform(pnt(p), 1e-7);
            switch (c.State()) {
            case TopAbs_IN:
                out.push_back(Side::In);
                break;
            case TopAbs_ON:
                out.push_back(Side::On);
                break;
            case TopAbs_OUT:
                out.push_back(Side::Out);
                break;
            default:
                throw Refusal::failed("the classifier could not place (" + Json(p[0]).dump() +
                                      ", " + Json(p[1]).dump() + ", " + Json(p[2]).dump() + ")");
            }
        }
        return out;
    }
};

} // namespace

int main() {
    // The STEP translators print their progress through the default messenger, which nothing reads.
    Message::DefaultMessenger()->RemovePrinters(STANDARD_TYPE(Message_PrinterOStream));
    // The replies go out on a copy of the standard output of their own, and the standard output
    // itself goes to stderr, so nothing else OpenCascade prints can land among them.
    std::FILE* replies = fdopen(dup(1), "w");
    if (replies == nullptr || dup2(2, 1) < 0) {
        std::perror("occt-native-adapter");
        return 1;
    }
    std::ios::sync_with_stdio(false);
    Occt kernel;
    serve(kernel, std::cin, replies);
    std::fclose(replies);
    return 0;
}
