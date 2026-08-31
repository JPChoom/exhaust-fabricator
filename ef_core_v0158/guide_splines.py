"""Editable soft guide splines for Header assisted routing.

Guide curves are ordinary non-rendering Blender CURVE objects parented to each
managed primary Route.  They never become exhaust geometry; the routing solver
samples them only as a soft directional preference.
"""
import bpy
from mathutils import Vector, Matrix
from mathutils.geometry import interpolate_bezier

GUIDE_FLAG = "exhaust_primary_guide"


def guide_for_route(route):
    if not route:
        return None
    for child in getattr(route, "children", ()):
        if child.type == 'CURVE' and bool(child.get(GUIDE_FLAG, False)):
            return child
    return None


def guide_objects(header):
    from .properties import header_primary_objects
    return [g for r in header_primary_objects(header) if (g := guide_for_route(r)) is not None]


def _target_local_for_route(header, route):
    """Current mapped collector connector expressed in Route local space."""
    from .properties import header_target_collector, collector_inlet_connectors, header_port_index, header_primary_objects
    from .geometry import route_end_treatment_spec, route_connection_hardware_spec
    collector = header_target_collector(header)
    if collector is None:
        # Fall back to current route endpoint.
        if "exhaust_connector_end" in route:
            return Vector(route["exhaust_connector_end"])
        return Vector((0.5, 0.0, 0.0))
    routes = header_primary_objects(header)
    try:
        idx = routes.index(route)
    except ValueError:
        idx = int(route.get("exhaust_header_primary_index", 0))
    ports = collector_inlet_connectors(collector)
    if not ports:
        return Vector((0.5, 0.0, 0.0))
    pi = header_port_index(header, idx, len(ports))
    if not (0 <= pi < len(ports)):
        return Vector((0.5, 0.0, 0.0))
    port = ports[pi]
    cp_world = collector.matrix_world @ Vector(port['point'])
    ct_world = (collector.matrix_world.to_3x3() @ Vector(port['tangent'])).normalized()
    try:
        end_spec = route_end_treatment_spec(route.exhaust_route, 'end')
        end_hw = route_connection_hardware_spec(route.exhaust_route, 'end', end_spec)
        ext = float(end_spec['extension'] + end_hw['extension'])
    except Exception:
        ext = 0.0
    cp_world = cp_world - ct_world * ext
    return route.matrix_world.inverted() @ cp_world


def create_or_reset_guide(header, route):
    guide = guide_for_route(route)
    if guide is None:
        curve = bpy.data.curves.new(f"{route.name}_Guide_Curve", type='CURVE')
        curve.dimensions = '3D'
        curve.resolution_u = 12
        curve.render_resolution_u = 12
        curve.bevel_depth = 0.0
        curve.extrude = 0.0
        guide = bpy.data.objects.new(f"{route.name}_Guide", curve)
        collection = route.users_collection[0] if route.users_collection else bpy.context.collection
        collection.objects.link(guide)
        guide.parent = route
        guide.matrix_parent_inverse = Matrix.Identity(4)
        guide.location = (0.0, 0.0, 0.0)
        guide.rotation_euler = (0.0, 0.0, 0.0)
        guide.scale = (1.0, 1.0, 1.0)
        guide[GUIDE_FLAG] = True
        guide["exhaust_header_owner"] = header.name
        guide["exhaust_primary_index"] = int(route.get("exhaust_header_primary_index", 0))
        guide.hide_render = True
        guide.show_in_front = True
        guide.display_type = 'WIRE'
        guide.color = (1.0, 0.25, 0.05, 1.0)
    curve = guide.data
    curve.splines.clear()
    spline = curve.splines.new('BEZIER')
    spline.bezier_points.add(3)
    target = _target_local_for_route(header, route)
    # A slightly eased initial sketch.  Users can reshape/subdivide freely.
    p0 = Vector((0.0, 0.0, 0.0))
    p3 = target.copy()
    delta = p3 - p0
    pts = (p0, p0 + delta * (1.0/3.0), p0 + delta * (2.0/3.0), p3)
    for bp, co in zip(spline.bezier_points, pts):
        bp.co = co
        bp.handle_left_type = 'AUTO'
        bp.handle_right_type = 'AUTO'
    spline.resolution_u = 12
    return guide


def create_or_reset_all(header):
    from .properties import header_primary_objects
    return [create_or_reset_guide(header, route) for route in header_primary_objects(header)]


def remove_all(header):
    removed = 0
    for guide in list(guide_objects(header)):
        data = guide.data
        bpy.data.objects.remove(guide, do_unlink=True)
        if data and data.users == 0:
            bpy.data.curves.remove(data)
        removed += 1
    return removed



def _point_segment_distance(point, a, b):
    """Distance from *point* to finite segment a-b in local 3D space."""
    p = Vector(point); a = Vector(a); b = Vector(b)
    ab = b - a
    denom = ab.length_squared
    if denom <= 1.0e-20:
        return (p - a).length
    t = max(0.0, min(1.0, (p - a).dot(ab) / denom))
    return (p - (a + ab * t)).length


def _rdp(points, tolerance):
    """Small 3D Ramer-Douglas-Peucker simplifier for editable guide points."""
    pts = [Vector(p) for p in points]
    if len(pts) <= 2:
        return [p.copy() for p in pts]
    tol = max(1.0e-9, float(tolerance))

    keep = {0, len(pts) - 1}
    stack = [(0, len(pts) - 1)]
    while stack:
        i0, i1 = stack.pop()
        if i1 <= i0 + 1:
            continue
        a, b = pts[i0], pts[i1]
        best_i = -1
        best_d = -1.0
        for i in range(i0 + 1, i1):
            d = _point_segment_distance(pts[i], a, b)
            if d > best_d:
                best_d = d
                best_i = i
        if best_i >= 0 and best_d > tol:
            keep.add(best_i)
            stack.append((i0, best_i))
            stack.append((best_i, i1))
    return [pts[i].copy() for i in sorted(keep)]


def create_or_update_guide_from_current(header, route):
    """Make a primary guide follow the Route's current modeled centerline.

    This intentionally creates an editable *soft* approximation, not an exact
    conversion of every mesh/arc sample.  Auto-generated routes can therefore
    be captured as a useful designer starting point without producing dozens of
    difficult-to-edit Bezier controls.
    """
    from .geometry import route_points

    guide = guide_for_route(route)
    if guide is None:
        # Reuse the normal creator so ownership/display metadata stays identical.
        guide = create_or_reset_guide(header, route)

    # "Splines to Current" is an explicit resync operation, so discard any
    # accidental object-level transform/reparenting and put the guide back in
    # the Route's local coordinate system before writing the new control points.
    guide.parent = route
    guide.matrix_parent_inverse = Matrix.Identity(4)
    guide.location = (0.0, 0.0, 0.0)
    guide.rotation_euler = (0.0, 0.0, 0.0)
    guide.scale = (1.0, 1.0, 1.0)

    raw = [Vector(p) for p in route_points(route.exhaust_route)]
    if len(raw) < 2:
        raw = [Vector((0.0, 0.0, 0.0)), _target_local_for_route(header, route)]

    # Keep enough curvature to resemble the current primary while avoiding an
    # unwieldy control point at every mandrel-bend resolution sample.
    try:
        od = max(1.0e-6, float(route.exhaust_route.outside_diameter))
    except Exception:
        od = 0.05
    tolerance = max(1.0e-5, od * 0.03)
    pts = _rdp(raw, tolerance)
    if len(pts) < 2:
        pts = [raw[0].copy(), raw[-1].copy()]

    curve = guide.data
    curve.splines.clear()
    spline = curve.splines.new('BEZIER')
    spline.bezier_points.add(len(pts) - 1)
    for bp, co in zip(spline.bezier_points, pts):
        bp.co = co
        bp.handle_left_type = 'AUTO'
        bp.handle_right_type = 'AUTO'
    spline.resolution_u = 12

    guide["exhaust_header_owner"] = header.name
    guide["exhaust_primary_index"] = int(route.get("exhaust_header_primary_index", 0))
    guide["exhaust_guide_source"] = "CURRENT_ROUTE"
    return guide


def guides_to_current(header):
    """Create/update every primary guide from its currently modeled Route."""
    from .properties import header_primary_objects
    return [create_or_update_guide_from_current(header, route) for route in header_primary_objects(header)]

def sample_guide_local(route, samples_per_segment=8):
    """Sample the Route-owned guide in Route-local coordinates."""
    guide = guide_for_route(route)
    if guide is None or guide.type != 'CURVE' or not guide.data.splines:
        return []
    out = []
    # guide is normally parented to route, but use matrices so manual reparenting
    # does not silently invalidate solver guidance.
    to_route = route.matrix_world.inverted() @ guide.matrix_world
    for spline in guide.data.splines:
        if spline.type == 'BEZIER':
            bps = spline.bezier_points
            if len(bps) < 2:
                continue
            seg_count = len(bps) if spline.use_cyclic_u else len(bps) - 1
            for i in range(seg_count):
                a = bps[i]
                b = bps[(i + 1) % len(bps)]
                pts = interpolate_bezier(a.co, a.handle_right, b.handle_left, b.co, max(3, int(samples_per_segment)))
                if out and pts:
                    pts = pts[1:]
                out.extend(to_route @ Vector(p) for p in pts)
        elif spline.type in {'POLY', 'NURBS'}:
            out.extend(to_route @ Vector((p.co.x, p.co.y, p.co.z)) for p in spline.points)
    return out


def resample_fractions(points, fractions=(0.25, 0.5, 0.75)):
    pts = [Vector(p) for p in (points or [])]
    if len(pts) < 2:
        return []
    lengths = [0.0]
    for a, b in zip(pts[:-1], pts[1:]):
        lengths.append(lengths[-1] + (b-a).length)
    total = lengths[-1]
    if total <= 1.0e-12:
        return [pts[0].copy() for _ in fractions]
    result = []
    for f in fractions:
        d = max(0.0, min(1.0, float(f))) * total
        j = 0
        while j + 1 < len(lengths) and lengths[j+1] < d:
            j += 1
        if j + 1 >= len(pts):
            result.append(pts[-1].copy()); continue
        span = max(1.0e-12, lengths[j+1] - lengths[j])
        t = (d - lengths[j]) / span
        result.append(pts[j].lerp(pts[j+1], t))
    return result


def guide_probe_points(route):
    return resample_fractions(sample_guide_local(route), (0.25, 0.5, 0.75))


def guide_preferred_clocking(route):
    """Estimate dogleg-plane clocking from the guide's midpoint direction."""
    probes = guide_probe_points(route)
    if not probes:
        return None
    mid = probes[1]
    lateral = Vector((0.0, mid.y, mid.z))
    if lateral.length < 1.0e-8:
        return None
    # Solver clocking zero points +Z; atan2(y,z) rotates +Z around +X.
    return float(__import__('math').atan2(lateral.y, lateral.z))
