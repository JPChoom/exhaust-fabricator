import math
import bpy
from mathutils import Vector, Matrix
from mathutils.geometry import tessellate_polygon, delaunay_2d_cdt
from .math_core import packed_radial_radius, route_centerline_length

_EPS = 1.0e-9
_rebuild_guard = set()


def _safe_normal(v: Vector, fallback=Vector((0.0, 0.0, 1.0))):
    if v.length < _EPS:
        return fallback.copy()
    return v.normalized()


def _rotation(axis: Vector, angle: float):
    return Matrix.Rotation(angle, 4, _safe_normal(axis))


def _segment_bend_basis(tangent, up, seg):
    """Return bend plane direction and turn axis using route clocking semantics."""
    clock_rot = _rotation(tangent, seg.clocking)
    bend_dir = _safe_normal(clock_rot @ up)
    bend_dir = _safe_normal(bend_dir - tangent * bend_dir.dot(tangent), up)
    sign = 1.0 if seg.angle >= 0.0 else -1.0
    signed_dir = bend_dir * sign
    axis = _safe_normal(tangent.cross(signed_dir))
    return bend_dir, signed_dir, axis


def _route_pie_layout(p, tangent, up, seg):
    """Pie-cut layout in the current route frame.

    Returns boundary centers, straight-section axes, miter-plane normals,
    turn axis, end tangent/up, angle per weld, and section C/L length.
    """
    sections = max(2, int(getattr(seg, "pie_sections", 6)))
    total = abs(float(seg.angle))
    if total < _EPS:
        return [p.copy(), p.copy()], [tangent.copy()], [tangent.copy(), tangent.copy()], tangent.cross(up), tangent.copy(), up.copy(), 0.0, 0.0

    bend_dir, _signed_dir, axis = _segment_bend_basis(tangent, up, seg)
    delta = total / (sections - 1)
    radius = max(1.0e-6, float(seg.radius))
    section_length = max(1.0e-6, 2.0 * radius * math.sin(delta * 0.5))

    axes = []
    for i in range(sections):
        axes.append(_safe_normal(_rotation(axis, delta * i) @ tangent, tangent))

    centers = [p.copy()]
    for d in axes:
        centers.append(centers[-1] + d * section_length)

    planes = [axes[0].copy()]
    for i in range(1, sections):
        planes.append(_safe_normal(axes[i - 1] + axes[i], axes[i - 1]))
    planes.append(axes[-1].copy())

    rot_end = _rotation(axis, total)
    tangent_end = _safe_normal(rot_end @ tangent, tangent)
    up_end = _safe_normal(rot_end @ bend_dir)
    up_end = _safe_normal(up_end - tangent_end * up_end.dot(tangent_end), Vector((0.0, 0.0, 1.0)))
    signed_delta = delta if seg.angle >= 0.0 else -delta
    return centers, axes, planes, axis, tangent_end, up_end, signed_delta, section_length


def route_points(settings):
    """Create a sampled 3D centerline from straight/bend segments.

    Mandrel bends follow a circular arc. Pie-cut bends follow the actual
    polygonal centerline of their straight mitered tube sections.
    """
    p = Vector((0.0, 0.0, 0.0))
    tangent = Vector((1.0, 0.0, 0.0))
    up = Vector((0.0, 0.0, 1.0))
    points = [p.copy()]

    for seg in settings.segments:
        if seg.kind == 'STRAIGHT':
            length = max(0.0, seg.length)
            if length > _EPS:
                p = p + tangent * length
                points.append(p.copy())
            continue

        angle = abs(float(seg.angle))
        if angle < _EPS:
            continue

        if getattr(seg, 'bend_style', 'MANDREL') == 'PIE_CUT':
            centers, _axes, _planes, _axis, tangent, up, _delta, _section_length = _route_pie_layout(p, tangent, up, seg)
            for q in centers[1:]:
                points.append(q.copy())
            p = centers[-1].copy()
            continue

        radius = max(seg.radius, 1.0e-6)
        bend_dir, signed_dir, axis = _segment_bend_basis(tangent, up, seg)
        center = p + signed_dir * radius
        radial0 = p - center
        steps = max(2, int(seg.resolution))

        for i in range(1, steps + 1):
            a = angle * (i / steps)
            rot = _rotation(axis, a)
            points.append((center + (rot @ radial0)).copy())

        rot_end = _rotation(axis, angle)
        tangent = _safe_normal(rot_end @ tangent)
        up = _safe_normal(rot_end @ bend_dir)
        up = _safe_normal(up - tangent * up.dot(tangent), Vector((0.0, 0.0, 1.0)))
        p = points[-1].copy()

    if len(points) == 1:
        points.append(Vector((0.001, 0.0, 0.0)))
    return points

def route_seam_frames(settings, include_end=True):
    """Return exact local-space frames at each route segment boundary."""
    p = Vector((0.0, 0.0, 0.0))
    tangent = Vector((1.0, 0.0, 0.0))
    up = Vector((0.0, 0.0, 1.0))
    frames = []

    def append_frame():
        t = _safe_normal(tangent, Vector((1.0, 0.0, 0.0)))
        n = _safe_normal(up - t * up.dot(t), Vector((0.0, 0.0, 1.0)))
        if abs(t.dot(n)) > 0.999:
            n = Vector((0.0, 1.0, 0.0))
            n = _safe_normal(n - t * n.dot(t))
        b = _safe_normal(t.cross(n), Vector((0.0, 1.0, 0.0)))
        n = _safe_normal(b.cross(t), n)
        if not frames or (p - frames[-1][0]).length > _EPS:
            frames.append((p.copy(), t.copy(), n.copy(), b.copy()))

    append_frame()

    for seg in settings.segments:
        if seg.kind == 'STRAIGHT':
            length = max(0.0, seg.length)
            if length > _EPS:
                p = p + tangent * length
            append_frame()
            continue

        angle = abs(float(seg.angle))
        if angle < _EPS:
            append_frame()
            continue

        if getattr(seg, 'bend_style', 'MANDREL') == 'PIE_CUT':
            centers, _axes, _planes, _axis, tangent, up, _delta, _section_length = _route_pie_layout(p, tangent, up, seg)
            p = centers[-1].copy()
            append_frame()
            continue

        radius = max(seg.radius, 1.0e-6)
        bend_dir, signed_dir, axis = _segment_bend_basis(tangent, up, seg)
        center = p + signed_dir * radius
        radial0 = p - center
        rot_end = _rotation(axis, angle)
        p = center + (rot_end @ radial0)
        tangent = _safe_normal(rot_end @ tangent)
        up = _safe_normal(rot_end @ bend_dir)
        up = _safe_normal(up - tangent * up.dot(tangent), Vector((0.0, 0.0, 1.0)))
        append_frame()

    if not include_end and len(frames) > 1:
        frames.pop()
    return frames

def route_segment_geometry(settings, index):
    """Return centerline points and exact START frame for one route segment."""
    if index < 0 or index >= len(settings.segments):
        return None

    p = Vector((0.0, 0.0, 0.0))
    tangent = Vector((1.0, 0.0, 0.0))
    up = Vector((0.0, 0.0, 1.0))

    def frame_at_current():
        t = _safe_normal(tangent, Vector((1.0, 0.0, 0.0)))
        n = _safe_normal(up - t * up.dot(t), Vector((0.0, 0.0, 1.0)))
        if abs(t.dot(n)) > 0.999:
            n = Vector((0.0, 1.0, 0.0))
            n = _safe_normal(n - t * n.dot(t))
        b = _safe_normal(t.cross(n), Vector((0.0, 1.0, 0.0)))
        n = _safe_normal(b.cross(t), n)
        return (p.copy(), t.copy(), n.copy(), b.copy())

    for seg_index, seg in enumerate(settings.segments):
        start_frame = frame_at_current()

        if seg.kind == 'STRAIGHT':
            start = p.copy()
            p = p + tangent * max(0.0, seg.length)
            seg_points = [start, p.copy()]
            if seg_index == index:
                return seg_points, start_frame
            continue

        angle = abs(float(seg.angle))
        start = p.copy()
        if angle < _EPS:
            if seg_index == index:
                return [start, start.copy()], start_frame
            continue

        if getattr(seg, 'bend_style', 'MANDREL') == 'PIE_CUT':
            centers, _axes, _planes, _axis, tangent, up, _delta, _section_length = _route_pie_layout(p, tangent, up, seg)
            p = centers[-1].copy()
            if seg_index == index:
                return [q.copy() for q in centers], start_frame
            continue

        radius = max(seg.radius, 1.0e-6)
        bend_dir, signed_dir, axis = _segment_bend_basis(tangent, up, seg)
        center = p + signed_dir * radius
        radial0 = p - center
        steps = max(2, int(seg.resolution))
        seg_points = [start]
        for i in range(1, steps + 1):
            a = angle * (i / steps)
            rot = _rotation(axis, a)
            seg_points.append((center + (rot @ radial0)).copy())

        rot_end = _rotation(axis, angle)
        tangent = _safe_normal(rot_end @ tangent)
        up = _safe_normal(rot_end @ bend_dir)
        up = _safe_normal(up - tangent * up.dot(tangent), Vector((0.0, 0.0, 1.0)))
        p = seg_points[-1].copy()
        if seg_index == index:
            return seg_points, start_frame

    return None

def _parallel_transport_frames(points):
    n = len(points)
    tangents = []
    for i in range(n):
        if i == 0:
            t = points[1] - points[0]
        elif i == n - 1:
            t = points[-1] - points[-2]
        else:
            t = points[i + 1] - points[i - 1]
        tangents.append(_safe_normal(t, Vector((1.0, 0.0, 0.0))))

    t0 = tangents[0]
    seed = Vector((0.0, 0.0, 1.0))
    if abs(t0.dot(seed)) > 0.95:
        seed = Vector((0.0, 1.0, 0.0))
    normal = _safe_normal(seed - t0 * seed.dot(t0))
    normals = [normal.copy()]
    binormals = [_safe_normal(t0.cross(normal))]

    for i in range(1, n):
        prev_t = tangents[i - 1]
        t = tangents[i]
        q = prev_t.rotation_difference(t)
        normal = q @ normals[-1]
        normal = _safe_normal(normal - t * normal.dot(t), normals[-1])
        binormal = _safe_normal(t.cross(normal), binormals[-1])
        normal = _safe_normal(binormal.cross(t), normal)
        normals.append(normal)
        binormals.append(binormal)
    return tangents, normals, binormals


def tube_mesh_data(points, outside_diameter, wall_thickness, sides, cap_start=True, cap_end=True):
    """Return verts/faces for a hollow swept circular tube.

    cap_start/cap_end control the visible annular cut-wall faces. Collector branches
    deliberately leave their downstream end open so the smooth merge body can overlap
    the branch without showing a blocking end ring.
    """
    sides = max(6, int(sides))
    ro = max(1.0e-6, outside_diameter * 0.5)
    wall = max(0.0, min(wall_thickness, ro * 0.999))
    ri = max(0.0, ro - wall)
    hollow = ri > 1.0e-6
    _, normals, binormals = _parallel_transport_frames(points)

    verts = []
    faces = []
    ring_stride = sides * (2 if hollow else 1)

    for p, normal, binormal in zip(points, normals, binormals):
        for j in range(sides):
            a = 2.0 * math.pi * j / sides
            radial = normal * math.cos(a) + binormal * math.sin(a)
            verts.append(tuple(p + radial * ro))
        if hollow:
            for j in range(sides):
                a = 2.0 * math.pi * j / sides
                radial = normal * math.cos(a) + binormal * math.sin(a)
                verts.append(tuple(p + radial * ri))

    ring_count = len(points)
    for r in range(ring_count - 1):
        a0 = r * ring_stride
        b0 = (r + 1) * ring_stride
        for j in range(sides):
            k = (j + 1) % sides
            faces.append((a0 + j, a0 + k, b0 + k, b0 + j))
            if hollow:
                ia = a0 + sides
                ib = b0 + sides
                faces.append((ia + j, ib + j, ib + k, ia + k))

    if hollow:
        # Annular cut-wall faces at requested open ends.
        requested = []
        if cap_start:
            requested.append((0, True))
        if cap_end:
            requested.append((ring_count - 1, False))
        for end_ring, reverse in requested:
            base = end_ring * ring_stride
            inner = base + sides
            for j in range(sides):
                k = (j + 1) % sides
                if reverse:
                    faces.append((base + j, inner + j, inner + k, base + k))
                else:
                    faces.append((base + j, base + k, inner + k, inner + j))
    return verts, faces



def _route_circle_loop(center, tangent, angular_ref, radius, sides):
    t = _safe_normal(tangent, Vector((1.0, 0.0, 0.0)))
    u = _safe_normal(angular_ref - t * angular_ref.dot(t), Vector((0.0, 0.0, 1.0)))
    if abs(t.dot(u)) > 0.999:
        u = Vector((0.0, 1.0, 0.0))
        u = _safe_normal(u - t * u.dot(t))
    v = _safe_normal(t.cross(u), Vector((0.0, 1.0, 0.0)))
    u = _safe_normal(v.cross(t), u)
    return [center + (u * math.cos(2.0 * math.pi * j / sides) + v * math.sin(2.0 * math.pi * j / sides)) * radius for j in range(sides)]


def _route_miter_loop(center, cylinder_axis, plane_normal, angular_ref, radius, sides):
    """Cylinder/plane intersection loop with a route-controlled vertex phase."""
    d = _safe_normal(cylinder_axis, Vector((1.0, 0.0, 0.0)))
    n = _safe_normal(plane_normal, d)
    u = _safe_normal(angular_ref - d * angular_ref.dot(d), Vector((0.0, 0.0, 1.0)))
    if abs(d.dot(u)) > 0.999:
        u = Vector((0.0, 1.0, 0.0))
        u = _safe_normal(u - d * u.dot(d))
    v = _safe_normal(d.cross(u), Vector((0.0, 1.0, 0.0)))
    u = _safe_normal(v.cross(d), u)
    denom = n.dot(d)
    if abs(denom) < 1.0e-6:
        denom = 1.0e-6 if denom >= 0.0 else -1.0e-6
    loop = []
    for j in range(sides):
        a = 2.0 * math.pi * j / sides
        radial = (u * math.cos(a) + v * math.sin(a)) * radius
        shift = -n.dot(radial) / denom
        loop.append(center + d * shift + radial)
    return loop


def _append_route_loop(verts, loop):
    ids = []
    for p in loop:
        ids.append(len(verts))
        verts.append(tuple(p))
    return ids


def _connect_route_rings(faces, a, b, inward=False):
    sides = len(a)
    for j in range(sides):
        k = (j + 1) % sides
        if inward:
            faces.append((a[j], b[j], b[k], a[k]))
        else:
            faces.append((a[j], a[k], b[k], b[j]))


def _settings_are_header_primary(settings):
    """True when a Route PropertyGroup belongs to a managed Header primary."""
    try:
        obj = settings.id_data
        header = obj.parent if obj else None
        hs = getattr(header, "exhaust_header", None) if header else None
        return bool(hs and hs.is_header and getattr(settings, "is_route", False))
    except Exception:
        return False


def route_end_treatment_spec(settings, which):
    """Resolve one Route end treatment into physical dimensions.

    Returns a dict with type, exposed OD/ID, transition length, straight collar
    length, and total axial extension.  Wall thickness is kept constant through
    the treatment.  A Slip Socket is sized from the route OD plus diametral
    clearance at the socket ID.
    """
    prefix = 'start' if which == 'start' else 'end'
    if _settings_are_header_primary(settings):
        treatment = 'PLAIN'
    else:
        treatment = getattr(settings, f"{prefix}_end_type", 'PLAIN')
    base_od = max(1.0e-6, float(settings.outside_diameter))
    base_ro = base_od * 0.5
    wall = max(0.0, min(float(settings.wall_thickness), base_ro * 0.999))
    min_od = max(2.0e-6, 2.0 * wall + 2.0e-6)

    if treatment == 'EXPANDED':
        exposed_od = max(base_od, float(getattr(settings, f"{prefix}_expanded_od", base_od)))
    elif treatment == 'REDUCED':
        requested = float(getattr(settings, f"{prefix}_reduced_od", base_od))
        exposed_od = min(base_od, max(min_od, requested))
    elif treatment == 'SLIP_SOCKET':
        clearance = max(0.0, float(getattr(settings, f"{prefix}_slip_clearance", 0.0)))
        # Female socket ID accepts a mating tube at nominal route OD.
        socket_id = base_od + clearance
        exposed_od = socket_id + 2.0 * wall
    else:
        treatment = 'PLAIN'
        exposed_od = base_od

    if treatment == 'PLAIN':
        transition = 0.0
        collar = 0.0
    else:
        transition = max(0.0, float(getattr(settings, f"{prefix}_transition_length", 0.0)))
        collar = max(0.0, float(getattr(settings, f"{prefix}_collar_length", 0.0)))
        # Avoid a completely coincident target/core pair when the diameter changes.
        if transition + collar < 1.0e-7 and abs(exposed_od - base_od) > 1.0e-7:
            transition = 1.0e-6

    exposed_id = max(0.0, exposed_od - 2.0 * wall)
    return {
        'type': treatment,
        'od': exposed_od,
        'id': exposed_id,
        'transition': transition,
        'collar': collar,
        'extension': transition + collar,
        'active': treatment != 'PLAIN',
    }



def route_connection_hardware_spec(settings, which, end_spec):
    """Resolve optional endpoint connection hardware into physical dimensions.

    v0.9.2 gives the automotive flat flanges their own outline dimensions:
    2-bolt uses A/B/C (height / width / bolt-center spacing) and 3-bolt uses
    A/B (height / width).  V-band and round weld flanges retain circular OD.
    """
    prefix = 'start' if which == 'start' else 'end'
    if _settings_are_header_primary(settings):
        kind = 'NONE'
    else:
        kind = getattr(settings, f"{prefix}_connection_type", 'NONE')
    exposed_od = max(1.0e-6, float(end_spec['od']))
    exposed_id = max(0.0, float(end_spec['id']))
    wall = max(0.0, (exposed_od - exposed_id) * 0.5)
    flange_od = max(exposed_od + 1.0e-6, float(getattr(settings, f"{prefix}_flange_od", exposed_od * 1.25)))

    result = {
        'type': kind,
        'active': kind != 'NONE',
        'flange_od': flange_od,
        'base_od': exposed_od,
        'base_id': exposed_id,
        'extension': 0.0,
        'connector_od': exposed_od,
        'connector_id': exposed_id,
    }
    if kind == 'VBAND':
        neck = max(0.0, float(getattr(settings, f"{prefix}_vband_neck_length", 0.0)))
        taper = max(1.0e-6, float(getattr(settings, f"{prefix}_vband_taper_length", 1.0e-6)))
        face = max(1.0e-6, float(getattr(settings, f"{prefix}_vband_face_width", 1.0e-6)))
        result.update(neck=neck, taper=taper, face=face, extension=neck + taper + face,
                      connector_od=flange_od, connector_id=exposed_id)
    elif kind == 'WELD_FLANGE':
        thickness = max(1.0e-6, float(getattr(settings, f"{prefix}_flange_thickness", 1.0e-6)))
        result.update(thickness=thickness, extension=thickness,
                      connector_od=flange_od, connector_id=exposed_id)
    elif kind == 'FLAT_2BOLT':
        thickness = max(1.0e-6, float(getattr(settings, f"{prefix}_flange_thickness", 1.0e-6)))
        hole_r = max(1.0e-6, float(getattr(settings, f"{prefix}_bolt_hole_diameter", exposed_od * 0.15)) * 0.5)
        height = max(exposed_od + 2.0e-6, float(getattr(settings, f"{prefix}_2bolt_height", exposed_od * 1.25)))
        width = max(height, float(getattr(settings, f"{prefix}_2bolt_width", exposed_od * 1.75)))
        spacing = max(2.0 * hole_r + 1.0e-6, float(getattr(settings, f"{prefix}_2bolt_spacing", exposed_od * 1.45)))
        phase = float(getattr(settings, f"{prefix}_bolt_phase", 0.0))
        # Guarantee enough material around each bolt hole without silently
        # shrinking the requested hole.
        ear_min = hole_r * 1.55
        width = max(width, spacing + 2.0 * ear_min)
        height = max(height, exposed_od + 2.0 * max(hole_r * 0.65, exposed_od * 0.04))
        result.update(thickness=thickness, extension=thickness, bolt_count=2,
                      bolt_hole_radius=hole_r, bolt_phase=phase,
                      flange_height=height, flange_width=width, bolt_spacing=spacing,
                      connector_od=max(width, height), connector_id=exposed_id)
    elif kind == 'FLAT_3BOLT':
        thickness = max(1.0e-6, float(getattr(settings, f"{prefix}_flange_thickness", 1.0e-6)))
        hole_r = max(1.0e-6, float(getattr(settings, f"{prefix}_bolt_hole_diameter", exposed_od * 0.15)) * 0.5)
        height = max(exposed_od + 2.0e-6, float(getattr(settings, f"{prefix}_3bolt_height", exposed_od * 1.45)))
        width = max(exposed_od + 2.0e-6, float(getattr(settings, f"{prefix}_3bolt_width", exposed_od * 1.45)))
        phase = float(getattr(settings, f"{prefix}_bolt_phase", 0.0))
        ear_min = hole_r * 1.60
        height = max(height, exposed_od + 2.0 * max(hole_r * 0.65, exposed_od * 0.04), 4.0 * ear_min)
        width = max(width, exposed_od + 2.0 * max(hole_r * 0.65, exposed_od * 0.04), 4.0 * ear_min)
        result.update(thickness=thickness, extension=thickness, bolt_count=3,
                      bolt_hole_radius=hole_r, bolt_phase=phase,
                      flange_height=height, flange_width=width,
                      connector_od=max(width, height), connector_id=exposed_id)
    else:
        result['type'] = 'NONE'
        result['active'] = False
        result['extension'] = 0.0
    return result

def collector_outlet_end_treatment_spec(settings):
    """Resolve the Collector's single downstream end treatment."""
    treatment = getattr(settings, 'outlet_end_type', 'PLAIN')
    base_od = max(1.0e-6, float(settings.outlet_od))
    base_ro = base_od * 0.5
    wall = max(0.0, min(float(settings.wall_thickness), base_ro * 0.999))
    min_od = max(2.0e-6, 2.0 * wall + 2.0e-6)
    if treatment == 'EXPANDED':
        exposed_od = max(base_od, float(getattr(settings, 'outlet_expanded_od', base_od)))
    elif treatment == 'REDUCED':
        exposed_od = min(base_od, max(min_od, float(getattr(settings, 'outlet_reduced_od', base_od))))
    elif treatment == 'SLIP_SOCKET':
        clearance = max(0.0, float(getattr(settings, 'outlet_slip_clearance', 0.0)))
        socket_id = base_od + clearance
        exposed_od = socket_id + 2.0 * wall
    else:
        treatment = 'PLAIN'
        exposed_od = base_od
    if treatment == 'PLAIN':
        transition = collar = 0.0
    else:
        transition = max(0.0, float(getattr(settings, 'outlet_transition_length', 0.0)))
        collar = max(0.0, float(getattr(settings, 'outlet_collar_length', 0.0)))
        if transition + collar < 1.0e-7 and abs(exposed_od - base_od) > 1.0e-7:
            transition = 1.0e-6
    exposed_id = max(0.0, exposed_od - 2.0 * wall)
    return {'type': treatment, 'od': exposed_od, 'id': exposed_id,
            'transition': transition, 'collar': collar, 'extension': transition + collar,
            'active': treatment != 'PLAIN'}


def collector_outlet_connection_hardware_spec(settings, end_spec):
    """Resolve Collector outlet hardware using the same dimensions as Route hardware."""
    kind = getattr(settings, 'outlet_connection_type', 'NONE')
    exposed_od = max(1.0e-6, float(end_spec['od']))
    exposed_id = max(0.0, float(end_spec['id']))
    flange_od = max(exposed_od + 1.0e-6, float(getattr(settings, 'outlet_flange_od', exposed_od * 1.25)))
    result = {'type': kind, 'active': kind != 'NONE', 'flange_od': flange_od,
              'base_od': exposed_od, 'base_id': exposed_id, 'extension': 0.0,
              'connector_od': exposed_od, 'connector_id': exposed_id}
    if kind == 'VBAND':
        neck = max(0.0, float(getattr(settings, 'outlet_vband_neck_length', 0.0)))
        taper = max(1.0e-6, float(getattr(settings, 'outlet_vband_taper_length', 1.0e-6)))
        face = max(1.0e-6, float(getattr(settings, 'outlet_vband_face_width', 1.0e-6)))
        result.update(neck=neck, taper=taper, face=face, extension=neck+taper+face,
                      connector_od=flange_od, connector_id=exposed_id)
    elif kind == 'WELD_FLANGE':
        thickness = max(1.0e-6, float(getattr(settings, 'outlet_flange_thickness', 1.0e-6)))
        result.update(thickness=thickness, extension=thickness, connector_od=flange_od, connector_id=exposed_id)
    elif kind == 'FLAT_2BOLT':
        thickness = max(1.0e-6, float(getattr(settings, 'outlet_flange_thickness', 1.0e-6)))
        hole_r = max(1.0e-6, float(getattr(settings, 'outlet_bolt_hole_diameter', exposed_od*0.15))*0.5)
        height = max(exposed_od+2.0e-6, float(getattr(settings, 'outlet_2bolt_height', exposed_od*1.25)))
        width = max(height, float(getattr(settings, 'outlet_2bolt_width', exposed_od*1.75)))
        spacing = max(2.0*hole_r+1.0e-6, float(getattr(settings, 'outlet_2bolt_spacing', exposed_od*1.45)))
        phase = float(getattr(settings, 'outlet_bolt_phase', 0.0))
        ear_min = hole_r * 1.55
        width = max(width, spacing + 2.0*ear_min)
        height = max(height, exposed_od + 2.0*max(hole_r*0.65, exposed_od*0.04))
        result.update(thickness=thickness, extension=thickness, bolt_count=2,
                      bolt_hole_radius=hole_r, bolt_phase=phase, flange_height=height, flange_width=width,
                      bolt_spacing=spacing, connector_od=max(width,height), connector_id=exposed_id)
    elif kind == 'FLAT_3BOLT':
        thickness = max(1.0e-6, float(getattr(settings, 'outlet_flange_thickness', 1.0e-6)))
        hole_r = max(1.0e-6, float(getattr(settings, 'outlet_bolt_hole_diameter', exposed_od*0.15))*0.5)
        height = max(exposed_od+2.0e-6, float(getattr(settings, 'outlet_3bolt_height', exposed_od*1.45)))
        width = max(exposed_od+2.0e-6, float(getattr(settings, 'outlet_3bolt_width', exposed_od*1.45)))
        phase = float(getattr(settings, 'outlet_bolt_phase', 0.0))
        ear_min = hole_r * 1.60
        height = max(height, exposed_od + 2.0*max(hole_r*0.65, exposed_od*0.04), 4.0*ear_min)
        width = max(width, exposed_od + 2.0*max(hole_r*0.65, exposed_od*0.04), 4.0*ear_min)
        result.update(thickness=thickness, extension=thickness, bolt_count=3,
                      bolt_hole_radius=hole_r, bolt_phase=phase, flange_height=height, flange_width=width,
                      connector_od=max(width,height), connector_id=exposed_id)
    else:
        result['type']='NONE'; result['active']=False; result['extension']=0.0
    return result


def _route_frame(tangent, angular_ref):
    t = _safe_normal(tangent, Vector((1.0, 0.0, 0.0)))
    u = _safe_normal(angular_ref - t * angular_ref.dot(t), Vector((0.0, 0.0, 1.0)))
    if abs(t.dot(u)) > 0.999:
        u = Vector((0.0, 1.0, 0.0))
        u = _safe_normal(u - t * u.dot(t))
    v = _safe_normal(t.cross(u), Vector((0.0, 1.0, 0.0)))
    u = _safe_normal(v.cross(t), u)
    return t, u, v


def _convex_hull_2d(points):
    """Monotonic-chain convex hull for local (y, z) flange-profile points."""
    pts = sorted(set((round(float(p[0]), 12), round(float(p[1]), 12)) for p in points))
    if len(pts) <= 2:
        return pts
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower = []
    for q in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 1.0e-12:
            lower.pop()
        lower.append(q)
    upper = []
    for q in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 1.0e-12:
            upper.pop()
        upper.append(q)
    return lower[:-1] + upper[:-1]


def _rotate_local_2d(y, z, angle):
    c, sn = math.cos(angle), math.sin(angle)
    return (y*c - z*sn, y*sn + z*c)


def _sample_local_circle(cy, cz, radius, segments):
    n = max(12, int(segments))
    return [(cy + radius*math.cos(2.0*math.pi*i/n),
             cz + radius*math.sin(2.0*math.pi*i/n)) for i in range(n)]


def _hardware_profile_2d(spec, base_ro, segment_hint):
    """Return automotive flange outer outline and bolt-hole specs in local Y/Z.

    The 2-bolt profile is the convex hull of a center body circle plus two bolt
    ear circles.  The A/B/C dimensions therefore directly control overall
    height, width, and bolt-center spacing like common exhaust-flange drawings.

    The 3-bolt profile is the convex hull of a center body and three bolt ears,
    producing the familiar rounded triangular automotive flange rather than a
    generic circular plate.
    """
    kind = spec.get('type')
    seg = max(32, int(segment_hint))
    phase = float(spec.get('bolt_phase', 0.0))

    if kind == 'WELD_FLANGE':
        R = max(base_ro + 1.0e-6, float(spec['flange_od']) * 0.5)
        return _sample_local_circle(0.0, 0.0, R, seg), []

    hole_r = float(spec.get('bolt_hole_radius', 0.0))
    holes = []
    cloud = []

    if kind == 'FLAT_2BOLT':
        A = float(spec['flange_height'])
        B = float(spec['flange_width'])
        C = float(spec['bolt_spacing'])
        ear_r = max(hole_r * 1.55, (B - C) * 0.5)
        body_r = max(A * 0.5, base_ro + max(hole_r * 0.55, base_ro * 0.045))
        # Keep the specified A/B extents authoritative unless tube/hole clearance
        # physically requires a slightly larger profile.
        B_eff = max(B, C + 2.0*ear_r, 2.0*body_r)
        ear_r = max(ear_r, (B_eff - C) * 0.5)
        cloud += _sample_local_circle(0.0, 0.0, body_r, seg)
        ear_seg = max(20, seg // 3)
        centers = [(-C*0.5, 0.0), (C*0.5, 0.0)]
        for cy, cz in centers:
            cloud += _sample_local_circle(cy, cz, ear_r, ear_seg)
            hy, hz = _rotate_local_2d(cy, cz, phase)
            holes.append({'y': hy, 'z': hz, 'radius': hole_r,
                          'segments': max(16, seg // 3)})

    elif kind == 'FLAT_3BOLT':
        A = float(spec['flange_height'])
        B = float(spec['flange_width'])
        ear_r = max(hole_r * 1.60, min(A, B) * 0.085)
        body_margin = max(hole_r * 0.55, base_ro * 0.045)
        body_r = max(base_ro + body_margin, min(A, B) * 0.30)
        cloud += _sample_local_circle(0.0, 0.0, body_r, seg)
        # One top ear and two lower ears.  Use an ellipse-scaled 120-degree
        # pattern so A/B independently control the rounded-triangle envelope
        # while maintaining a balanced bolt layout around the exhaust bore.
        sy = max(1.0e-6, (B*0.5 - ear_r) / math.sin(math.radians(60.0)))
        sz = max(1.0e-6, (A - 2.0*ear_r) / 1.5)
        centers = [
            (0.0, sz),
            (-math.sin(math.radians(60.0))*sy, -0.5*sz),
            ( math.sin(math.radians(60.0))*sy, -0.5*sz),
        ]
        ear_seg = max(20, seg // 3)
        for cy, cz in centers:
            cloud += _sample_local_circle(cy, cz, ear_r, ear_seg)
            hy, hz = _rotate_local_2d(cy, cz, phase)
            holes.append({'y': hy, 'z': hz, 'radius': hole_r,
                          'segments': max(16, seg // 3)})
    else:
        R = max(base_ro + 1.0e-6, float(spec.get('flange_od', 2.0*base_ro)) * 0.5)
        return _sample_local_circle(0.0, 0.0, R, seg), []

    # Rotate the complete plate profile with the bolt-pattern rotation.
    cloud = [_rotate_local_2d(y, z, phase) for y, z in cloud]
    return _convex_hull_2d(cloud), holes


def _point_in_poly_2d(point, poly):
    x, y = point
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if ((y1 > y) != (y2 > y)):
            den = (y2 - y1)
            if abs(den) < 1.0e-20:
                continue
            xin = (x2 - x1) * (y - y1) / den + x1
            if x < xin:
                inside = not inside
    return inside


def _append_cap_region_with_holes(verts, faces, center, tangent, angular_ref, outer_radius,
                                  inner_radius, outer_segments, inner_loop_ids=None,
                                  hole_specs=None, reverse=False, outer_points_local=None):
    """Add a planar flange face with a center opening and optional bolt holes.

    v0.9.2 uses the CDT only as a *constrained convex-hull triangulator* and
    removes triangles inside the explicit exhaust/bolt-hole loops afterward.
    This avoids the nested-face/hole classification path that proved fragile in
    the earlier flat-flange implementation while still preserving every hole
    boundary as a hard triangulation constraint.
    """
    t, u, v = _route_frame(tangent, angular_ref)
    center = Vector(center)
    outer_segments = max(24, int(outer_segments))

    if outer_points_local is None:
        outer_points_local = [
            (math.cos(2.0*math.pi*j/outer_segments)*outer_radius,
             math.sin(2.0*math.pi*j/outer_segments)*outer_radius)
            for j in range(outer_segments)
        ]
    outer_pts = [center + u*y + v*z for y, z in outer_points_local]
    outer_ids = _append_route_loop(verts, outer_pts)

    if inner_loop_ids is None:
        inner_segments = max(12, int(outer_segments // 2))
        inner_pts = [
            center + (u * math.cos(2.0 * math.pi * j / inner_segments)
                      + v * math.sin(2.0 * math.pi * j / inner_segments)) * inner_radius
            for j in range(inner_segments)
        ]
        inner_ids = _append_route_loop(verts, inner_pts)
    else:
        inner_ids = list(inner_loop_ids)
        inner_pts = [Vector(verts[i]) for i in inner_ids]

    hole_loops = []
    for hs in (hole_specs or []):
        hc = center + u * hs['y'] + v * hs['z']
        hseg = max(12, int(hs.get('segments', 20)))
        pts = [
            hc + (u * math.cos(2.0 * math.pi * j / hseg)
                  + v * math.sin(2.0 * math.pi * j / hseg)) * hs['radius']
            for j in range(hseg)
        ]
        hole_loops.append(_append_route_loop(verts, pts))

    in_coords = []
    input_to_global = []
    loops_local = []

    def add_loop(global_ids):
        loop = []
        for gid in global_ids:
            co = Vector(verts[gid]) - center
            loop.append(len(in_coords))
            in_coords.append(Vector((co.dot(u), co.dot(v))))
            input_to_global.append(gid)
        loops_local.append(loop)
        return loop

    add_loop(outer_ids)
    add_loop(inner_ids)
    for ids in hole_loops:
        add_loop(ids)

    constraint_edges = []
    for loop in loops_local:
        for j in range(len(loop)):
            constraint_edges.append((loop[j], loop[(j + 1) % len(loop)]))

    # Triangulate the constrained convex region.  Holes are filtered explicitly
    # after triangulation; their boundary edges remain immutable constraints.
    try:
        out_coords, _out_edges, out_faces, orig_verts, _orig_edges, _orig_faces = delaunay_2d_cdt(
            in_coords, constraint_edges, [], 0, 1.0e-9, True
        )
    except Exception:
        # Fallback uses Blender's polygon tessellator with reversed inner loops.
        polylines = [[Vector(verts[i]) for i in outer_ids]]
        polylines.append(list(reversed([Vector(verts[i]) for i in inner_ids])))
        for ids in hole_loops:
            polylines.append(list(reversed([Vector(verts[i]) for i in ids])))
        triangles = tessellate_polygon(polylines)
        coord_map = {}
        def k3(vec):
            return (round(vec.x, 10), round(vec.y, 10), round(vec.z, 10))
        for gid in outer_ids + inner_ids + [g for loop in hole_loops for g in loop]:
            coord_map.setdefault(k3(Vector(verts[gid])), gid)
        for tri in triangles:
            ids = tuple(coord_map[k3(Vector(q))] for q in tri)
            if len(set(ids)) >= 3:
                faces.append(tuple(reversed(ids)) if reverse else ids)
        return outer_ids, inner_ids, hole_loops

    out_to_global = []
    for oi, co2 in enumerate(out_coords):
        source_ids = orig_verts[oi] if oi < len(orig_verts) else []
        gid = None
        for src in source_ids:
            if 0 <= src < len(input_to_global):
                gid = input_to_global[src]
                break
        if gid is None:
            p3 = center + u * co2.x + v * co2.y
            gid = len(verts)
            verts.append(tuple(p3))
        out_to_global.append(gid)

    # Explicit 2D hole polygons in CDT output coordinates.
    hole_polys = []
    for loop in loops_local[1:]:
        hole_polys.append([(float(in_coords[i].x), float(in_coords[i].y)) for i in loop])

    for f in out_faces:
        if len(f) < 3:
            continue
        cx = sum(float(out_coords[i].x) for i in f) / len(f)
        cy = sum(float(out_coords[i].y) for i in f) / len(f)
        if any(_point_in_poly_2d((cx, cy), hp) for hp in hole_polys):
            continue
        ids = tuple(out_to_global[i] for i in f)
        if len(set(ids)) < 3:
            continue
        faces.append(tuple(reversed(ids)) if reverse else ids)

    return outer_ids, inner_ids, hole_loops


def _rounded_rect_profile_2d(min_y, max_y, min_z, max_z, radius, corner_segments):
    """CCW rounded rectangle in the local Y/Z plane."""
    width = max(1.0e-9, max_y - min_y)
    height = max(1.0e-9, max_z - min_z)
    r = max(0.0, min(float(radius), width * 0.5, height * 0.5))
    if r <= 1.0e-9:
        return [(max_y, max_z), (min_y, max_z), (min_y, min_z), (max_y, min_z)]
    n = max(1, int(corner_segments))
    result = []
    corners = [
        ((max_y-r, max_z-r), 0.0, 0.5*math.pi),
        ((min_y+r, max_z-r), 0.5*math.pi, math.pi),
        ((min_y+r, min_z+r), math.pi, 1.5*math.pi),
        ((max_y-r, min_z+r), 1.5*math.pi, 2.0*math.pi),
    ]
    for (cy, cz), a0, a1 in corners:
        for i in range(n + 1):
            if result and i == 0:
                continue
            a = a0 + (a1-a0) * (i / n)
            result.append((cy + r*math.cos(a), cz + r*math.sin(a)))
    return result


def _planar_plate_with_holes_mesh(outer_profile, hole_profiles, x_front, x_back):
    """Extrude one planar polygon with any number of circular/closed holes.

    The front/back plate faces are produced from one constrained triangulation,
    so every perimeter and port-bore loop is preserved exactly and the through
    hole walls share those boundary vertices.
    """
    outer = [(float(y), float(z)) for y, z in outer_profile]
    holes = [[(float(y), float(z)) for y, z in loop] for loop in hole_profiles]
    loops = [outer] + holes
    in_coords = []
    loop_indices = []
    for loop in loops:
        ids = []
        for y, z in loop:
            ids.append(len(in_coords))
            in_coords.append(Vector((y, z)))
        loop_indices.append(ids)
    edges = []
    for loop in loop_indices:
        for i in range(len(loop)):
            edges.append((loop[i], loop[(i+1) % len(loop)]))

    try:
        out_coords, _oe, out_faces, orig_verts, _orig_edges, _orig_faces = delaunay_2d_cdt(
            in_coords, edges, [], 0, 1.0e-9, True
        )
    except Exception:
        # This path is only a safety net.  Blender 4.2+ exposes the CDT used above.
        poly3 = [[Vector((0.0, y, z)) for y, z in outer]]
        poly3.extend(list(reversed([Vector((0.0, y, z)) for y, z in h])) for h in holes)
        tris = tessellate_polygon(poly3)
        unique = {}
        verts = []
        def get_idx(x, y, z):
            k=(round(x,10),round(y,10),round(z,10))
            if k not in unique:
                unique[k]=len(verts); verts.append((x,y,z))
            return unique[k]
        faces=[]
        for tri in tris:
            front=[get_idx(x_front,q.y,q.z) for q in tri]
            back=[get_idx(x_back,q.y,q.z) for q in reversed(tri)]
            faces.append(tuple(front)); faces.append(tuple(back))
        # Add side walls from exact input loops.
        for li, loop in enumerate(loops):
            fg=[get_idx(x_front,y,z) for y,z in loop]
            bg=[get_idx(x_back,y,z) for y,z in loop]
            for j in range(len(loop)):
                k=(j+1)%len(loop)
                if li == 0:
                    faces.append((bg[j], bg[k], fg[k], fg[j]))
                else:
                    faces.append((bg[k], bg[j], fg[j], fg[k]))
        return verts, faces

    # Create exact front/back copies of every CDT output coordinate.  Reuse the
    # same coordinate ordering on both planes, which guarantees matching side
    # boundaries and through-bore walls.
    verts=[]
    front=[]; back=[]
    for co in out_coords:
        front.append(len(verts)); verts.append((float(x_front), float(co.x), float(co.y)))
        back.append(len(verts)); verts.append((float(x_back), float(co.x), float(co.y)))

    # Map original input loop vertices to the corresponding CDT output indices.
    input_to_out={}
    for oi, sources in enumerate(orig_verts):
        for src in sources:
            if 0 <= src < len(in_coords):
                input_to_out.setdefault(src, oi)

    def signed_area_for_face(face):
        area=0.0
        for j in range(len(face)):
            a=out_coords[face[j]]; b=out_coords[face[(j+1)%len(face)]]
            area += float(a.x)*float(b.y)-float(b.x)*float(a.y)
        return 0.5*area

    hole_polys = holes
    faces=[]
    for f in out_faces:
        if len(f) < 3:
            continue
        cy=sum(float(out_coords[i].x) for i in f)/len(f)
        cz=sum(float(out_coords[i].y) for i in f)/len(f)
        if any(_point_in_poly_2d((cy,cz), hp) for hp in hole_polys):
            continue
        ids=list(f)
        if signed_area_for_face(ids) < 0.0:
            ids.reverse()
        faces.append(tuple(front[i] for i in ids))
        faces.append(tuple(back[i] for i in reversed(ids)))

    # Exact plate outer wall and all through-bore walls.
    for li, loop in enumerate(loop_indices):
        out_loop=[]
        for src in loop:
            oi=input_to_out.get(src)
            if oi is None:
                # Find a numerically identical output coordinate if CDT omitted
                # provenance for this particular constrained input vertex.
                q=in_coords[src]
                oi=min(range(len(out_coords)), key=lambda k: (out_coords[k]-q).length_squared)
            out_loop.append(oi)
        for j in range(len(out_loop)):
            k=(j+1)%len(out_loop)
            a=out_loop[j]; b=out_loop[k]
            if li == 0:
                faces.append((back[a], back[b], front[b], front[a]))
            else:
                faces.append((back[b], back[a], front[a], front[b]))
    return verts, faces


def header_flange_mesh_data(header_obj):
    """One generic flange spanning all managed Header primary starts."""
    from .properties import header_primary_objects
    hs = getattr(header_obj, 'exhaust_header', None)
    routes = header_primary_objects(header_obj)
    if not hs or not hs.is_header or not routes:
        return [], []

    starts=[]
    max_profile=24
    for route in routes:
        # Header primaries are intended to share one start plane.  Use the
        # object's local origin for each port center and record any axial error
        # so the UI can warn if a primary was manually moved off the flange.
        loc=Vector(route.location)
        starts.append(loc)
        max_profile=max(max_profile, int(route.exhaust_route.profile_segments))
    plane_x=sum(p.x for p in starts)/len(starts)
    hs.flange_plane_deviation=max(abs(p.x-plane_x) for p in starts)

    bore_d=max(1.0e-6, float(hs.primary_od) + max(0.0, float(hs.flange_bore_clearance)))
    bore_r=0.5*bore_d
    extent_r=max(0.5*float(hs.primary_od), bore_r)
    margin=max(0.0, float(hs.flange_edge_margin))
    min_y=min(p.y for p in starts)-extent_r-margin
    max_y=max(p.y for p in starts)+extent_r+margin
    min_z=min(p.z for p in starts)-extent_r-margin
    max_z=max(p.z for p in starts)+extent_r+margin
    hs.flange_width=max_y-min_y
    hs.flange_height=max_z-min_z

    outer=_rounded_rect_profile_2d(
        min_y,max_y,min_z,max_z,float(hs.flange_corner_radius),int(hs.flange_corner_segments)
    )
    hseg=max(24,max_profile)
    holes=[]
    for p in starts:
        holes.append([
            (p.y+bore_r*math.cos(2.0*math.pi*j/hseg),
             p.z+bore_r*math.sin(2.0*math.pi*j/hseg))
            for j in range(hseg)
        ])
    thickness=max(1.0e-6,float(hs.flange_thickness))
    # Tube-side face is exactly on the primary start plane; the head-side face
    # extends upstream, leaving every primary's normal square-cut start flush.
    return _planar_plate_with_holes_mesh(outer, holes, plane_x, plane_x-thickness)


def rebuild_header_flange_object(header_obj):
    """Create/update the single managed head flange for a Header coordinator."""
    if not header_obj:
        return None
    hs=getattr(header_obj,'exhaust_header',None)
    if not hs or not hs.is_header:
        return None
    flange=None
    for child in getattr(header_obj,'children',()):
        if child.get('exhaust_header_shared_flange',False):
            flange=child; break
    if flange is None:
        mesh=bpy.data.meshes.new(f"{header_obj.name}_Shared_Flange_Mesh")
        flange=bpy.data.objects.new(f"{header_obj.name}_Shared_Flange",mesh)
        collection=header_obj.users_collection[0] if header_obj.users_collection else bpy.context.collection
        collection.objects.link(flange)
        flange.parent=header_obj
        flange.matrix_parent_inverse=Matrix.Identity(4)
        flange.location=(0.0,0.0,0.0)
        flange.rotation_euler=(0.0,0.0,0.0)
        flange['exhaust_header_shared_flange']=True
        flange['exhaust_header_owner']=header_obj.name
        flange.hide_select=True
    verts,faces=header_flange_mesh_data(header_obj)
    mesh=bpy.data.meshes.new(flange.name+"_Mesh")
    mesh.from_pydata(verts,[],faces)
    mesh.update()
    old=flange.data if flange.type=='MESH' else None
    flange.data=mesh
    if old and old.users==0:
        bpy.data.meshes.remove(old)
    for poly in mesh.polygons:
        poly.use_smooth=False
    flange['exhaust_header_flange_width']=float(hs.flange_width)
    flange['exhaust_header_flange_height']=float(hs.flange_height)
    flange['exhaust_header_flange_thickness']=float(hs.flange_thickness)
    return flange


def _append_vband_forward(verts, faces, base_center, tangent, angular_ref, outer_current, inner_current,
                          base_ro, base_ri, spec, sides):
    """Append a rotationally symmetric V-band flange from tube end outward."""
    p = Vector(base_center)
    t = _safe_normal(tangent)
    current_o, current_i = list(outer_current), list(inner_current) if inner_current else None
    x = 0.0
    R = max(base_ro + 1.0e-6, spec['flange_od'] * 0.5)
    # neck
    if spec['neck'] > _EPS:
        x += spec['neck']
        co = p + t*x
        no = _append_route_loop(verts, _route_circle_loop(co, t, angular_ref, base_ro, sides))
        _connect_route_rings(faces, current_o, no, inward=False)
        if current_i is not None:
            ni = _append_route_loop(verts, _route_circle_loop(co, t, angular_ref, base_ri, sides))
            _connect_route_rings(faces, current_i, ni, inward=True); current_i = ni
        current_o = no
    # tapered lip
    x += spec['taper']
    co = p + t*x
    no = _append_route_loop(verts, _route_circle_loop(co, t, angular_ref, R, sides))
    _connect_route_rings(faces, current_o, no, inward=False)
    if current_i is not None:
        ni = _append_route_loop(verts, _route_circle_loop(co, t, angular_ref, base_ri, sides))
        _connect_route_rings(faces, current_i, ni, inward=True); current_i = ni
    current_o = no
    # clamp face land
    x += spec['face']
    co = p + t*x
    no = _append_route_loop(verts, _route_circle_loop(co, t, angular_ref, R, sides))
    _connect_route_rings(faces, current_o, no, inward=False)
    if current_i is not None:
        ni = _append_route_loop(verts, _route_circle_loop(co, t, angular_ref, base_ri, sides))
        _connect_route_rings(faces, current_i, ni, inward=True); current_i = ni
    current_o = no
    if current_i is not None:
        for j in range(sides):
            k=(j+1)%sides
            faces.append((current_o[j], current_o[k], current_i[k], current_i[j]))
    return co, current_o, current_i


def _append_vband_start(verts, faces, base_center, tangent, angular_ref, base_ro, base_ri, spec, sides, hollow):
    """Create a start-side V-band from exposed face forward to the Route/treatment base."""
    p = Vector(base_center)
    t = _safe_normal(tangent)
    R = max(base_ro + 1.0e-6, spec['flange_od'] * 0.5)
    exposed = p - t*spec['extension']
    outer = _append_route_loop(verts, _route_circle_loop(exposed, t, angular_ref, R, sides))
    inner = _append_route_loop(verts, _route_circle_loop(exposed, t, angular_ref, base_ri, sides)) if hollow else None
    if inner is not None:
        for j in range(sides):
            k=(j+1)%sides
            faces.append((outer[k], outer[j], inner[j], inner[k]))
    x = spec['face']
    c = exposed + t*x
    no = _append_route_loop(verts, _route_circle_loop(c, t, angular_ref, R, sides)); _connect_route_rings(faces, outer, no, False)
    if inner is not None:
        ni = _append_route_loop(verts, _route_circle_loop(c, t, angular_ref, base_ri, sides)); _connect_route_rings(faces, inner, ni, True); inner=ni
    outer=no
    x += spec['taper']
    c=exposed+t*x
    no=_append_route_loop(verts,_route_circle_loop(c,t,angular_ref,base_ro,sides)); _connect_route_rings(faces,outer,no,False)
    if inner is not None:
        ni=_append_route_loop(verts,_route_circle_loop(c,t,angular_ref,base_ri,sides)); _connect_route_rings(faces,inner,ni,True); inner=ni
    outer=no
    if spec['neck'] > _EPS:
        c=p
        no=_append_route_loop(verts,_route_circle_loop(c,t,angular_ref,base_ro,sides)); _connect_route_rings(faces,outer,no,False)
        if inner is not None:
            ni=_append_route_loop(verts,_route_circle_loop(c,t,angular_ref,base_ri,sides)); _connect_route_rings(faces,inner,ni,True); inner=ni
        outer=no
    return outer, inner, exposed


def _append_flat_flange_forward(verts, faces, base_center, tangent, angular_ref, outer_current, inner_current,
                                base_ro, base_ri, spec, sides):
    """Append a round-weld or automotive flat flange as one endpoint shell."""
    p0 = Vector(base_center); t = _safe_normal(tangent)
    outer_seg = max(sides, int(spec.get('profile_segments', sides)))
    outer_profile, hole_specs = _hardware_profile_2d(spec, base_ro, outer_seg)
    R_hint = max((math.hypot(y, z) for y, z in outer_profile), default=base_ro)

    # Rear face: the existing route OD is the inner plate boundary.
    rear_outer, _, rear_holes = _append_cap_region_with_holes(
        verts, faces, p0, t, angular_ref, R_hint, base_ro, outer_seg,
        inner_loop_ids=outer_current, hole_specs=hole_specs, reverse=True,
        outer_points_local=outer_profile)

    p1 = p0 + t * spec['thickness']
    front_inner = _append_route_loop(verts, _route_circle_loop(p1, t, angular_ref, base_ri, sides)) if inner_current is not None else None
    front_outer, _, front_holes = _append_cap_region_with_holes(
        verts, faces, p1, t, angular_ref, R_hint, base_ri, outer_seg,
        inner_loop_ids=front_inner, hole_specs=hole_specs, reverse=False,
        outer_points_local=outer_profile)

    _connect_route_rings(faces, rear_outer, front_outer, False)
    for a, b in zip(rear_holes, front_holes):
        _connect_route_rings(faces, a, b, True)
    if inner_current is not None:
        _connect_route_rings(faces, inner_current, front_inner, True)
    return p1, front_outer, front_inner


def _append_flat_flange_start(verts, faces, base_center, tangent, angular_ref, base_ro, base_ri, spec, sides, hollow):
    """Start-side round/flat flange; exposed face lies upstream of tube base."""
    pbase = Vector(base_center); t = _safe_normal(tangent); pfront = pbase - t * spec['thickness']
    outer_seg = max(sides, int(spec.get('profile_segments', sides)))
    outer_profile, hole_specs = _hardware_profile_2d(spec, base_ro, outer_seg)
    R_hint = max((math.hypot(y, z) for y, z in outer_profile), default=base_ro)

    front_inner = _append_route_loop(verts, _route_circle_loop(pfront, t, angular_ref, base_ri, sides)) if hollow else None
    front_outer, _, front_holes = _append_cap_region_with_holes(
        verts, faces, pfront, t, angular_ref, R_hint, base_ri, outer_seg,
        inner_loop_ids=front_inner, hole_specs=hole_specs, reverse=True,
        outer_points_local=outer_profile)

    base_outer = _append_route_loop(verts, _route_circle_loop(pbase, t, angular_ref, base_ro, sides))
    rear_outer, _, rear_holes = _append_cap_region_with_holes(
        verts, faces, pbase, t, angular_ref, R_hint, base_ro, outer_seg,
        inner_loop_ids=base_outer, hole_specs=hole_specs, reverse=False,
        outer_points_local=outer_profile)

    _connect_route_rings(faces, front_outer, rear_outer, False)
    for a, b in zip(front_holes, rear_holes):
        _connect_route_rings(faces, a, b, True)
    base_inner = _append_route_loop(verts, _route_circle_loop(pbase, t, angular_ref, base_ri, sides)) if hollow else None
    if hollow:
        _connect_route_rings(faces, front_inner, base_inner, True)
    return base_outer, base_inner, pfront

def route_mixed_mesh_data(settings):
    """Single shared-vertex route mesh for advanced fabrication features.

    Supports Pie-Cut bends, swaged/reduced/slip end treatments, and endpoint
    connection hardware.  Hardware is built directly from the exposed Route
    rings so the tube bore continues through the fitting without intersecting
    helper cylinders or loose grouped pieces.
    """
    sides = max(12, int(settings.profile_segments))
    ro = max(1.0e-6, settings.outside_diameter * 0.5)
    wall = max(0.0, min(settings.wall_thickness, ro * 0.999))
    ri = max(0.0, ro - wall)
    hollow = ri > 1.0e-6

    verts, faces = [], []
    p = Vector((0.0, 0.0, 0.0))
    tangent = Vector((1.0, 0.0, 0.0))
    up = Vector((0.0, 0.0, 1.0))
    mesh_normal = Vector((0.0, 0.0, 1.0))

    start_spec = route_end_treatment_spec(settings, 'start')
    end_spec = route_end_treatment_spec(settings, 'end')
    start_hw = route_connection_hardware_spec(settings, 'start', start_spec)
    end_hw = route_connection_hardware_spec(settings, 'end', end_spec)
    hwseg = max(sides, int(getattr(settings, 'hardware_profile_segments', 64)))
    start_hw['profile_segments'] = hwseg
    end_hw['profile_segments'] = hwseg
    transition_steps = max(2, int(getattr(settings, 'end_treatment_segments', 8)))

    start_closed = False
    end_closed = False
    outer_start = None
    inner_start = None

    # The exposed end of a start treatment is upstream of the nominal route
    # origin.  Optional hardware attaches there and extends farther upstream.
    target_ro = max(1.0e-6, start_spec['od'] * 0.5)
    target_ri = max(0.0, start_spec['id'] * 0.5)
    start_base = p - tangent * start_spec['extension']

    if start_hw['active']:
        if start_hw['type'] == 'VBAND':
            outer_current, inner_current, _hardware_front = _append_vband_start(
                verts, faces, start_base, tangent, mesh_normal,
                target_ro, target_ri, start_hw, sides, hollow)
        else:
            outer_current, inner_current, _hardware_front = _append_flat_flange_start(
                verts, faces, start_base, tangent, mesh_normal,
                target_ro, target_ri, start_hw, sides, hollow)
        start_closed = True
    else:
        outer_current = _append_route_loop(verts, _route_circle_loop(start_base, tangent, mesh_normal, target_ro, sides))
        inner_current = _append_route_loop(verts, _route_circle_loop(start_base, tangent, mesh_normal, target_ri, sides)) if hollow else None
        outer_start = outer_current[:]
        inner_start = inner_current[:] if hollow else None

    # Start treatment: exposed diameter/collar -> nominal route OD at p.
    if start_spec['active']:
        collar_end = p - tangent * start_spec['transition']
        if start_spec['collar'] > _EPS:
            outer_next = _append_route_loop(verts, _route_circle_loop(collar_end, tangent, mesh_normal, target_ro, sides))
            _connect_route_rings(faces, outer_current, outer_next, inward=False)
            if hollow:
                inner_next = _append_route_loop(verts, _route_circle_loop(collar_end, tangent, mesh_normal, target_ri, sides))
                _connect_route_rings(faces, inner_current, inner_next, inward=True)
                inner_current = inner_next
            outer_current = outer_next

        if start_spec['transition'] > _EPS:
            for i in range(1, transition_steps + 1):
                tt = i / transition_steps
                sm = tt * tt * (3.0 - 2.0 * tt)
                center = collar_end + tangent * (start_spec['transition'] * tt)
                r_out = target_ro + (ro - target_ro) * sm
                r_in = max(0.0, r_out - wall)
                outer_next = _append_route_loop(verts, _route_circle_loop(center, tangent, mesh_normal, r_out, sides))
                _connect_route_rings(faces, outer_current, outer_next, inward=False)
                if hollow:
                    inner_next = _append_route_loop(verts, _route_circle_loop(center, tangent, mesh_normal, r_in, sides))
                    _connect_route_rings(faces, inner_current, inner_next, inward=True)
                    inner_current = inner_next
                outer_current = outer_next
        elif abs(target_ro - ro) > _EPS:
            outer_next = _append_route_loop(verts, _route_circle_loop(p, tangent, mesh_normal, ro, sides))
            _connect_route_rings(faces, outer_current, outer_next, inward=False)
            if hollow:
                inner_next = _append_route_loop(verts, _route_circle_loop(p, tangent, mesh_normal, ri, sides))
                _connect_route_rings(faces, inner_current, inner_next, inward=True)
                inner_current = inner_next
            outer_current = outer_next
    elif start_hw['active']:
        # Hardware base ring is already at nominal route diameter and p.
        pass

    # Main route segments.
    for seg in settings.segments:
        if seg.kind == 'STRAIGHT':
            length = max(0.0, seg.length)
            if length <= _EPS:
                continue
            p = p + tangent * length
            outer_next = _append_route_loop(verts, _route_circle_loop(p, tangent, mesh_normal, ro, sides))
            _connect_route_rings(faces, outer_current, outer_next, inward=False)
            if hollow:
                inner_next = _append_route_loop(verts, _route_circle_loop(p, tangent, mesh_normal, ri, sides))
                _connect_route_rings(faces, inner_current, inner_next, inward=True)
                inner_current = inner_next
            outer_current = outer_next
            continue

        angle = abs(float(seg.angle))
        if angle < _EPS:
            continue

        if getattr(seg, 'bend_style', 'MANDREL') == 'PIE_CUT':
            centers, axes, planes, axis, tangent_end, up_end, _signed_delta, _section_length = _route_pie_layout(p, tangent, up, seg)
            sections = len(axes)
            mesh_start = _safe_normal(mesh_normal - tangent * mesh_normal.dot(tangent), mesh_normal)
            for boundary in range(1, len(centers)):
                if boundary < len(centers) - 1:
                    cyl_axis = axes[boundary - 1]
                    ref = _rotation(axis, angle * (boundary - 1) / max(1, sections - 1)) @ mesh_start
                else:
                    cyl_axis = axes[-1]
                    ref = _rotation(axis, angle) @ mesh_start
                outer_loop = _route_miter_loop(centers[boundary], cyl_axis, planes[boundary], ref, ro, sides)
                outer_next = _append_route_loop(verts, outer_loop)
                _connect_route_rings(faces, outer_current, outer_next, inward=False)
                if hollow:
                    inner_loop = _route_miter_loop(centers[boundary], cyl_axis, planes[boundary], ref, ri, sides)
                    inner_next = _append_route_loop(verts, inner_loop)
                    _connect_route_rings(faces, inner_current, inner_next, inward=True)
                    inner_current = inner_next
                outer_current = outer_next

            p = centers[-1].copy()
            mesh_normal = _safe_normal(_rotation(axis, angle) @ mesh_start)
            mesh_normal = _safe_normal(mesh_normal - tangent_end * mesh_normal.dot(tangent_end), mesh_normal)
            tangent = tangent_end
            up = up_end
            continue

        radius = max(seg.radius, 1.0e-6)
        bend_dir, signed_dir, axis = _segment_bend_basis(tangent, up, seg)
        center = p + signed_dir * radius
        radial0 = p - center
        steps = max(2, int(seg.resolution))
        tangent_start = tangent.copy()
        mesh_start = _safe_normal(mesh_normal - tangent * mesh_normal.dot(tangent), mesh_normal)

        for i in range(1, steps + 1):
            a = angle * (i / steps)
            rot = _rotation(axis, a)
            q = center + (rot @ radial0)
            ti = _safe_normal(rot @ tangent_start, tangent_start)
            ni = _safe_normal(rot @ mesh_start, mesh_start)
            outer_next = _append_route_loop(verts, _route_circle_loop(q, ti, ni, ro, sides))
            _connect_route_rings(faces, outer_current, outer_next, inward=False)
            if hollow:
                inner_next = _append_route_loop(verts, _route_circle_loop(q, ti, ni, ri, sides))
                _connect_route_rings(faces, inner_current, inner_next, inward=True)
                inner_current = inner_next
            outer_current = outer_next

        rot_end = _rotation(axis, angle)
        p = center + (rot_end @ radial0)
        tangent = _safe_normal(rot_end @ tangent_start)
        mesh_normal = _safe_normal(rot_end @ mesh_start)
        mesh_normal = _safe_normal(mesh_normal - tangent * mesh_normal.dot(tangent), mesh_normal)
        up = _safe_normal(rot_end @ bend_dir)
        up = _safe_normal(up - tangent * up.dot(tangent), Vector((0.0, 0.0, 1.0)))

    # Finish treatment grows forward from the nominal Route end.
    target_ro = max(1.0e-6, end_spec['od'] * 0.5)
    target_ri = max(0.0, end_spec['id'] * 0.5)
    if end_spec['active']:
        transition_start = p.copy()
        if end_spec['transition'] > _EPS:
            for i in range(1, transition_steps + 1):
                tt = i / transition_steps
                sm = tt * tt * (3.0 - 2.0 * tt)
                center = transition_start + tangent * (end_spec['transition'] * tt)
                r_out = ro + (target_ro - ro) * sm
                r_in = max(0.0, r_out - wall)
                outer_next = _append_route_loop(verts, _route_circle_loop(center, tangent, mesh_normal, r_out, sides))
                _connect_route_rings(faces, outer_current, outer_next, inward=False)
                if hollow:
                    inner_next = _append_route_loop(verts, _route_circle_loop(center, tangent, mesh_normal, r_in, sides))
                    _connect_route_rings(faces, inner_current, inner_next, inward=True)
                    inner_current = inner_next
                outer_current = outer_next
        elif abs(target_ro - ro) > _EPS:
            outer_next = _append_route_loop(verts, _route_circle_loop(p, tangent, mesh_normal, target_ro, sides))
            _connect_route_rings(faces, outer_current, outer_next, inward=False)
            if hollow:
                inner_next = _append_route_loop(verts, _route_circle_loop(p, tangent, mesh_normal, target_ri, sides))
                _connect_route_rings(faces, inner_current, inner_next, inward=True)
                inner_current = inner_next
            outer_current = outer_next

        if end_spec['collar'] > _EPS:
            collar_end = p + tangent * end_spec['extension']
            outer_next = _append_route_loop(verts, _route_circle_loop(collar_end, tangent, mesh_normal, target_ro, sides))
            _connect_route_rings(faces, outer_current, outer_next, inward=False)
            if hollow:
                inner_next = _append_route_loop(verts, _route_circle_loop(collar_end, tangent, mesh_normal, target_ri, sides))
                _connect_route_rings(faces, inner_current, inner_next, inward=True)
                inner_current = inner_next
            outer_current = outer_next
        hardware_base = p + tangent * end_spec['extension']
    else:
        hardware_base = p.copy()

    if end_hw['active']:
        if end_hw['type'] == 'VBAND':
            _front, outer_current, inner_current = _append_vband_forward(
                verts, faces, hardware_base, tangent, mesh_normal,
                outer_current, inner_current, target_ro, target_ri, end_hw, sides)
        else:
            _front, outer_current, inner_current = _append_flat_flange_forward(
                verts, faces, hardware_base, tangent, mesh_normal,
                outer_current, inner_current, target_ro, target_ri, end_hw, sides)
        end_closed = True

    if hollow:
        if not start_closed and outer_start is not None and inner_start is not None:
            for j in range(sides):
                k = (j + 1) % sides
                faces.append((outer_start[j], inner_start[j], inner_start[k], outer_start[k]))
        if not end_closed:
            for j in range(sides):
                k = (j + 1) % sides
                faces.append((outer_current[j], outer_current[k], inner_current[k], inner_current[j]))

    return verts, faces

def route_pie_cut_seam_loops(settings, guide_offset=0.0):
    """Viewport-only internal miter loops for Pie-Cut bends embedded in routes."""
    loops = []
    p = Vector((0.0, 0.0, 0.0))
    tangent = Vector((1.0, 0.0, 0.0))
    up = Vector((0.0, 0.0, 1.0))
    sides = max(12, int(settings.profile_segments))
    ro = max(1.0e-6, settings.outside_diameter * 0.5 + max(0.0, guide_offset))

    for seg in settings.segments:
        if seg.kind == 'STRAIGHT':
            p = p + tangent * max(0.0, seg.length)
            continue

        angle = abs(float(seg.angle))
        if angle < _EPS:
            continue

        if getattr(seg, 'bend_style', 'MANDREL') == 'PIE_CUT':
            centers, axes, planes, axis, tangent_end, up_end, _signed_delta, _section_length = _route_pie_layout(p, tangent, up, seg)
            if getattr(seg, 'show_pie_weld_seams', True):
                # Vertex phase is irrelevant for a guide loop; use a stable
                # transported reference around each straight cylinder section.
                ref0 = up.copy()
                for boundary in range(1, len(centers) - 1):
                    ref = _rotation(axis, angle * (boundary - 1) / max(1, len(axes) - 1)) @ ref0
                    loops.append(_route_miter_loop(centers[boundary], axes[boundary - 1], planes[boundary], ref, ro, sides))
            p = centers[-1].copy()
            tangent, up = tangent_end, up_end
            continue

        radius = max(seg.radius, 1.0e-6)
        bend_dir, signed_dir, axis = _segment_bend_basis(tangent, up, seg)
        center = p + signed_dir * radius
        radial0 = p - center
        rot_end = _rotation(axis, angle)
        p = center + (rot_end @ radial0)
        tangent = _safe_normal(rot_end @ tangent)
        up = _safe_normal(rot_end @ bend_dir)
        up = _safe_normal(up - tangent * up.dot(tangent), Vector((0.0, 0.0, 1.0)))

    return loops


def _replace_mesh(obj, verts, faces):
    mesh = bpy.data.meshes.new(obj.name + "_Mesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    old = obj.data if obj.type == 'MESH' else None
    obj.data = mesh
    if old and old.users == 0:
        bpy.data.meshes.remove(old)
    for poly in mesh.polygons:
        poly.use_smooth = True


def _set_connector_metadata(obj, start_point, start_tangent, end_point, end_tangent):
    obj["exhaust_connector_start"] = tuple(start_point)
    obj["exhaust_connector_start_tangent"] = tuple(start_tangent)
    obj["exhaust_connector_end"] = tuple(end_point)
    obj["exhaust_connector_end_tangent"] = tuple(end_tangent)


def rebuild_route_object(obj):
    if not obj or obj.name in _rebuild_guard or not getattr(obj, "exhaust_route", None):
        return
    settings = obj.exhaust_route
    if not settings.is_route:
        return
    _rebuild_guard.add(obj.name)
    try:
        pts = route_points(settings)
        has_pie = any(seg.kind == 'BEND' and getattr(seg, 'bend_style', 'MANDREL') == 'PIE_CUT' for seg in settings.segments)
        start_spec = route_end_treatment_spec(settings, 'start')
        end_spec = route_end_treatment_spec(settings, 'end')
        start_hw = route_connection_hardware_spec(settings, 'start', start_spec)
        end_hw = route_connection_hardware_spec(settings, 'end', end_spec)
        has_end_treatments = start_spec['active'] or end_spec['active']
        has_hardware = start_hw['active'] or end_hw['active']
        if has_pie or has_end_treatments or has_hardware:
            verts, faces = route_mixed_mesh_data(settings)
            obj["exhaust_route_contains_pie_cut"] = bool(has_pie)
            obj["exhaust_route_has_end_treatments"] = bool(has_end_treatments)
            obj["exhaust_route_has_connection_hardware"] = bool(has_hardware)
        else:
            # Preserve the validated legacy route sweep exactly for ordinary routes.
            verts, faces = tube_mesh_data(pts, settings.outside_diameter, settings.wall_thickness, settings.profile_segments)
            obj["exhaust_route_contains_pie_cut"] = False
            obj["exhaust_route_has_end_treatments"] = False
            obj["exhaust_route_has_connection_hardware"] = False
        _replace_mesh(obj, verts, faces)
        t0 = _safe_normal(pts[1] - pts[0])
        t1 = _safe_normal(pts[-1] - pts[-2])
        start_point = pts[0] - t0 * (start_spec['extension'] + start_hw['extension'])
        end_point = pts[-1] + t1 * (end_spec['extension'] + end_hw['extension'])
        settings.centerline_length = (route_centerline_length(settings.segments) + start_spec['extension'] + end_spec['extension']
                                      + start_hw['extension'] + end_hw['extension'])
        _set_connector_metadata(obj, start_point, t0, end_point, t1)
        obj["exhaust_connector_start_od"] = float(start_hw['connector_od'] if start_hw['active'] else start_spec['od'])
        obj["exhaust_connector_start_id"] = float(start_hw['connector_id'] if start_hw['active'] else start_spec['id'])
        obj["exhaust_connector_end_od"] = float(end_hw['connector_od'] if end_hw['active'] else end_spec['od'])
        obj["exhaust_connector_end_id"] = float(end_hw['connector_id'] if end_hw['active'] else end_spec['id'])
        obj["exhaust_connector_start_type"] = str(start_hw['type'] if start_hw['active'] else start_spec['type'])
        obj["exhaust_connector_end_type"] = str(end_hw['type'] if end_hw['active'] else end_spec['type'])
        # Header equal-length stats are derived from managed child Routes.
        try:
            header = obj.parent
            hs = getattr(header, "exhaust_header", None) if header else None
            if hs and hs.is_header:
                from .properties import refresh_header_stats
                rebuild_header_flange_object(header)
                refresh_header_stats(header)
        except Exception:
            pass
    finally:
        _rebuild_guard.discard(obj.name)


def _append_mesh_piece(all_verts, all_faces, verts, faces):
    offset = len(all_verts)
    all_verts.extend(verts)
    all_faces.extend(tuple(offset + i for i in face) for face in faces)


def _smoothstep(t):
    return t * t * (3.0 - 2.0 * t)


def collector_branch_points(angle, radius_start, radius_end, length, samples):
    """Smooth radial-symmetric branch centerline between two radial offsets."""
    radial = Vector((0.0, math.cos(angle), math.sin(angle)))
    points = []
    samples = max(4, int(samples))
    for i in range(samples + 1):
        t = i / samples
        sm = _smoothstep(t)
        rr = radius_start + (radius_end - radius_start) * sm
        x = length * t
        points.append(Vector((x, radial.y * rr, radial.z * rr)))
    return points


def collector_radius(settings):
    if settings.auto_radial_spread:
        return packed_radial_radius(settings.primary_count, settings.primary_od, settings.radial_gap)
    return max(0.0, settings.radial_spread)


def _circle_envelope_radius(theta, n, phase, center_radius, circle_radius):
    """Outer radial envelope of N equal circles arranged radially about the X axis.

    The merge loft begins only after the circle centers are inside the circle radius,
    making the union star-shaped around the collector axis.  This lets a single polar
    loop exactly trace the rounded N-lobe outer envelope and preserve radial symmetry.
    """
    if circle_radius <= _EPS:
        return 0.0
    if center_radius <= _EPS:
        return circle_radius
    best = 0.0
    for i in range(n):
        phi = phase + 2.0 * math.pi * i / n
        d = theta - phi
        sd = math.sin(d)
        disc = circle_radius * circle_radius - center_radius * center_radius * sd * sd
        if disc < 0.0:
            continue
        r = center_radius * math.cos(d) + math.sqrt(max(0.0, disc))
        if r > best:
            best = r
    return max(best, circle_radius * 0.05)


def _circle_gap_radius(theta, n, phase, center_radius, circle_radius):
    """Radius of the central gap bounded by N equal offset circles.

    This is the FIRST positive intersection of a ray from the collector axis
    with any primary OUTER circle.  At exact primary first-tangency it traces
    the curvilinear polygonal hole between the tubes.  It is therefore the
    correct non-overlapping boundary for an automatic merge core.
    """
    if circle_radius <= _EPS or center_radius <= circle_radius + _EPS:
        return 0.0
    best = None
    for i in range(n):
        phi = phase + 2.0 * math.pi * i / n
        d = theta - phi
        sd = math.sin(d)
        disc = circle_radius * circle_radius - center_radius * center_radius * sd * sd
        if disc < -1.0e-12:
            continue
        disc = max(0.0, disc)
        r = center_radius * math.cos(d) - math.sqrt(disc)
        if r >= -1.0e-9:
            r = max(0.0, r)
            if best is None or r < best:
                best = r
    return 0.0 if best is None else best


def _inverse_smoothstep(y):
    """Numerically invert smoothstep on [0, 1]."""
    y = max(0.0, min(1.0, y))
    lo, hi = 0.0, 1.0
    for _ in range(32):
        mid = 0.5 * (lo + hi)
        if _smoothstep(mid) < y:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def collector_merge_core_mesh_data(settings, body_start_x, tangent_center_radius, total_length, sides):
    """Create the automatic central merge core without intersecting primaries.

    The widest ring is EXACTLY the empty space between the primary OUTER
    circles at their first-tangency plane.  The core therefore fills the hole
    seen from the inlet without clipping through any primary tube.  It receives
    a short upstream nose inside the still-larger branch gap and tapers
    downstream until the outer-circle central gap mathematically closes.

    This is deliberately generated from the same radial symmetry math as the
    collector, rather than as an arbitrary pyramid/star insert.
    """
    n = max(2, min(12, int(settings.primary_count)))
    if n <= 2:
        # Two tangent circles have no finite central interstitial hole.
        return [], []

    phase = settings.radial_phase
    ro0 = max(1.0e-6, settings.primary_od * 0.5)
    ro1 = max(1.0e-6, settings.outlet_od * 0.5)
    transition_len = max(1.0e-6, total_length - body_start_x)

    # Solve for the normalized transition position where the collapsing circle
    # centers reach the local outer radius.  At this point the central gap is
    # exactly zero for every angular direction, so the core can terminate at
    # one clean downstream point with no overlap.
    denom = tangent_center_radius + (ro1 - ro0)
    if denom <= _EPS:
        sm_tip = 0.0
    else:
        sm_tip = (tangent_center_radius - ro0) / denom
    sm_tip = max(0.0, min(0.999999, sm_tip))
    t_tip = _inverse_smoothstep(sm_tip)
    if t_tip <= 1.0e-5:
        return [], []

    # Keep the upstream point well inside the available branch length.
    nose_len = min(max(0.0, body_start_x) * 0.35, settings.primary_od * 0.75)
    nose_x = body_start_x - nose_len
    downstream_tip_x = body_start_x + transition_len * t_tip

    ring_steps = max(3, int(round(max(4, settings.transition_segments) * t_tip)))
    rings = []
    for k in range(ring_steps):
        # Do not create the mathematically zero final ring; it would generate
        # degenerate quads.  A single point closes the core downstream.
        t = t_tip * (k / ring_steps)
        sm = _smoothstep(t)
        x = body_start_x + transition_len * t
        local_ro = ro0 + (ro1 - ro0) * sm
        center_r = tangent_center_radius * (1.0 - sm)
        ring = []
        for j in range(sides):
            theta = 2.0 * math.pi * j / sides
            r = _circle_gap_radius(theta, n, phase, center_r, local_ro)
            ring.append((x, math.cos(theta) * r, math.sin(theta) * r))
        rings.append(ring)

    if not rings:
        return [], []

    verts = []
    faces = []
    upstream_tip = len(verts)
    verts.append((nose_x, 0.0, 0.0))

    bases = []
    for ring in rings:
        bases.append(len(verts))
        verts.extend(ring)

    downstream_tip = len(verts)
    verts.append((downstream_tip_x, 0.0, 0.0))

    # Upstream pointed nose -> exact central-gap ring at first tangency.
    first = bases[0]
    for j in range(sides):
        q = (j + 1) % sides
        faces.append((upstream_tip, first + q, first + j))

    # Radially symmetric core body.
    for r in range(len(bases) - 1):
        a = bases[r]
        b = bases[r + 1]
        for j in range(sides):
            q = (j + 1) % sides
            faces.append((a + j, a + q, b + q, b + j))

    # Final ring -> exact downstream closure point.
    last = bases[-1]
    for j in range(sides):
        q = (j + 1) % sides
        faces.append((last + j, last + q, downstream_tip))

    return verts, faces


def _radial_shell_radius_from_outer(outer_radius, wall):
    """Approximate constant wall thickness for the common collector shell.

    The common collector is star-shaped about the X axis.  Offsetting its inner
    boundary radially is stable for live editing and, critically, does not require
    overlapping copies of the individual primary tubes inside the merge region.
    """
    if wall <= _EPS:
        return 0.0
    return max(1.0e-6, outer_radius - wall)


def collector_loft_mesh_data(settings, body_start_x, tangent_center_radius, total_length):
    """Create the common collector body after the primaries reach first tangency.

    v0.3.3 tangent-shell topology:
      * Primary OUTER surfaces stop exactly when neighboring primaries first touch.
      * No primary cylinder is allowed to penetrate or cross another primary.
      * From that station onward a single N-fold radial envelope forms the OUTER
        collector surface and morphs continuously to the round outlet.
      * The common inner surface is derived from that same envelope minus wall
        thickness, so the collector remains a hollow shell without adding another
        set of intersecting primary meshes inside the body.

    The central interstitial hole is filled separately by a mathematically bounded
    merge core that never penetrates the tangent-trimmed primary tubes.
    """
    n = max(2, min(12, int(settings.primary_count)))
    # Profile Segments is treated as PER-PRIMARY resolution for the common
    # body.  A 4:1 with 48-sided primaries therefore receives 192 samples
    # around the shared multi-lobe transition instead of only 48.
    per_primary_sides = max(12, int(settings.profile_segments))
    sides = n * per_primary_sides
    # Never make the common body axially coarser than the branch centerlines.
    axial_steps = max(4, int(settings.transition_segments), int(settings.branch_samples))
    phase = settings.radial_phase
    transition_len = max(1.0e-6, total_length - body_start_x)

    ro0 = max(1.0e-6, settings.primary_od * 0.5)
    ro1 = max(1.0e-6, settings.outlet_od * 0.5)
    wall = max(0.0, min(settings.wall_thickness, min(ro0, ro1) * 0.999))
    hollow = wall > 1.0e-6

    outer_rings = []
    inner_rings = []
    for k in range(axial_steps + 1):
        t = k / axial_steps
        sm = _smoothstep(t)
        x = body_start_x + transition_len * t

        # The centers begin at exact first-contact spacing and then collapse
        # smoothly to the collector axis.  Because the common shell owns this
        # whole region, there are no N penetrating primary cylinders here.
        local_ro = ro0 + (ro1 - ro0) * sm
        # Keep the N-circle envelope star-shaped for every slice, including
        # unusual collectors whose outlet is smaller than a primary.
        star_limit = local_ro / max(math.sin(math.pi / n), 1.0e-6)
        center_r = min(tangent_center_radius * (1.0 - sm), star_limit)

        outer = []
        inner = []
        for j in range(sides):
            theta = 2.0 * math.pi * j / sides
            r = _circle_envelope_radius(theta, n, phase, center_r, local_ro)
            outer.append((x, math.cos(theta) * r, math.sin(theta) * r))
            if hollow:
                ir = _radial_shell_radius_from_outer(r, wall)
                inner.append((x, math.cos(theta) * ir, math.sin(theta) * ir))
        outer_rings.append(outer)
        if hollow:
            inner_rings.append(inner)

    # Straight round outlet remains part of the same common shell.
    outlet_len = max(0.0, settings.outlet_length)
    if outlet_len > _EPS:
        x = total_length + outlet_len
        outer_rings.append([
            (x, math.cos(2.0 * math.pi * j / sides) * ro1,
             math.sin(2.0 * math.pi * j / sides) * ro1)
            for j in range(sides)
        ])
        if hollow:
            ri1 = max(1.0e-6, ro1 - wall)
            inner_rings.append([
                (x, math.cos(2.0 * math.pi * j / sides) * ri1,
                 math.sin(2.0 * math.pi * j / sides) * ri1)
                for j in range(sides)
            ])

    verts = []
    faces = []
    outer_base = []
    for ring in outer_rings:
        outer_base.append(len(verts))
        verts.extend(ring)

    inner_base = []
    if hollow:
        for ring in inner_rings:
            inner_base.append(len(verts))
            verts.extend(ring)

    for r in range(len(outer_rings) - 1):
        a = outer_base[r]
        b = outer_base[r + 1]
        for j in range(sides):
            q = (j + 1) % sides
            faces.append((a + j, a + q, b + q, b + j))
            if hollow:
                ia = inner_base[r]
                ib = inner_base[r + 1]
                faces.append((ia + j, ib + j, ib + q, ia + q))

    if hollow:
        # Only the downstream cut is capped.  The upstream body boundary opens
        # directly into the tangent-trimmed primary tubes.
        ob = outer_base[-1]
        ib = inner_base[-1]
        for j in range(sides):
            q = (j + 1) % sides
            faces.append((ob + j, ob + q, ib + q, ib + j))
    return verts, faces



def _collector_aligned_profile_sides(requested, n):
    """Profile count with exact vertices at every adjacent-primary tangent point.

    Tangency occurs at local circle angle pi/2 + pi/N.  A multiple of 4*N
    guarantees that angle is represented exactly on every primary ring, which
    lets branch, outer collector, and merge-spike boundaries share vertices.
    """
    requested = max(12, int(requested))
    multiple = max(8, 4 * max(2, int(n)))
    return ((requested + multiple - 1) // multiple) * multiple


def _collector_arc_indices(sides, n):
    """Return outer-perimeter and central-gap local ring index sequences.

    Each sequence includes its START tangent vertex and excludes its END
    tangent vertex.  When the N primary sequences are concatenated, the next
    primary supplies the coincident end tangent.  A final weld therefore makes
    one continuous loop without duplicate seam vertices.
    """
    if n <= 2:
        # For a 2:1, the two tangent circles have no finite enclosed central
        # gap.  The exterior union uses essentially the full circumference.
        return list(range(sides)), []

    j_plus = sides // 4 + sides // (2 * n)
    j_minus = (-j_plus) % sides

    outer = []
    j = j_minus
    while j != j_plus:
        outer.append(j)
        j = (j + 1) % sides

    # Reverse the inward arc so the resulting central-gap loop walks in the
    # same global angular direction as the outer perimeter.
    gap = []
    j = j_minus
    while j != j_plus:
        gap.append(j)
        j = (j - 1) % sides
    return outer, gap


def _collector_transition_center_radius(t, tangent_radius, strength):
    """Radial center collapse used by both the outer and inner merge topology."""
    sm = _smoothstep(max(0.0, min(1.0, t)))
    # High lobe strength keeps the individual primary lobes farther downstream;
    # low values converge more quickly.  The range deliberately avoids extreme
    # exponents that can pinch the mesh numerically.
    strength = max(0.05, min(0.95, float(strength)))
    exponent = 1.50 - strength  # 1.45 .. 0.55
    return tangent_radius * max(0.0, 1.0 - sm) ** exponent


def _collector_local_radius(t, start_radius, end_radius):
    sm = _smoothstep(max(0.0, min(1.0, t)))
    return start_radius + (end_radius - start_radius) * sm


def _solve_transition_t(fn, lo=0.0, hi=1.0):
    """Solve a monotonic sign-changing transition equation on [0, 1]."""
    flo = fn(lo)
    fhi = fn(hi)
    if flo <= 0.0:
        return lo
    if fhi >= 0.0:
        return hi
    for _ in range(48):
        mid = 0.5 * (lo + hi)
        if fn(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _collector_branch_frame(phi, t, radius_start, radius_end, length):
    """Analytic radial-plane frame for one collector branch sample."""
    radial = Vector((0.0, math.cos(phi), math.sin(phi)))
    sm = _smoothstep(t)
    rr = radius_start + (radius_end - radius_start) * sm
    center = Vector((length * t, radial.y * rr, radial.z * rr))

    # d smoothstep / dt = 6t(1-t); divide radial derivative by dx/dt=length.
    if length > _EPS:
        drr_dx = (radius_end - radius_start) * (6.0 * t * (1.0 - t)) / length
    else:
        drr_dx = 0.0
    tangent = _safe_normal(Vector((1.0, radial.y * drr_dx, radial.z * drr_dx)), Vector((1.0, 0.0, 0.0)))
    outward = _safe_normal(radial - tangent * radial.dot(tangent), radial)
    circum = _safe_normal(tangent.cross(outward), Vector((0.0, 0.0, 1.0)))
    outward = _safe_normal(circum.cross(tangent), outward)
    return center, tangent, outward, circum


def _append_ring(verts, center, u, v, radius, sides):
    refs = []
    for j in range(sides):
        a = 2.0 * math.pi * j / sides
        refs.append(len(verts))
        verts.append(tuple(center + u * (math.cos(a) * radius) + v * (math.sin(a) * radius)))
    return refs


def _append_loop_ring_from_polar(verts, x, thetas, radii):
    refs = []
    for theta, radius in zip(thetas, radii):
        refs.append(len(verts))
        verts.append((x, math.cos(theta) * radius, math.sin(theta) * radius))
    return refs


def _connect_loops(faces, a, b, inward=False):
    if len(a) != len(b) or len(a) < 2:
        return
    m = len(a)
    for j in range(m):
        q = (j + 1) % m
        if inward:
            faces.append((a[j], b[j], b[q], a[q]))
        else:
            faces.append((a[j], a[q], b[q], b[j]))


def _loop_thetas(verts, refs):
    out = []
    for idx in refs:
        _, y, z = verts[idx]
        out.append(math.atan2(z, y))
    return out


def _weld_mesh_data(verts, faces, tolerance=1.0e-7, merge_min_x=None):
    """Merge coincident seam vertices and remove faces collapsed by the weld.

    This is intentionally only a seam weld, not a remesh.  It preserves the
    parametric rings and merely makes tangent primary vertices truly identical
    vertices in the final collector mesh.
    """
    if not verts:
        return verts, faces
    inv = 1.0 / max(tolerance, 1.0e-12)
    buckets = {}
    remap = [0] * len(verts)
    new_verts = []

    for i, co in enumerate(verts):
        if merge_min_x is not None and co[0] < merge_min_x - tolerance:
            # Do not weld long parallel/tangent primary runs upstream.  Only
            # the actual collector junction and downstream topology are seam-
            # welded into shared vertices.
            key = ("pre", i)
        else:
            key = (round(co[0] * inv), round(co[1] * inv), round(co[2] * inv))
        idx = buckets.get(key)
        if idx is None:
            idx = len(new_verts)
            buckets[key] = idx
            new_verts.append(co)
        remap[i] = idx

    new_faces = []
    seen = set()
    for face in faces:
        mapped = [remap[i] for i in face]
        compact = []
        for i in mapped:
            if not compact or compact[-1] != i:
                compact.append(i)
        if len(compact) > 1 and compact[0] == compact[-1]:
            compact.pop()
        if len(set(compact)) < 3:
            continue
        key = tuple(sorted(compact))
        if key in seen:
            continue
        seen.add(key)
        new_faces.append(tuple(compact))
    return new_verts, new_faces


def collector_unified_mesh_data(settings, inlet_radius, tangent_center_radius, body_start_x, total_length):
    """Generate the full N->1 collector as ONE topologically connected shell.

    Topology model:
      1) N hollow primaries approach until their OUTER circles first touch.
      2) At that exact ring each primary outer circumference is split into an
         external perimeter arc and a central-gap arc.  Both downstream surfaces
         reuse those SAME vertices.  The central-gap surface is the concave
         OUTSIDE of the merge spike; it is not a separate insert.
      3) The N INNER primary bores continue downstream independently until their
         inner circles first touch.  At that ring the same split creates the main
         common inner bore plus the inner face of the merge spike.
      4) Outer and inner spike faces collapse at their mathematically correct
         closure stations.  Their axial separation is the physical merge-spike
         metal thickness/length generated by the tube wall thickness.
      5) The common outer/inner rings then continue to one round outlet.

    No overlapping primary cylinders, no grouped sub-mesh collector body, and no
    separately inserted spike geometry are used.
    """
    n = max(2, min(12, int(settings.primary_count)))
    phase = float(settings.radial_phase)
    ro0 = max(1.0e-6, settings.primary_od * 0.5)
    ro1 = max(1.0e-6, settings.outlet_od * 0.5)
    wall = max(0.0, min(settings.wall_thickness, min(ro0, ro1) * 0.999))
    hollow = wall > 1.0e-6
    ri0 = max(1.0e-6, ro0 - wall)
    ri1 = max(1.0e-6, ro1 - wall)

    sides = _collector_aligned_profile_sides(settings.profile_segments, n)
    outer_arc, gap_arc = _collector_arc_indices(sides, n)
    branch_steps = max(4, int(settings.branch_samples))
    axial_steps = max(6, int(settings.transition_segments), int(settings.branch_samples))
    transition_len = max(1.0e-6, total_length - body_start_x)

    verts = []
    faces = []
    branch_end_outer = []
    branch_end_inner = []

    # --- N hollow primary branches -------------------------------------------------
    for i in range(n):
        phi = phase + 2.0 * math.pi * i / n
        outer_rings = []
        inner_rings = []
        for k in range(branch_steps + 1):
            t = k / branch_steps
            center, tangent, u, v = _collector_branch_frame(phi, t, inlet_radius, tangent_center_radius, body_start_x)
            outer_rings.append(_append_ring(verts, center, u, v, ro0, sides))
            if hollow:
                inner_rings.append(_append_ring(verts, center, u, v, ri0, sides))

        for r in range(branch_steps):
            _connect_loops(faces, outer_rings[r], outer_rings[r + 1], inward=False)
            if hollow:
                _connect_loops(faces, inner_rings[r], inner_rings[r + 1], inward=True)

        if hollow:
            # Annular inlet cut face only.  The downstream rings feed directly
            # into the common topology below and are never capped.
            o = outer_rings[0]
            inn = inner_rings[0]
            for j in range(sides):
                q = (j + 1) % sides
                faces.append((o[j], inn[j], inn[q], o[q]))

        branch_end_outer.append(outer_rings[-1])
        if hollow:
            branch_end_inner.append(inner_rings[-1])

    # Split every tangent OUTER primary ring into the common external perimeter
    # and the central concave merge-spike boundary.  The split uses existing
    # primary vertices: there is no duplicate collector-body seam ring.
    outer_start = []
    outer_gap_start = []
    for i in range(n):
        ring = branch_end_outer[i]
        outer_start.extend(ring[j] for j in outer_arc)
        if gap_arc:
            outer_gap_start.extend(ring[j] for j in gap_arc)

    outer_theta = _loop_thetas(verts, outer_start)
    outer_gap_theta = _loop_thetas(verts, outer_gap_start) if outer_gap_start else []

    def center_at(t):
        return _collector_transition_center_radius(t, tangent_center_radius, settings.merge_lobe_strength)

    def ro_at(t):
        return _collector_local_radius(t, ro0, ro1)

    def ri_at(t):
        return _collector_local_radius(t, ri0, ri1)

    # --- Common OUTER perimeter ----------------------------------------------------
    prev = outer_start
    outer_final = None
    for k in range(1, axial_steps + 1):
        t = k / axial_steps
        x = body_start_x + transition_len * t
        c = center_at(t)
        rr = ro_at(t)
        radii = [_circle_envelope_radius(th, n, phase, c, rr) for th in outer_theta]
        cur = _append_loop_ring_from_polar(verts, x, outer_theta, radii)
        _connect_loops(faces, prev, cur, inward=False)
        prev = cur
    outer_final = prev

    # --- Concave OUTER face of merge spike ----------------------------------------
    if outer_gap_start:
        t_outer_close = _solve_transition_t(lambda t: center_at(t) - ro_at(t))
        gap_steps = max(2, int(math.ceil(axial_steps * max(t_outer_close, 0.05))))
        prev = outer_gap_start
        for k in range(1, gap_steps):
            t = t_outer_close * (k / gap_steps)
            x = body_start_x + transition_len * t
            c = center_at(t)
            rr = ro_at(t)
            radii = [_circle_gap_radius(th, n, phase, c, rr) for th in outer_gap_theta]
            cur = _append_loop_ring_from_polar(verts, x, outer_gap_theta, radii)
            # This is an exterior surface facing the central valley, so its
            # winding is the inverse of the common outer perimeter.
            _connect_loops(faces, prev, cur, inward=True)
            prev = cur
        tip = len(verts)
        tip_x = body_start_x + transition_len * t_outer_close
        verts.append((tip_x, 0.0, 0.0))
        m = len(prev)
        for j in range(m):
            q = (j + 1) % m
            faces.append((prev[j], tip, prev[q]))

    inner_final = None
    if hollow:
        # --- N separate inner bores until INNER first tangency ---------------------
        sin_sector = max(math.sin(math.pi / n), 1.0e-6)
        t_inner_pair = _solve_transition_t(lambda t: center_at(t) * sin_sector - ri_at(t))
        bore_steps = max(1, int(math.ceil(axial_steps * max(t_inner_pair, 0.02))))
        inner_tangent_rings = []

        for i in range(n):
            phi = phase + 2.0 * math.pi * i / n
            prev = branch_end_inner[i]
            for k in range(1, bore_steps + 1):
                t = t_inner_pair * (k / bore_steps)
                c = center_at(t)
                rr = ri_at(t)
                x = body_start_x + transition_len * t
                center = Vector((x, math.cos(phi) * c, math.sin(phi) * c))
                u = Vector((0.0, math.cos(phi), math.sin(phi)))
                v = Vector((0.0, -math.sin(phi), math.cos(phi)))
                cur = _append_ring(verts, center, u, v, rr, sides)
                _connect_loops(faces, prev, cur, inward=True)
                prev = cur
            inner_tangent_rings.append(prev)

        # At inner first-tangency, each complete inner ring splits using the same
        # exact tangent indices.  Outer arcs become the common flow bore; inward
        # arcs become the INNER face of the finite-thickness merge spike.
        inner_main_start = []
        inner_gap_start = []
        for i in range(n):
            ring = inner_tangent_rings[i]
            inner_main_start.extend(ring[j] for j in outer_arc)
            if gap_arc:
                inner_gap_start.extend(ring[j] for j in gap_arc)

        inner_theta = _loop_thetas(verts, inner_main_start)
        inner_gap_theta = _loop_thetas(verts, inner_gap_start) if inner_gap_start else []

        # --- Common INNER flow boundary -------------------------------------------
        remaining = max(1.0e-6, 1.0 - t_inner_pair)
        inner_steps = max(2, int(math.ceil(axial_steps * remaining)))
        prev = inner_main_start
        for k in range(1, inner_steps + 1):
            t = t_inner_pair + remaining * (k / inner_steps)
            x = body_start_x + transition_len * t
            c = center_at(t)
            rr = ri_at(t)
            radii = [_circle_envelope_radius(th, n, phase, c, rr) for th in inner_theta]
            cur = _append_loop_ring_from_polar(verts, x, inner_theta, radii)
            _connect_loops(faces, prev, cur, inward=True)
            prev = cur
        inner_final = prev

        # --- INNER face of merge spike --------------------------------------------
        if inner_gap_start:
            t_inner_close = _solve_transition_t(lambda t: center_at(t) - ri_at(t), lo=t_inner_pair)
            span = max(0.0, t_inner_close - t_inner_pair)
            spike_steps = max(2, int(math.ceil(axial_steps * max(span, 0.05))))
            prev = inner_gap_start
            for k in range(1, spike_steps):
                t = t_inner_pair + span * (k / spike_steps)
                x = body_start_x + transition_len * t
                c = center_at(t)
                rr = ri_at(t)
                radii = [_circle_gap_radius(th, n, phase, c, rr) for th in inner_gap_theta]
                cur = _append_loop_ring_from_polar(verts, x, inner_gap_theta, radii)
                # The central metal spike lies INSIDE this loop; its outward
                # normal points radially away from the axis.
                _connect_loops(faces, prev, cur, inward=False)
                prev = cur
            tip = len(verts)
            tip_x = body_start_x + transition_len * t_inner_close
            verts.append((tip_x, 0.0, 0.0))
            m = len(prev)
            for j in range(m):
                q = (j + 1) % m
                faces.append((prev[j], prev[q], tip))

        # The common outer/inner perimeter loops use identical primary arc
        # sampling, so they have matching counts at the round outlet.  Extend
        # both as one straight section and cap the downstream cut annulus.
        outlet_len = max(0.0, settings.outlet_length)
        if outlet_len > _EPS:
            x = total_length + outlet_len
            outer_out = _append_loop_ring_from_polar(verts, x, outer_theta, [ro1] * len(outer_theta))
            inner_out = _append_loop_ring_from_polar(verts, x, inner_theta, [ri1] * len(inner_theta))
            _connect_loops(faces, outer_final, outer_out, inward=False)
            _connect_loops(faces, inner_final, inner_out, inward=True)
            outer_final = outer_out
            inner_final = inner_out

        if len(outer_final) == len(inner_final):
            m = len(outer_final)
            for j in range(m):
                q = (j + 1) % m
                faces.append((outer_final[j], outer_final[q], inner_final[q], inner_final[j]))
    else:
        # Solid/surface-only mode: close the outlet perimeter with a face.  This
        # path is secondary; automotive exhaust use is expected to be hollow.
        outlet_len = max(0.0, settings.outlet_length)
        if outlet_len > _EPS:
            x = total_length + outlet_len
            outer_out = _append_loop_ring_from_polar(verts, x, outer_theta, [ro1] * len(outer_theta))
            _connect_loops(faces, outer_final, outer_out, inward=False)
            outer_final = outer_out
        faces.append(tuple(reversed(outer_final)))

    # Exact tangent endpoints generated independently by adjacent branches are
    # mathematically coincident.  Weld ONLY those coincident vertices so every
    # primary/body/spike junction is a shared mesh vertex instead of grouped
    # geometry occupying the same location.
    verts, faces = _weld_mesh_data(
        verts, faces,
        tolerance=max(1.0e-8, settings.primary_od * 1.0e-7),
        merge_min_x=body_start_x,
    )
    return verts, faces, sides


def _append_collector_outlet_fabrication(verts, faces, settings, base_x):
    """Replace the collector's square-cut outlet cap with optional end treatment/hardware.

    The existing final round outlet rings are reused as the first boundary of the
    added fabrication, so the collector outlet remains one connected mesh.
    """
    end_spec = collector_outlet_end_treatment_spec(settings)
    hw_spec = collector_outlet_connection_hardware_spec(settings, end_spec)
    hw_spec['profile_segments'] = max(16, int(getattr(settings, 'outlet_hardware_profile_segments', 64)))
    if not end_spec['active'] and not hw_spec['active']:
        return verts, faces, Vector((base_x, 0.0, 0.0)), end_spec, hw_spec

    ro = max(1.0e-6, float(settings.outlet_od) * 0.5)
    wall = max(0.0, min(float(settings.wall_thickness), ro * 0.999))
    ri = max(0.0, ro - wall)
    hollow = ri > 1.0e-6
    x_tol = max(1.0e-8, abs(base_x) * 1.0e-8, ro * 1.0e-7)
    r_tol = max(1.0e-7, ro * 2.0e-5)

    plane_ids = [i for i, co in enumerate(verts) if abs(float(co[0]) - base_x) <= x_tol]
    def radius_of(i):
        co = verts[i]
        return math.hypot(float(co[1]), float(co[2]))
    outer = [i for i in plane_ids if abs(radius_of(i) - ro) <= r_tol]
    inner = [i for i in plane_ids if hollow and abs(radius_of(i) - ri) <= r_tol]
    if len(outer) < 8 or (hollow and len(inner) != len(outer)):
        # Defensive fallback: preserve the validated collector if a future
        # topology revision changes the final-ring layout unexpectedly.
        return verts, faces, Vector((base_x, 0.0, 0.0)), {'type':'PLAIN','od':settings.outlet_od,'id':max(0.0,settings.outlet_od-2.0*wall),'extension':0.0,'active':False}, {'type':'NONE','active':False,'extension':0.0,'connector_od':settings.outlet_od,'connector_id':max(0.0,settings.outlet_od-2.0*wall)}

    outer.sort(key=lambda i: math.atan2(float(verts[i][2]), float(verts[i][1])))
    if hollow:
        inner.sort(key=lambda i: math.atan2(float(verts[i][2]), float(verts[i][1])))
    sides = len(outer)
    theta0 = math.atan2(float(verts[outer[0]][2]), float(verts[outer[0]][1]))
    angular_ref = Vector((0.0, math.cos(theta0), math.sin(theta0)))
    tangent = Vector((1.0, 0.0, 0.0))

    # Remove only the old planar outlet cap. Side faces have upstream vertices
    # and therefore remain untouched.
    new_faces = []
    for face in faces:
        if face and all(abs(float(verts[idx][0]) - base_x) <= x_tol for idx in face):
            continue
        new_faces.append(face)
    faces = new_faces

    outer_current = list(outer)
    inner_current = list(inner) if hollow else None
    target_ro = max(1.0e-6, float(end_spec['od']) * 0.5)
    target_ri = max(0.0, float(end_spec['id']) * 0.5)
    steps = max(2, int(getattr(settings, 'outlet_treatment_segments', 8)))
    x = float(base_x)

    if end_spec['active']:
        if end_spec['transition'] > _EPS:
            x0 = x
            for k in range(1, steps + 1):
                tt = k / steps
                sm = _smoothstep(tt)
                cx = x0 + end_spec['transition'] * tt
                r_out = ro + (target_ro - ro) * sm
                r_in = max(0.0, r_out - wall)
                no = _append_route_loop(verts, _route_circle_loop(Vector((cx,0.0,0.0)), tangent, angular_ref, r_out, sides))
                _connect_route_rings(faces, outer_current, no, False)
                outer_current = no
                if hollow:
                    ni = _append_route_loop(verts, _route_circle_loop(Vector((cx,0.0,0.0)), tangent, angular_ref, r_in, sides))
                    _connect_route_rings(faces, inner_current, ni, True)
                    inner_current = ni
            x = x0 + end_spec['transition']
        elif abs(target_ro - ro) > _EPS:
            no = _append_route_loop(verts, _route_circle_loop(Vector((x,0.0,0.0)), tangent, angular_ref, target_ro, sides))
            _connect_route_rings(faces, outer_current, no, False); outer_current = no
            if hollow:
                ni = _append_route_loop(verts, _route_circle_loop(Vector((x,0.0,0.0)), tangent, angular_ref, target_ri, sides))
                _connect_route_rings(faces, inner_current, ni, True); inner_current = ni
        if end_spec['collar'] > _EPS:
            x += end_spec['collar']
            no = _append_route_loop(verts, _route_circle_loop(Vector((x,0.0,0.0)), tangent, angular_ref, target_ro, sides))
            _connect_route_rings(faces, outer_current, no, False); outer_current = no
            if hollow:
                ni = _append_route_loop(verts, _route_circle_loop(Vector((x,0.0,0.0)), tangent, angular_ref, target_ri, sides))
                _connect_route_rings(faces, inner_current, ni, True); inner_current = ni

    hardware_base = Vector((x, 0.0, 0.0))
    connector_point = hardware_base.copy()
    if hw_spec['active']:
        if hw_spec['type'] == 'VBAND':
            connector_point, outer_current, inner_current = _append_vband_forward(
                verts, faces, hardware_base, tangent, angular_ref,
                outer_current, inner_current, target_ro, target_ri, hw_spec, sides)
        else:
            connector_point, outer_current, inner_current = _append_flat_flange_forward(
                verts, faces, hardware_base, tangent, angular_ref,
                outer_current, inner_current, target_ro, target_ri, hw_spec, sides)
    else:
        if hollow:
            for j in range(sides):
                q=(j+1)%sides
                faces.append((outer_current[j], outer_current[q], inner_current[q], inner_current[j]))
        else:
            faces.append(tuple(reversed(outer_current)))
    return verts, faces, Vector(connector_point), end_spec, hw_spec


def rebuild_collector_object(obj):
    if not obj or obj.name in _rebuild_guard or not getattr(obj, "exhaust_collector", None):
        return
    settings = obj.exhaust_collector
    if not settings.is_collector:
        return
    _rebuild_guard.add(obj.name)
    try:
        n = max(2, min(12, settings.primary_count))
        requested_inlet_radius = collector_radius(settings)
        total_length = max(0.001, settings.collector_length)
        phase = settings.radial_phase

        ro = max(1.0e-6, settings.primary_od * 0.5)
        sin_half_sector = max(math.sin(math.pi / n), 1.0e-6)
        tangent_center_radius = ro / sin_half_sector
        inlet_radius = max(requested_inlet_radius, tangent_center_radius)

        transition_len = max(
            settings.outlet_od * 0.25,
            min(settings.transition_length, total_length * 0.90),
        )
        body_start_x = max(total_length * 0.05, total_length - transition_len)

        if n == 2:
            # Keep the proven tangent-shell 2:1 path for this corrective build.
            # The shared-vertex multi-loop saddle introduced below is intended
            # for collectors with a finite central interstitial merge spike
            # (3:1 and higher).  2:1 will move to the dedicated Y-pipe topology
            # rather than forcing a degenerate zero-area spike into this solver.
            all_verts, all_faces = [], []
            for i in range(n):
                phi = phase + 2.0 * math.pi * i / n
                pts = collector_branch_points(
                    phi, inlet_radius, tangent_center_radius,
                    body_start_x, settings.branch_samples,
                )
                pv, pf = tube_mesh_data(
                    pts, settings.primary_od, settings.wall_thickness,
                    settings.profile_segments, cap_start=True, cap_end=False,
                )
                _append_mesh_piece(all_verts, all_faces, pv, pf)
            pv, pf = collector_loft_mesh_data(
                settings, body_start_x, tangent_center_radius, total_length
            )
            _append_mesh_piece(all_verts, all_faces, pv, pf)
            verts, faces = all_verts, all_faces
            actual_primary_sides = max(12, int(settings.profile_segments))
            unified = False
        else:
            verts, faces, actual_primary_sides = collector_unified_mesh_data(
                settings, inlet_radius, tangent_center_radius, body_start_x, total_length
            )
            unified = True

        outlet_len = max(0.0, settings.outlet_length)
        base_outlet_x = total_length + outlet_len
        verts, faces, collector_end_point, outlet_end_spec, outlet_hw_spec = _append_collector_outlet_fabrication(
            verts, faces, settings, base_outlet_x
        )
        _replace_mesh(obj, verts, faces)

        settings.computed_radial_spread = inlet_radius
        if hasattr(settings, "computed_body_start"):
            settings.computed_body_start = body_start_x
        if hasattr(settings, "computed_join_radius"):
            settings.computed_join_radius = tangent_center_radius

        obj["exhaust_collector_count"] = n
        obj["exhaust_radial_symmetry"] = True
        obj["exhaust_radial_phase"] = phase
        obj["exhaust_collector_merge"] = "UNIFIED_SHARED_VERTEX_CONCAVE_SPIKE_V1"
        obj["exhaust_primary_profile_segments_actual"] = actual_primary_sides
        obj["exhaust_single_manifold_mesh"] = unified
        obj["exhaust_primary_overlap"] = False
        obj["exhaust_merge_spike_integrated"] = (n > 2 and unified)
        obj["exhaust_primary_tangent_radius"] = tangent_center_radius

        # Publish every radial inlet as an addressable connector. Header-primary
        # mapping uses these exact branch-start centers/tangents rather than a
        # synthetic generic collector start connector.
        for old_i in range(12):
            for suffix in ("", "_tangent", "_od", "_id"):
                key = f"exhaust_collector_inlet_{old_i}{suffix}"
                if key in obj:
                    try:
                        del obj[key]
                    except Exception:
                        pass
        inlet_id = max(0.0, settings.primary_od - 2.0 * settings.wall_thickness)
        for i in range(n):
            phi = phase + 2.0 * math.pi * i / n
            center, tangent, _u, _v = _collector_branch_frame(
                phi, 0.0, inlet_radius, tangent_center_radius, body_start_x
            )
            obj[f"exhaust_collector_inlet_{i}"] = tuple(center)
            obj[f"exhaust_collector_inlet_{i}_tangent"] = tuple(tangent)
            obj[f"exhaust_collector_inlet_{i}_od"] = float(settings.primary_od)
            obj[f"exhaust_collector_inlet_{i}_id"] = float(inlet_id)
        obj["exhaust_collector_inlet_count"] = int(n)

        _set_connector_metadata(
            obj,
            Vector((0.0, inlet_radius, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            collector_end_point,
            Vector((1.0, 0.0, 0.0)),
        )
        obj["exhaust_connector_end_od"] = float(outlet_hw_spec['connector_od'] if outlet_hw_spec['active'] else outlet_end_spec['od'])
        obj["exhaust_connector_end_id"] = float(outlet_hw_spec['connector_id'] if outlet_hw_spec['active'] else outlet_end_spec['id'])
        obj["exhaust_connector_end_type"] = str(outlet_hw_spec['type'] if outlet_hw_spec['active'] else outlet_end_spec['type'])
        obj["exhaust_collector_has_outlet_treatment"] = bool(outlet_end_spec['active'])
        obj["exhaust_collector_has_outlet_hardware"] = bool(outlet_hw_spec['active'])
    finally:
        _rebuild_guard.discard(obj.name)



def _y_profile_sides(requested):
    """Profile resolution with exact vertices on the in-plane merge tangent."""
    requested = max(12, int(requested))
    return ((requested + 3) // 4) * 4


def _connect_open_strips(faces, a, b, inward=False):
    """Connect equal-length open arc rows. Endpoints are not wrapped together."""
    if len(a) != len(b) or len(a) < 2:
        return
    for j in range(len(a) - 1):
        q = j + 1
        if inward:
            faces.append((a[j], b[j], b[q], a[q]))
        else:
            faces.append((a[j], a[q], b[q], b[j]))


def _y_hermite_value(s, y0, y1, slope0, length):
    """Cubic Hermite branch centerline ending parallel to the outlet axis."""
    s = max(0.0, min(1.0, s))
    h00 = 2.0 * s**3 - 3.0 * s**2 + 1.0
    h10 = s**3 - 2.0 * s**2 + s
    h01 = -2.0 * s**3 + 3.0 * s**2
    y = h00 * y0 + h10 * length * slope0 + h01 * y1

    dh00 = 6.0 * s**2 - 6.0 * s
    dh10 = 3.0 * s**2 - 4.0 * s + 1.0
    dh01 = -6.0 * s**2 + 6.0 * s
    dy_ds = dh00 * y0 + dh10 * length * slope0 + dh01 * y1
    dy_dx = dy_ds / max(length, 1.0e-9)
    return y, dy_dx


def _y_formed_value(s, y0, y1, slope0, length):
    """Quintic branch curve with zero curvature at both ends.

    This is used by the formed/organic topology so the tube leaves its inlet
    tangent cleanly and arrives parallel to the outlet without a visible change
    in curvature at either end.
    """
    s = max(0.0, min(1.0, s))
    a0 = y0
    a1 = length * slope0
    d = y1 - y0 - a1
    a3 = 10.0 * d + 4.0 * a1
    a4 = -15.0 * d - 7.0 * a1
    a5 = 6.0 * d + 3.0 * a1
    y = a0 + a1*s + a3*s**3 + a4*s**4 + a5*s**5
    dy_ds = a1 + 3.0*a3*s**2 + 4.0*a4*s**3 + 5.0*a5*s**4
    return y, dy_ds / max(length, 1.0e-9)


def _y_late_blend_value(s, y0, y1, slope0, length, straight_fraction):
    """Hold a straight inlet tangent, then perform the terminal merge sweep."""
    s = max(0.0, min(1.0, s))
    f = max(0.0, min(0.90, straight_fraction))
    if f <= 1.0e-6:
        return _y_hermite_value(s, y0, y1, slope0, length)
    if s <= f:
        x = length * s
        return y0 + slope0 * x, slope0
    x0 = length * f
    y_mid = y0 + slope0 * x0
    rem = max(1.0e-9, length - x0)
    local_s = (s - f) / max(1.0e-9, 1.0 - f)
    return _y_hermite_value(local_s, y_mid, y1, slope0, rem)


def _y_branch_frame(s, length, y0, y1, slope0, mode='SWEPT', straight_fraction=0.70):
    if mode == 'CLASSIC':
        y, dy_dx = _y_late_blend_value(s, y0, y1, slope0, length, straight_fraction)
    elif mode == 'FORMED':
        y, dy_dx = _y_formed_value(s, y0, y1, slope0, length)
    else:
        y, dy_dx = _y_hermite_value(s, y0, y1, slope0, length)
    center = Vector((length * s, y, 0.0))
    tangent = _safe_normal(Vector((1.0, dy_dx, 0.0)), Vector((1.0, 0.0, 0.0)))
    # u is the in-plane cross-section axis and always tends to +Y at the merge.
    u = _safe_normal(Vector((-dy_dx, 1.0, 0.0)), Vector((0.0, 1.0, 0.0)))
    v = _safe_normal(tangent.cross(u), Vector((0.0, 0.0, 1.0)))
    u = _safe_normal(v.cross(tangent), u)
    return center, tangent, u, v


def _y_transition_mix(t, profile='SMOOTH'):
    t = max(0.0, min(1.0, t))
    if profile == 'FORMED':
        # smootherstep: zero first and second derivative at both ends
        return t*t*t*(t*(t*6.0 - 15.0) + 10.0)
    return _smoothstep(t)


def _y_circle_state(t, ro_a, ro_b, outlet_r, target_y=0.0, profile='SMOOTH'):
    sm = _y_transition_mix(t, profile)
    ca = ro_a * (1.0 - sm) + target_y * sm
    cb = -ro_b * (1.0 - sm) + target_y * sm
    ra = ro_a + (outlet_r - ro_a) * sm
    rb = ro_b + (outlet_r - ro_b) * sm
    return ca, ra, cb, rb


def _y_inner_circle_state(t, ro_a, ro_b, ri_a, ri_b, outlet_ri, target_y=0.0, profile='SMOOTH'):
    sm = _y_transition_mix(t, profile)
    ca = ro_a * (1.0 - sm) + target_y * sm
    cb = -ro_b * (1.0 - sm) + target_y * sm
    ra = ri_a + (outlet_ri - ri_a) * sm
    rb = ri_b + (outlet_ri - ri_b) * sm
    return ca, ra, cb, rb


def _y_exposed_arc_angles(ca, ra, cb, rb, which):
    """Return the exposed union-boundary arc for two Y/Z-plane circles.

    Circle A is the +Y branch; circle B is the -Y branch.  The returned angle
    interval is unwrapped so sampling always follows increasing local circle angle.
    At exact outer tangency A and B each contribute a complete circle; as their
    centers collapse toward the outlet they smoothly become opposite semicircles.
    """
    d = ca - cb
    if d <= 1.0e-10:
        if which == 'A':
            return 1.5 * math.pi, 2.5 * math.pi
        return 0.5 * math.pi, 1.5 * math.pi

    # Exact external tangency deserves an exact full-circle parameterization.
    # Without this guard, floating-point sqrt noise at z≈0 can leave tiny seam
    # slivers when an exposed union arc hands back to a conventional tube ring.
    tangency_tol = max(1.0e-12, (ra + rb) * 1.0e-9)
    if abs(d - (ra + rb)) <= tangency_tol:
        if which == 'A':
            return math.pi, 3.0 * math.pi
        return 0.0, 2.0 * math.pi

    # Standard two-circle intersection measured along the +Y/-Y center line.
    a = (ra * ra - rb * rb + d * d) / (2.0 * d)
    rel_a_y = -a
    z2 = max(0.0, ra * ra - a * a)
    z = math.sqrt(z2)
    y_int = ca + rel_a_y
    rel_b_y = y_int - cb

    alpha_a = math.atan2(z, rel_a_y)
    if alpha_a < 0.0:
        alpha_a += 2.0 * math.pi
    alpha_b = math.atan2(z, rel_b_y)
    if alpha_b < 0.0:
        alpha_b += 2.0 * math.pi

    if which == 'A':
        # lower intersection -> +Y outside -> upper intersection
        return 2.0 * math.pi - alpha_a, 2.0 * math.pi + alpha_a
    # upper intersection -> -Y outside -> lower intersection
    return alpha_b, 2.0 * math.pi - alpha_b


def _append_y_arc_row(verts, x, ca, ra, cb, rb, sides, which):
    start, end = _y_exposed_arc_angles(ca, ra, cb, rb, which)
    center_y = ca if which == 'A' else cb
    radius = ra if which == 'A' else rb
    refs = []
    for j in range(sides + 1):
        u = j / sides
        a = start + (end - start) * u
        refs.append(len(verts))
        verts.append((x, center_y + math.cos(a) * radius, math.sin(a) * radius))
    return refs


def _y_branch_tangent_sequence(ring, sides, which):
    if which == 'A':
        start = sides // 2  # -Y point on the +Y inlet circle
    else:
        start = 0           # +Y point on the -Y inlet circle
    return [ring[(start + j) % sides] for j in range(sides + 1)]


def _y_combined_loop(upper_arc, lower_arc):
    # upper: lower pole -> +Y side -> upper pole
    # lower: upper pole -> -Y side -> lower pole
    return list(upper_arc[:-1]) + list(lower_arc[:-1])


def y_pipe_mesh_data(settings):
    """Dedicated 2->1 automotive Y-pipe as one welded manifold shell.

    The outer branch circles meet at an exact saddle/tangency station before a
    shared two-lobe union surface morphs to the round outlet.  The inner bores
    remain separate until their own later tangency station, automatically leaving
    a finite-thickness metal merge web between the two flow passages.
    """
    topology = getattr(settings, 'topology', 'SWEPT')
    symmetric = bool(settings.symmetric) and topology != 'TANGENT'
    od_a = max(1.0e-6, settings.inlet_a_od)
    od_b = od_a if symmetric else max(1.0e-6, settings.inlet_b_od)
    ro_a = 0.5 * od_a
    ro_b = 0.5 * od_b
    ro_out = max(1.0e-6, 0.5 * settings.outlet_od)

    wall = max(0.0, min(settings.wall_thickness, min(ro_a, ro_b, ro_out) * 0.999))
    hollow = wall > 1.0e-6
    ri_a = max(1.0e-6, ro_a - wall)
    ri_b = max(1.0e-6, ro_b - wall)
    ri_out = max(1.0e-6, ro_out - wall)

    angle_a = max(0.0, min(math.radians(80.0), settings.inlet_a_angle))
    angle_b = angle_a if symmetric else max(0.0, min(math.radians(80.0), settings.inlet_b_angle))
    branch_length = max(1.0e-6, settings.branch_length)
    transition_length = max(1.0e-6, settings.merge_length)
    outlet_len = max(0.0, settings.outlet_length)
    sides = _y_profile_sides(settings.profile_segments)
    branch_steps = max(4, int(settings.branch_samples))
    transition_steps = max(6, int(settings.transition_segments))

    # Every topology hands off to the same proven shared-saddle merge at exact
    # outer tangency.  What changes is how the inlet centerlines approach that
    # station and where the final common outlet is biased.
    y_a1 = ro_a
    y_b1 = -ro_b
    slope_a0 = -math.tan(angle_a)
    slope_b0 = math.tan(angle_b)
    y_a0 = y_a1 + math.tan(angle_a) * branch_length
    y_b0 = y_b1 - math.tan(angle_b) * branch_length
    mode_a = mode_b = 'SWEPT'
    transition_profile = 'SMOOTH'
    outlet_center_y = 0.0
    straight_fraction = max(0.0, min(0.90, getattr(settings, 'straight_fraction', 0.70)))

    if topology == 'CLASSIC':
        # Long straight legs with only the terminal portion sweeping parallel to
        # the outlet before the shared saddle begins.
        mode_a = mode_b = 'CLASSIC'
    elif topology == 'PARALLEL':
        spacing = max(getattr(settings, 'inlet_spacing', ro_a + ro_b), ro_a + ro_b + 1.0e-6)
        midpoint = 0.5 * (y_a1 + y_b1)
        y_a0 = midpoint + 0.5 * spacing
        y_b0 = midpoint - 0.5 * spacing
        slope_a0 = slope_b0 = 0.0
        mode_a = mode_b = 'FORMED'
        transition_profile = 'FORMED'
    elif topology == 'TANGENT':
        main = getattr(settings, 'main_branch', 'A')
        bias = max(0.0, min(1.0, getattr(settings, 'outlet_bias', 0.75)))
        transition_profile = 'FORMED'
        if main == 'B':
            # B remains the near-straight run; A sweeps in from the side.
            y_b0 = y_b1
            slope_b0 = 0.0
            y_a0 = y_a1 + math.tan(angle_a) * branch_length
            slope_a0 = -math.tan(angle_a)
            outlet_center_y = y_b1 * bias
            mode_a, mode_b = 'FORMED', 'SWEPT'
        else:
            # A remains the near-straight run; B sweeps in from the side.
            y_a0 = y_a1
            slope_a0 = 0.0
            y_b0 = y_b1 - math.tan(angle_b) * branch_length
            slope_b0 = math.tan(angle_b)
            outlet_center_y = y_a1 * bias
            mode_a, mode_b = 'SWEPT', 'FORMED'
    elif topology == 'FORMED':
        mode_a = mode_b = 'FORMED'
        transition_profile = 'FORMED'
    elif topology == 'CUSTOM':
        spacing = max(getattr(settings, 'inlet_spacing', ro_a + ro_b), ro_a + ro_b + 1.0e-6)
        midpoint = 0.5 * (y_a1 + y_b1)
        y_a0 = midpoint + 0.5 * spacing
        y_b0 = midpoint - 0.5 * spacing
        # Preserve independently requested entry tangents while allowing a
        # controlled straight section before the terminal sweep.
        mode_a = mode_b = 'CLASSIC' if straight_fraction > 1.0e-4 else 'SWEPT'
        signed_bias = max(-1.0, min(1.0, getattr(settings, 'custom_outlet_bias', 0.0)))
        outlet_center_y = signed_bias * (ro_a if signed_bias >= 0.0 else ro_b)

    verts = []
    faces = []
    branch_outer_end = {}
    branch_inner_end = {}
    inlet_frames = {}

    for which, ro, ri, y0, y1, slope0 in (
        ('A', ro_a, ri_a, y_a0, y_a1, slope_a0),
        ('B', ro_b, ri_b, y_b0, y_b1, slope_b0),
    ):
        outer_rings = []
        inner_rings = []
        for k in range(branch_steps + 1):
            t = k / branch_steps
            center, tangent, u, v = _y_branch_frame(t, branch_length, y0, y1, slope0, mode=(mode_a if which == 'A' else mode_b), straight_fraction=straight_fraction)
            outer_rings.append(_append_ring(verts, center, u, v, ro, sides))
            if hollow:
                inner_rings.append(_append_ring(verts, center, u, v, ri, sides))
            if k == 0:
                inlet_frames[which] = (center.copy(), tangent.copy())

        for k in range(branch_steps):
            _connect_loops(faces, outer_rings[k], outer_rings[k + 1], inward=False)
            if hollow:
                _connect_loops(faces, inner_rings[k], inner_rings[k + 1], inward=True)

        if hollow:
            o = outer_rings[0]
            inn = inner_rings[0]
            for j in range(sides):
                q = (j + 1) % sides
                faces.append((o[j], inn[j], inn[q], o[q]))

        branch_outer_end[which] = outer_rings[-1]
        if hollow:
            branch_inner_end[which] = inner_rings[-1]

    # --- Outer saddle / shared Y body --------------------------------------------
    prev_a = _y_branch_tangent_sequence(branch_outer_end['A'], sides, 'A')
    prev_b = _y_branch_tangent_sequence(branch_outer_end['B'], sides, 'B')
    outer_final_a = prev_a
    outer_final_b = prev_b

    for k in range(1, transition_steps + 1):
        t = k / transition_steps
        x = branch_length + transition_length * t
        ca, ra, cb, rb = _y_circle_state(t, ro_a, ro_b, ro_out, outlet_center_y, transition_profile)
        cur_a = _append_y_arc_row(verts, x, ca, ra, cb, rb, sides, 'A')
        cur_b = _append_y_arc_row(verts, x, ca, ra, cb, rb, sides, 'B')
        _connect_open_strips(faces, prev_a, cur_a, inward=False)
        _connect_open_strips(faces, prev_b, cur_b, inward=False)
        prev_a, prev_b = cur_a, cur_b
    outer_final_a, outer_final_b = prev_a, prev_b

    inner_final_a = inner_final_b = None
    if hollow:
        # Inner bores keep their full circular walls until their own first tangency.
        def inner_gap_fn(t):
            ca, ra, cb, rb = _y_inner_circle_state(t, ro_a, ro_b, ri_a, ri_b, ri_out, outlet_center_y, transition_profile)
            return (ca - cb) - (ra + rb)

        t_inner = _solve_transition_t(inner_gap_fn)
        pre_steps = max(1, int(math.ceil(transition_steps * max(t_inner, 0.02))))
        tangent_inner = {}
        for which in ('A', 'B'):
            prev = branch_inner_end[which]
            for k in range(1, pre_steps + 1):
                t = t_inner * (k / pre_steps)
                x = branch_length + transition_length * t
                ca, ra, cb, rb = _y_inner_circle_state(t, ro_a, ro_b, ri_a, ri_b, ri_out, outlet_center_y, transition_profile)
                cy = ca if which == 'A' else cb
                rr = ra if which == 'A' else rb
                cur = _append_ring(
                    verts, Vector((x, cy, 0.0)),
                    Vector((0.0, 1.0, 0.0)), Vector((0.0, 0.0, 1.0)), rr, sides,
                )
                _connect_loops(faces, prev, cur, inward=True)
                prev = cur
            tangent_inner[which] = prev

        prev_a = _y_branch_tangent_sequence(tangent_inner['A'], sides, 'A')
        prev_b = _y_branch_tangent_sequence(tangent_inner['B'], sides, 'B')
        remaining = max(1.0e-6, 1.0 - t_inner)
        inner_steps = max(2, int(math.ceil(transition_steps * remaining)))
        for k in range(1, inner_steps + 1):
            t = t_inner + remaining * (k / inner_steps)
            x = branch_length + transition_length * t
            ca, ra, cb, rb = _y_inner_circle_state(t, ro_a, ro_b, ri_a, ri_b, ri_out, outlet_center_y, transition_profile)
            cur_a = _append_y_arc_row(verts, x, ca, ra, cb, rb, sides, 'A')
            cur_b = _append_y_arc_row(verts, x, ca, ra, cb, rb, sides, 'B')
            _connect_open_strips(faces, prev_a, cur_a, inward=True)
            _connect_open_strips(faces, prev_b, cur_b, inward=True)
            prev_a, prev_b = cur_a, cur_b
        inner_final_a, inner_final_b = prev_a, prev_b

    # --- Straight round outlet, split only by two longitudinal topology seams -----
    body_end_x = branch_length + transition_length
    if outlet_len > _EPS:
        out_x = body_end_x + outlet_len
        cur_a = _append_y_arc_row(verts, out_x, outlet_center_y, ro_out, outlet_center_y, ro_out, sides, 'A')
        cur_b = _append_y_arc_row(verts, out_x, outlet_center_y, ro_out, outlet_center_y, ro_out, sides, 'B')
        _connect_open_strips(faces, outer_final_a, cur_a, inward=False)
        _connect_open_strips(faces, outer_final_b, cur_b, inward=False)
        outer_final_a, outer_final_b = cur_a, cur_b

        if hollow:
            ia = _append_y_arc_row(verts, out_x, outlet_center_y, ri_out, outlet_center_y, ri_out, sides, 'A')
            ib = _append_y_arc_row(verts, out_x, outlet_center_y, ri_out, outlet_center_y, ri_out, sides, 'B')
            _connect_open_strips(faces, inner_final_a, ia, inward=True)
            _connect_open_strips(faces, inner_final_b, ib, inward=True)
            inner_final_a, inner_final_b = ia, ib

    if hollow:
        outer_loop = _y_combined_loop(outer_final_a, outer_final_b)
        inner_loop = _y_combined_loop(inner_final_a, inner_final_b)
        if len(outer_loop) == len(inner_loop):
            m = len(outer_loop)
            for j in range(m):
                q = (j + 1) % m
                faces.append((outer_loop[j], outer_loop[q], inner_loop[q], inner_loop[j]))
    else:
        # Surface-only fallback: close the final outlet loop.
        faces.append(tuple(reversed(_y_combined_loop(outer_final_a, outer_final_b))))

    # Weld exact saddle/intersection seams only. Upstream branch tubes remain distinct.
    verts, faces = _weld_mesh_data(
        verts, faces,
        tolerance=max(1.0e-8, min(od_a, od_b) * 1.0e-7),
        merge_min_x=branch_length,
    )
    return verts, faces, sides, (y_a0 - y_b0), inlet_frames, outlet_center_y, topology


def rebuild_y_pipe_object(obj):
    if not obj or obj.name in _rebuild_guard or not getattr(obj, "exhaust_y_pipe", None):
        return
    settings = obj.exhaust_y_pipe
    if not settings.is_y_pipe:
        return
    _rebuild_guard.add(obj.name)
    try:
        verts, faces, sides, spacing, inlet_frames, outlet_center_y, topology = y_pipe_mesh_data(settings)
        _replace_mesh(obj, verts, faces)
        settings.computed_inlet_spacing = spacing
        if hasattr(settings, "computed_outlet_offset"):
            settings.computed_outlet_offset = outlet_center_y

        body_end_x = max(1.0e-6, settings.branch_length) + max(1.0e-6, settings.merge_length)
        end_x = body_end_x + max(0.0, settings.outlet_length)
        a_point, a_tangent = inlet_frames['A']
        b_point, b_tangent = inlet_frames['B']
        _set_connector_metadata(
            obj, a_point, a_tangent,
            Vector((end_x, outlet_center_y, 0.0)), Vector((1.0, 0.0, 0.0)),
        )
        obj["exhaust_connector_inlet_a"] = tuple(a_point)
        obj["exhaust_connector_inlet_a_tangent"] = tuple(a_tangent)
        obj["exhaust_connector_inlet_b"] = tuple(b_point)
        obj["exhaust_connector_inlet_b_tangent"] = tuple(b_tangent)
        obj["exhaust_y_pipe_single_manifold_mesh"] = True
        obj["exhaust_y_pipe_shared_saddle"] = True
        obj["exhaust_y_pipe_profile_segments_actual"] = sides
        obj["exhaust_y_pipe_topology"] = topology
        obj["exhaust_y_pipe_outlet_offset_y"] = outlet_center_y
    finally:
        _rebuild_guard.discard(obj.name)


def _ring(center_x, radius, sides):
    return [(center_x, math.cos(2.0 * math.pi * j / sides) * radius, math.sin(2.0 * math.pi * j / sides) * radius) for j in range(sides)]



# -----------------------------------------------------------------------------
# Automotive 2->2 X-pipe solver
# -----------------------------------------------------------------------------

def _x_hermite_value(s, y0, y1, slope0, slope1, length):
    """General cubic Hermite centerline with independent end tangents."""
    s = max(0.0, min(1.0, s))
    h00 = 2.0 * s**3 - 3.0 * s**2 + 1.0
    h10 = s**3 - 2.0 * s**2 + s
    h01 = -2.0 * s**3 + 3.0 * s**2
    h11 = s**3 - s**2
    y = h00 * y0 + h10 * length * slope0 + h01 * y1 + h11 * length * slope1

    dh00 = 6.0 * s**2 - 6.0 * s
    dh10 = 3.0 * s**2 - 4.0 * s + 1.0
    dh01 = -6.0 * s**2 + 6.0 * s
    dh11 = 3.0 * s**2 - 2.0 * s
    dy_ds = dh00 * y0 + dh10 * length * slope0 + dh01 * y1 + dh11 * length * slope1
    dy_dx = dy_ds / max(length, 1.0e-9)
    return y, dy_dx


def _x_branch_frame(s, x0, length, y0, y1, slope0, slope1):
    y, dy_dx = _x_hermite_value(s, y0, y1, slope0, slope1, length)
    center = Vector((x0 + length * s, y, 0.0))
    tangent = _safe_normal(Vector((1.0, dy_dx, 0.0)), Vector((1.0, 0.0, 0.0)))
    u = _safe_normal(Vector((-dy_dx, 1.0, 0.0)), Vector((0.0, 1.0, 0.0)))
    v = _safe_normal(tangent.cross(u), Vector((0.0, 0.0, 1.0)))
    u = _safe_normal(v.cross(tangent), u)
    return center, tangent, u, v


def _x_profile_exponent(settings):
    topo = settings.topology
    if topo == 'SWEPT':
        return 0.70
    if topo == 'PARALLEL':
        return 0.90
    if topo == 'CUSTOM':
        return max(0.35, min(3.0, float(settings.custom_profile)))
    return 2.10  # CLASSIC: compact crossover concentrated near the center


def _x_state(t, ro_a0, ro_b0, ro_a1, ro_b1, wall, opening, exponent):
    """Two-circle state across the communicating section of an X-pipe.

    At t=0 and t=1 the OUTER circles are exactly tangent. Around t=0.5 the
    center spacing becomes smaller than the INNER-radius sum, so the two bores
    genuinely communicate.  Outer and inner tangencies therefore happen at
    different axial stations, naturally preserving a finite-thickness web.
    """
    t = max(0.0, min(1.0, t))
    sm = _smoothstep(t)
    ra = ro_a0 + (ro_a1 - ro_a0) * sm
    rb = ro_b0 + (ro_b1 - ro_b0) * sm
    ria = max(1.0e-6, ra - wall)
    rib = max(1.0e-6, rb - wall)

    # sin(pi*t)^p is zero at both tangent ends and one at the crossover center.
    if t <= 1.0e-12 or t >= 1.0 - 1.0e-12:
        q = 0.0
    else:
        q = max(0.0, math.sin(math.pi * t)) ** exponent
    opening = max(0.05, min(1.0, float(opening)))
    # 5% opening is already enough to make the internal bores communicate;
    # 100% gives a broad opening while deliberately avoiding coincident centers.
    open_distance = (ria + rib) * (1.0 - 0.85 * opening)
    # With unequal tube diameters, do not let the smaller circle become fully
    # contained inside the larger one. The X solver intentionally maintains a
    # two-lobe intersecting-circle topology throughout the crossover so both
    # flow paths retain a visible, controllable contribution to the common body.
    containment_floor = max(abs(ra - rb), abs(ria - rib)) + min(ria, rib) * 0.01
    open_distance = max(open_distance, containment_floor)
    tangent_distance = ra + rb
    d = tangent_distance * (1.0 - q) + open_distance * q

    total_r = max(ra + rb, 1.0e-9)
    ca = d * (ra / total_r)
    cb = -d * (rb / total_r)
    return ca, ra, cb, rb, ria, rib


def _x_gap_value(t, *state_args):
    ca, ra, cb, rb, ria, rib = _x_state(t, *state_args)
    return (ca - cb) - (ria + rib)


def _bisect_sign_change(fn, lo, hi, want_positive_at_lo=True):
    flo = fn(lo)
    fhi = fn(hi)
    if abs(flo) < 1.0e-12:
        return lo
    if abs(fhi) < 1.0e-12:
        return hi
    for _ in range(56):
        mid = 0.5 * (lo + hi)
        fm = fn(mid)
        if abs(fm) < 1.0e-12:
            return mid
        if want_positive_at_lo:
            if fm > 0.0:
                lo = mid
            else:
                hi = mid
        else:
            if fm < 0.0:
                lo = mid
            else:
                hi = mid
    return 0.5 * (lo + hi)


def _x_rotate_about_axis(co, angle):
    if abs(angle) < 1.0e-12:
        return tuple(co)
    x, y, z = co
    c = math.cos(angle)
    ss = math.sin(angle)
    return (x, y * c - z * ss, y * ss + z * c)


def _x_rotate_vector(v, angle):
    x, y, z = v
    c = math.cos(angle)
    ss = math.sin(angle)
    return Vector((x, y * c - z * ss, y * ss + z * c))


def x_pipe_mesh_data(settings):
    """Generate a four-port automotive X-pipe as one manifold shell.

    Separate hollow inlet tubes reach an exact OUTER tangency, transition through
    a two-lobe common shell, and separate again at the downstream outer tangency.
    The INNER bores merge later and split earlier, creating a real communicating
    crossover opening while maintaining metal wall/web thickness throughout.
    """
    symmetric = bool(settings.symmetric)
    od_a0 = max(1.0e-6, settings.pipe_a_od)
    od_b0 = od_a0 if symmetric else max(1.0e-6, settings.pipe_b_od)
    # v0.5 uses constant A/B diameters from inlet to outlet. Keeping the state
    # solver end-radius aware leaves room for reducer-style X topologies later.
    od_a1 = od_a0
    od_b1 = od_b0
    ro_a0, ro_b0 = 0.5 * od_a0, 0.5 * od_b0
    ro_a1, ro_b1 = 0.5 * od_a1, 0.5 * od_b1
    wall = max(0.0, min(settings.wall_thickness, min(ro_a0, ro_b0, ro_a1, ro_b1) * 0.999))
    hollow = wall > 1.0e-6
    ri_a0, ri_b0 = max(1.0e-6, ro_a0 - wall), max(1.0e-6, ro_b0 - wall)
    ri_a1, ri_b1 = max(1.0e-6, ro_a1 - wall), max(1.0e-6, ro_b1 - wall)

    sides = _y_profile_sides(settings.profile_segments)
    branch_steps = max(4, int(settings.branch_samples))
    cross_steps = max(10, int(settings.crossover_segments))
    in_len = max(1.0e-6, settings.inlet_branch_length)
    cross_len = max(1.0e-6, settings.crossover_length)
    out_len = max(1.0e-6, settings.outlet_branch_length)

    min_in_spacing = ro_a0 + ro_b0
    min_out_spacing = ro_a1 + ro_b1
    in_spacing = max(float(settings.inlet_spacing), min_in_spacing)
    out_spacing = max(float(settings.outlet_spacing), min_out_spacing)

    # Split center-to-center spacing proportionally by pipe size so unequal A/B
    # layouts still meet at Y=0 at the exact tangent stations.
    sum_in = max(ro_a0 + ro_b0, 1.0e-9)
    sum_out = max(ro_a1 + ro_b1, 1.0e-9)
    y_a_in = in_spacing * ro_a0 / sum_in
    y_b_in = -in_spacing * ro_b0 / sum_in
    y_a_tan0, y_b_tan0 = ro_a0, -ro_b0
    y_a_tan1, y_b_tan1 = ro_a1, -ro_b1
    y_a_out = out_spacing * ro_a1 / sum_out
    y_b_out = -out_spacing * ro_b1 / sum_out

    if settings.topology == 'PARALLEL':
        ia = ib = oa = ob = 0.0
    else:
        ia = max(0.0, min(math.radians(75.0), settings.inlet_a_angle))
        ib = ia if symmetric else max(0.0, min(math.radians(75.0), settings.inlet_b_angle))
        oa = max(0.0, min(math.radians(75.0), settings.outlet_a_angle))
        ob = oa if symmetric else max(0.0, min(math.radians(75.0), settings.outlet_b_angle))

    # Positive-Y A approaches inward with negative inlet slope and departs outward
    # with positive outlet slope. B is the mirror convention.
    inlet_slopes = {'A': -math.tan(ia), 'B': math.tan(ib)}
    outlet_slopes = {'A': math.tan(oa), 'B': -math.tan(ob)}

    verts = []
    faces = []
    inlet_outer_end = {}
    inlet_inner_end = {}
    inlet_frames = {}

    # --- Inlet branches --------------------------------------------------------
    for which, ro, ri, y0, y1 in (
        ('A', ro_a0, ri_a0, y_a_in, y_a_tan0),
        ('B', ro_b0, ri_b0, y_b_in, y_b_tan0),
    ):
        outer_rings = []
        inner_rings = []
        for k in range(branch_steps + 1):
            u = k / branch_steps
            center, tangent, ax_u, ax_v = _x_branch_frame(
                u, 0.0, in_len, y0, y1, inlet_slopes[which], 0.0,
            )
            outer_rings.append(_append_ring(verts, center, ax_u, ax_v, ro, sides))
            if hollow:
                inner_rings.append(_append_ring(verts, center, ax_u, ax_v, ri, sides))
            if k == 0:
                inlet_frames[which] = (center.copy(), tangent.copy())

        for k in range(branch_steps):
            _connect_loops(faces, outer_rings[k], outer_rings[k + 1], inward=False)
            if hollow:
                _connect_loops(faces, inner_rings[k], inner_rings[k + 1], inward=True)
        if hollow:
            o, inn = outer_rings[0], inner_rings[0]
            for j in range(sides):
                q = (j + 1) % sides
                faces.append((o[j], inn[j], inn[q], o[q]))
        inlet_outer_end[which] = outer_rings[-1]
        if hollow:
            inlet_inner_end[which] = inner_rings[-1]

    exponent = _x_profile_exponent(settings)
    state_args = (ro_a0, ro_b0, ro_a1, ro_b1, wall, settings.crossover_opening, exponent)

    # --- Shared OUTER crossover: tangent -> overlap -> tangent ------------------
    prev_a = _y_branch_tangent_sequence(inlet_outer_end['A'], sides, 'A')
    prev_b = _y_branch_tangent_sequence(inlet_outer_end['B'], sides, 'B')
    outer_final_a = prev_a
    outer_final_b = prev_b
    for k in range(1, cross_steps + 1):
        t = k / cross_steps
        x = in_len + cross_len * t
        ca, ra, cb, rb, _, _ = _x_state(t, *state_args)
        cur_a = _append_y_arc_row(verts, x, ca, ra, cb, rb, sides, 'A')
        cur_b = _append_y_arc_row(verts, x, ca, ra, cb, rb, sides, 'B')
        _connect_open_strips(faces, prev_a, cur_a, inward=False)
        _connect_open_strips(faces, prev_b, cur_b, inward=False)
        prev_a, prev_b = cur_a, cur_b
    outer_final_a, outer_final_b = prev_a, prev_b

    # --- INNER bores: separate -> communicating union -> separate --------------
    inner_cross_end = {}
    if hollow:
        gap_fn = lambda t: _x_gap_value(t, *state_args)
        center_gap = gap_fn(0.5)
        # crossover_opening is clamped so this should be negative; retain a
        # defensive fallback in case future diameter-transition modes change it.
        if center_gap >= -1.0e-10:
            t_enter = t_exit = 0.5
        else:
            t_enter = _bisect_sign_change(gap_fn, 0.0, 0.5, want_positive_at_lo=True)
            t_exit = _bisect_sign_change(gap_fn, 0.5, 1.0, want_positive_at_lo=False)

        # Full separate inner circles from crossover start to first inner tangency.
        tang_in = {}
        pre_steps = max(1, int(math.ceil(cross_steps * max(t_enter, 0.02))))
        for which in ('A', 'B'):
            prev = inlet_inner_end[which]
            for k in range(1, pre_steps + 1):
                t = t_enter * (k / pre_steps)
                x = in_len + cross_len * t
                ca, _, cb, _, ria, rib = _x_state(t, *state_args)
                cy = ca if which == 'A' else cb
                rr = ria if which == 'A' else rib
                cur = _append_ring(
                    verts, Vector((x, cy, 0.0)),
                    Vector((0.0, 1.0, 0.0)), Vector((0.0, 0.0, 1.0)), rr, sides,
                )
                _connect_loops(faces, prev, cur, inward=True)
                prev = cur
            tang_in[which] = prev

        prev_a = _y_branch_tangent_sequence(tang_in['A'], sides, 'A')
        prev_b = _y_branch_tangent_sequence(tang_in['B'], sides, 'B')
        union_steps = max(2, int(math.ceil(cross_steps * max(t_exit - t_enter, 0.05))))
        tangent_exit_state = None
        for k in range(1, union_steps + 1):
            t = t_enter + (t_exit - t_enter) * (k / union_steps)
            x = in_len + cross_len * t
            ca, _, cb, _, ria, rib = _x_state(t, *state_args)
            if k == union_steps:
                # Make the downstream handoff an EXACT inner tangency. Bisection
                # is numerically very close, but swept/custom profiles can leave
                # enough sub-micron overlap for the arc endpoint not to weld to
                # the following full-circle ring. Preserve the current midpoint
                # while enforcing center distance == ria + rib.
                mid = 0.5 * (ca + cb)
                ca = mid + ria
                cb = mid - rib
                tangent_exit_state = (ca, cb, ria, rib)
            cur_a = _append_y_arc_row(verts, x, ca, ria, cb, rib, sides, 'A')
            cur_b = _append_y_arc_row(verts, x, ca, ria, cb, rib, sides, 'B')
            _connect_open_strips(faces, prev_a, cur_a, inward=True)
            _connect_open_strips(faces, prev_b, cur_b, inward=True)
            prev_a, prev_b = cur_a, cur_b

        # At second inner tangency, hand off from union arcs to full separated rings.
        if tangent_exit_state is None:
            ca, _, cb, _, ria, rib = _x_state(t_exit, *state_args)
            mid = 0.5 * (ca + cb)
            ca, cb = mid + ria, mid - rib
        else:
            ca, cb, ria, rib = tangent_exit_state
        start_a = _append_ring(verts, Vector((in_len + cross_len * t_exit, ca, 0.0)), Vector((0.0, 1.0, 0.0)), Vector((0.0, 0.0, 1.0)), ria, sides)
        start_b = _append_ring(verts, Vector((in_len + cross_len * t_exit, cb, 0.0)), Vector((0.0, 1.0, 0.0)), Vector((0.0, 0.0, 1.0)), rib, sides)
        # No zero-area bridge faces are needed; the seam weld below makes the arc
        # boundary and these rings share the same actual vertices.
        post_steps = max(1, int(math.ceil(cross_steps * max(1.0 - t_exit, 0.02))))
        for which, prev in (('A', start_a), ('B', start_b)):
            for k in range(1, post_steps + 1):
                t = t_exit + (1.0 - t_exit) * (k / post_steps)
                x = in_len + cross_len * t
                ca, _, cb, _, ria, rib = _x_state(t, *state_args)
                cy = ca if which == 'A' else cb
                rr = ria if which == 'A' else rib
                cur = _append_ring(verts, Vector((x, cy, 0.0)), Vector((0.0, 1.0, 0.0)), Vector((0.0, 0.0, 1.0)), rr, sides)
                _connect_loops(faces, prev, cur, inward=True)
                prev = cur
            inner_cross_end[which] = prev

    # --- Outlet branches -------------------------------------------------------
    cross_end_x = in_len + cross_len
    outlet_frames = {}
    for which, ro, ri, y0, y1 in (
        ('A', ro_a1, ri_a1, y_a_tan1, y_a_out),
        ('B', ro_b1, ri_b1, y_b_tan1, y_b_out),
    ):
        outer_rings = []
        inner_rings = []
        for k in range(branch_steps + 1):
            u = k / branch_steps
            center, tangent, ax_u, ax_v = _x_branch_frame(
                u, cross_end_x, out_len, y0, y1, 0.0, outlet_slopes[which],
            )
            outer_rings.append(_append_ring(verts, center, ax_u, ax_v, ro, sides))
            if hollow:
                inner_rings.append(_append_ring(verts, center, ax_u, ax_v, ri, sides))
            if k == branch_steps:
                outlet_frames[which] = (center.copy(), tangent.copy())

        # The first outlet ring is coincident with the final full-circle union arc
        # and is seam-welded below. Continue from it without an overlapping tube.
        for k in range(branch_steps):
            _connect_loops(faces, outer_rings[k], outer_rings[k + 1], inward=False)
            if hollow:
                _connect_loops(faces, inner_rings[k], inner_rings[k + 1], inward=True)

        if hollow:
            o, inn = outer_rings[-1], inner_rings[-1]
            for j in range(sides):
                q = (j + 1) % sides
                faces.append((o[j], o[q], inn[q], inn[j]))

    # Seam weld crossover tangencies and exact arc/full-ring handoffs. Upstream
    # separated inlet pipes remain distinct before their true junction station.
    verts, faces = _weld_mesh_data(
        verts, faces,
        tolerance=max(1.0e-8, min(od_a0, od_b0) * 1.0e-7),
        merge_min_x=in_len,
    )

    # Clock the finished X plane about +X after topology/welding calculations.
    plane_rot = float(settings.plane_rotation)
    if abs(plane_rot) > 1.0e-12:
        verts = [_x_rotate_about_axis(v, plane_rot) for v in verts]
        inlet_frames = {k: (p, _x_rotate_vector(t, plane_rot)) for k, (p, t) in inlet_frames.items()}
        outlet_frames = {k: (p, _x_rotate_vector(t, plane_rot)) for k, (p, t) in outlet_frames.items()}
        # Points themselves also need rotation.
        inlet_frames = {k: (Vector(_x_rotate_about_axis(tuple(p), plane_rot)), t) for k, (p, t) in inlet_frames.items()}
        outlet_frames = {k: (Vector(_x_rotate_about_axis(tuple(p), plane_rot)), t) for k, (p, t) in outlet_frames.items()}

    return verts, faces, sides, in_spacing, out_spacing, inlet_frames, outlet_frames


def rebuild_x_pipe_object(obj):
    if not obj or obj.name in _rebuild_guard or not getattr(obj, "exhaust_x_pipe", None):
        return
    settings = obj.exhaust_x_pipe
    if not settings.is_x_pipe:
        return
    _rebuild_guard.add(obj.name)
    try:
        verts, faces, sides, in_spacing, out_spacing, inlet_frames, outlet_frames = x_pipe_mesh_data(settings)
        _replace_mesh(obj, verts, faces)
        settings.computed_inlet_spacing = in_spacing
        settings.computed_outlet_spacing = out_spacing

        a0, at0 = inlet_frames['A']
        b0, bt0 = inlet_frames['B']
        a1, at1 = outlet_frames['A']
        b1, bt1 = outlet_frames['B']
        _set_connector_metadata(obj, a0, at0, a1, at1)
        obj["exhaust_connector_inlet_a"] = tuple(a0)
        obj["exhaust_connector_inlet_a_tangent"] = tuple(at0)
        obj["exhaust_connector_inlet_b"] = tuple(b0)
        obj["exhaust_connector_inlet_b_tangent"] = tuple(bt0)
        obj["exhaust_connector_outlet_a"] = tuple(a1)
        obj["exhaust_connector_outlet_a_tangent"] = tuple(at1)
        obj["exhaust_connector_outlet_b"] = tuple(b1)
        obj["exhaust_connector_outlet_b_tangent"] = tuple(bt1)
        obj["exhaust_x_pipe_single_manifold_mesh"] = True
        obj["exhaust_x_pipe_true_crossover"] = True
        obj["exhaust_x_pipe_profile_segments_actual"] = sides
    finally:
        _rebuild_guard.discard(obj.name)


# -----------------------------------------------------------------------------
# H-pipe geometry
# -----------------------------------------------------------------------------

def _h_wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _h_unique_angles(values, tol=1.0e-7):
    vals = sorted((v % (2.0 * math.pi)) for v in values)
    out = []
    for v in vals:
        if not out or abs(v - out[-1]) > tol:
            out.append(v)
    if len(out) > 1 and abs((out[0] + 2.0 * math.pi) - out[-1]) <= tol:
        out.pop()
    return out


def _h_port_half_angle(main_radius, branch_radius):
    if main_radius <= _EPS or branch_radius <= _EPS:
        return 0.0
    ratio = min(0.999999, max(0.0, branch_radius / main_radius))
    return math.asin(ratio)


def _h_port_bounds(theta, theta0, radius, branch_radius, axis_dx, axis_dy, station):
    """Exact infinite-cylinder intersection projected onto the main-cylinder X axis.

    The main cylinder runs along +X.  The crossover axis lies in XY and crosses
    the main centerline at *station*.  Only the inward-facing saddle half of the
    intersection is used by the caller.
    """
    half = _h_port_half_angle(radius, branch_radius)
    delta = _h_wrap_angle(theta - theta0)
    if abs(delta) > half + 1.0e-9:
        return station, station, False

    y = radius * math.cos(theta)
    z = radius * math.sin(theta)
    rem = branch_radius * branch_radius - z * z
    if rem < -1.0e-10:
        return station, station, False
    # Exact saddle endpoints should collapse to one shared vertex. Floating-point
    # evaluation of sin(asin()) can leave a tiny positive remainder that would
    # otherwise create a microscopic two-vertex slit in the inner wall.
    if rem <= max(1.0e-20, branch_radius * branch_radius * 1.0e-12):
        rem = 0.0
    else:
        rem = max(0.0, rem)
    dy = axis_dy
    if abs(dy) < 1.0e-5:
        dy = -1.0e-5 if dy < 0.0 else 1.0e-5
    center_shift = (axis_dx / dy) * y
    half_x = math.sqrt(rem) / abs(dy)
    return station + center_shift - half_x, station + center_shift + half_x, True


def _h_loop_project_orientation(verts, refs, center, u, v):
    if len(refs) < 3:
        return refs
    area2 = 0.0
    pts = []
    for ref in refs:
        rel = Vector(verts[ref]) - center
        pts.append((rel.dot(u), rel.dot(v)))
    for i, (x0, y0) in enumerate(pts):
        x1, y1 = pts[(i + 1) % len(pts)]
        area2 += x0 * y1 - x1 * y0
    return refs if area2 >= 0.0 else list(reversed(refs))


def _h_connect_loops_resampled(faces, a, b, inward=False):
    """Zip two closed loops with different vertex counts using triangles/quads."""
    na, nb = len(a), len(b)
    if na < 3 or nb < 3:
        return
    i = j = 0
    eps = 1.0e-12
    while i < na or j < nb:
        ai = a[i % na]
        bj = b[j % nb]
        fi = (i + 1) / na if i < na else float('inf')
        fj = (j + 1) / nb if j < nb else float('inf')
        if abs(fi - fj) <= eps:
            an = a[(i + 1) % na]
            bn = b[(j + 1) % nb]
            face = (ai, an, bn, bj)
            i += 1
            j += 1
        elif fi < fj:
            an = a[(i + 1) % na]
            face = (ai, an, bj)
            i += 1
        else:
            bn = b[(j + 1) % nb]
            face = (ai, bn, bj)
            j += 1
        if inward:
            face = tuple(reversed(face))
        faces.append(face)


def _h_append_ring(verts, center, axis, radius, sides):
    axis = _safe_normal(axis, Vector((0.0, -1.0, 0.0)))
    u = Vector((0.0, 0.0, 1.0))
    if abs(axis.dot(u)) > 0.999:
        u = Vector((0.0, 1.0, 0.0))
    u = _safe_normal(u - axis * u.dot(axis), Vector((0.0, 0.0, 1.0)))
    v = _safe_normal(axis.cross(u), Vector((1.0, 0.0, 0.0)))
    # u x v == axis, which matches _connect_loops' expected ring winding.
    return _append_ring(verts, center, u, v, radius, sides), u, v


def _h_main_surface_with_port(
    verts, faces, *, length, center_y, station, theta0, radius,
    branch_radius, axis_dx, axis_dy, theta_values, longitudinal_segments,
    inward=False,
):
    """Create one cylindrical surface with an exact saddle opening.

    The cylinder is split into upstream/downstream sheets.  They meet at the
    station everywhere except inside the saddle window, leaving a real boundary
    loop for the crossover surface to share.
    """
    ntheta = len(theta_values)
    pre_steps = max(2, int(longitudinal_segments * max(station / max(length, _EPS), 0.15)))
    post_steps = max(2, int(longitudinal_segments * max((length - station) / max(length, _EPS), 0.15)))

    pre = []
    post = []
    bounds = []
    for theta in theta_values:
        xl, xr, inside = _h_port_bounds(theta, theta0, radius, branch_radius, axis_dx, axis_dy, station)
        xl = max(0.0, min(length, xl))
        xr = max(0.0, min(length, xr))
        if xr < xl:
            xl, xr = xr, xl
        bounds.append((xl, xr, inside))

        prow = []
        for k in range(pre_steps + 1):
            t = k / pre_steps
            x = xl * t
            prow.append(len(verts))
            verts.append((x, center_y + radius * math.cos(theta), radius * math.sin(theta)))
        pre.append(prow)

        qrow = []
        for k in range(post_steps + 1):
            t = k / post_steps
            x = xr + (length - xr) * t
            qrow.append(len(verts))
            verts.append((x, center_y + radius * math.cos(theta), radius * math.sin(theta)))
        post.append(qrow)

    for j in range(ntheta):
        q = (j + 1) % ntheta
        for k in range(pre_steps):
            face = (pre[j][k], pre[q][k], pre[q][k + 1], pre[j][k + 1])
            faces.append(tuple(reversed(face)) if inward else face)
        for k in range(post_steps):
            face = (post[j][k], post[q][k], post[q][k + 1], post[j][k + 1])
            faces.append(tuple(reversed(face)) if inward else face)

    half = _h_port_half_angle(radius, branch_radius)
    band = []
    for j, theta in enumerate(theta_values):
        delta = _h_wrap_angle(theta - theta0)
        if abs(delta) <= half + 2.0e-8:
            band.append((delta, j))
    band.sort(key=lambda item: item[0])

    # Upstream edge from -half -> +half, then downstream edge back to -half.
    port = []
    if len(band) >= 2:
        for _, j in band:
            port.append(pre[j][-1])
        for _, j in reversed(band[1:-1]):
            port.append(post[j][0])

    start_ring = [pre[j][0] for j in range(ntheta)]
    end_ring = [post[j][-1] for j in range(ntheta)]
    return port, start_ring, end_ring


def _h_add_junction_transition(
    verts, faces, port_refs, centerline_point, axis, main_radius, branch_radius,
    blend_length, segments, sides, inward=False, reverse_axis_side=False,
):
    """Bridge a cylindrical saddle boundary to a round crossover tube."""
    axis = _safe_normal(axis, Vector((0.0, -1.0, 0.0)))
    direction = -axis if reverse_axis_side else axis
    abs_dy = max(abs(axis.y), 1.0e-5)
    leave_distance = main_radius / abs_dy
    segments = max(1, int(segments))

    u = Vector((0.0, 0.0, 1.0))
    u = _safe_normal(u - axis * u.dot(axis), Vector((0.0, 0.0, 1.0)))
    v = _safe_normal(axis.cross(u), Vector((1.0, 0.0, 0.0)))
    port_refs = _h_loop_project_orientation(verts, list(port_refs), centerline_point, u, v)

    # Align the saddle loop's cyclic start with the round crossover ring.
    # The saddle at the B main pipe is naturally parameterized roughly 180 deg
    # out of phase with the ring created by _h_append_ring().  Connecting the
    # two loops without correcting that phase causes long crossing triangles
    # (the visible pinwheel/star artifact).  Rotate only the loop indexing; the
    # geometry itself is unchanged and remains shared/manifold.
    if port_refs:
        best_i = 0
        best_abs_angle = float('inf')
        for i, ref in enumerate(port_refs):
            rel = Vector(verts[ref]) - centerline_point
            angle = math.atan2(rel.dot(v), rel.dot(u))
            angle = _h_wrap_angle(angle)
            score = abs(angle)
            if score < best_abs_angle:
                best_abs_angle = score
                best_i = i
        if best_i:
            port_refs = port_refs[best_i:] + port_refs[:best_i]

    prev = port_refs
    final_ring = None
    for k in range(1, segments + 1):
        frac = k / segments
        dist = leave_distance + blend_length * frac
        center = centerline_point + direction * dist
        # Use the global A->B axis winding for both ends.  At the B end the
        # surface is connected in reverse axial order below.
        ring, _, _ = _h_append_ring(verts, center, axis, branch_radius, sides)
        if k == 1:
            if reverse_axis_side:
                _h_connect_loops_resampled(faces, ring, prev, inward=inward)
            else:
                _h_connect_loops_resampled(faces, prev, ring, inward=inward)
        else:
            if reverse_axis_side:
                _connect_loops(faces, ring, prev, inward=inward)
            else:
                _connect_loops(faces, prev, ring, inward=inward)
        prev = ring
        final_ring = ring
    return final_ring


def _h_rotate_x_point(p, angle):
    if abs(angle) < 1.0e-12:
        return tuple(p)
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = p
    return (x, y * c - z * s, y * s + z * c)


def _h_rotate_x_vector(v, angle):
    return Vector(_h_rotate_x_point(tuple(v), angle))


def h_pipe_mesh_data(settings):
    od_a = max(0.001, float(settings.pipe_a_od))
    od_b = od_a if bool(settings.symmetric) else max(0.001, float(settings.pipe_b_od))
    cross_od = max(0.001, float(settings.crossover_od))
    # A saddle branch equal to or larger than the smaller main pipe reaches a
    # degenerate half-cylinder opening. Keep an effectively equal-size branch
    # just below that limit so same-diameter H-pipes remain visually identical
    # while retaining a clean manifold boundary loop.
    cross_od = min(cross_od, min(od_a, od_b) * 0.999)
    ro_a, ro_b, ro_c = od_a * 0.5, od_b * 0.5, cross_od * 0.5
    wall = max(0.0, float(settings.wall_thickness))
    wall = min(wall, min(ro_a, ro_b, ro_c) * 0.92)
    ri_a, ri_b, ri_c = ro_a - wall, ro_b - wall, ro_c - wall
    hollow = wall > 1.0e-6 and min(ri_a, ri_b, ri_c) > 1.0e-6

    length = max(0.01, float(settings.main_length))
    spacing_min = ro_a + ro_b + max(0.02 * min(od_a, od_b), 1.0e-5)
    spacing = max(spacing_min, float(settings.main_spacing))

    # Signed angle away from the normal connecting line.  Keep enough Y
    # component that saddle intersection math remains well-conditioned.
    requested_angle = max(math.radians(-65.0), min(math.radians(65.0), float(settings.crossover_angle)))
    angle = requested_angle
    sides = max(16, int(settings.profile_segments))
    main_segments = max(6, int(settings.main_segments))
    junction_segments = max(1, int(settings.junction_segments))

    # Reduce angle only if the requested saddle cannot fit inside the main-pipe
    # length.  This protects the parametric mesh from self-clipping near ends.
    xmid_requested = max(0.0, min(length, float(settings.crossover_position)))
    for _ in range(40):
        dx = math.sin(angle)
        dy = -math.cos(angle)
        abs_dy = max(abs(dy), 1.0e-5)
        delta_x = spacing * math.tan(angle)
        extent_a = (abs(dx) * ro_a + ro_c) / abs_dy
        extent_b = (abs(dx) * ro_b + ro_c) / abs_dy
        min_mid = max(extent_a + 0.5 * delta_x, extent_b - 0.5 * delta_x, 0.0)
        max_mid = min(length - extent_a + 0.5 * delta_x, length - extent_b - 0.5 * delta_x, length)
        if min_mid <= max_mid:
            break
        angle *= 0.90
    dx = math.sin(angle)
    dy = -math.cos(angle)
    abs_dy = max(abs(dy), 1.0e-5)
    delta_x = spacing * math.tan(angle)
    extent_a = (abs(dx) * ro_a + ro_c) / abs_dy
    extent_b = (abs(dx) * ro_b + ro_c) / abs_dy
    min_mid = max(extent_a + 0.5 * delta_x, extent_b - 0.5 * delta_x, 0.0)
    max_mid = min(length - extent_a + 0.5 * delta_x, length - extent_b - 0.5 * delta_x, length)
    xmid = min(max(xmid_requested, min_mid), max_mid) if min_mid <= max_mid else length * 0.5
    tap_a = xmid - 0.5 * delta_x
    tap_b = xmid + 0.5 * delta_x

    axis = _safe_normal(Vector((dx, dy, 0.0)), Vector((0.0, -1.0, 0.0)))
    center_a = Vector((tap_a, spacing * 0.5, 0.0))
    center_b = Vector((tap_b, -spacing * 0.5, 0.0))

    # Keep the blend zones from colliding in the free span between the mains.
    free_outer = max(0.0, (spacing - ro_a - ro_b) / abs_dy)
    blend = min(max(0.0, float(settings.junction_blend_length)), free_outer * 0.42)

    # Angle lists include exact outer and inner saddle endpoints, so the hole
    # boundary is represented by real shared vertices rather than crossing edges.
    def theta_values(theta0, ro, ri):
        vals = [2.0 * math.pi * j / sides for j in range(sides)]
        for rr, br in ((ro, ro_c), (ri, ri_c)):
            if rr > _EPS and br > _EPS:
                h = _h_port_half_angle(rr, br)
                vals.extend((theta0 - h, theta0 + h))
        return _h_unique_angles(vals)

    verts, faces = [], []
    ends = {}
    ports = {}

    for name, cy, station, theta0, ro, ri in (
        ('A', spacing * 0.5, tap_a, math.pi, ro_a, ri_a),
        ('B', -spacing * 0.5, tap_b, 0.0, ro_b, ri_b),
    ):
        thetas = theta_values(theta0, ro, ri)
        outer_port, outer_start, outer_end = _h_main_surface_with_port(
            verts, faces, length=length, center_y=cy, station=station, theta0=theta0,
            radius=ro, branch_radius=ro_c, axis_dx=axis.x, axis_dy=axis.y,
            theta_values=thetas, longitudinal_segments=main_segments, inward=False,
        )
        inner_port = inner_start = inner_end = None
        if hollow:
            inner_port, inner_start, inner_end = _h_main_surface_with_port(
                verts, faces, length=length, center_y=cy, station=station, theta0=theta0,
                radius=ri, branch_radius=ri_c, axis_dx=axis.x, axis_dy=axis.y,
                theta_values=thetas, longitudinal_segments=main_segments, inward=True,
            )
            n = len(outer_start)
            # Same theta list -> same ring count, so annular cut walls are direct.
            for j in range(n):
                q = (j + 1) % n
                faces.append((outer_start[j], inner_start[j], inner_start[q], outer_start[q]))
                faces.append((outer_end[j], outer_end[q], inner_end[q], inner_end[j]))
        ends[name] = (outer_start, outer_end, inner_start, inner_end)
        ports[name] = (outer_port, inner_port)

    # Outer crossover junctions and central tube.
    a_outer_neck = _h_add_junction_transition(
        verts, faces, ports['A'][0], center_a, axis, ro_a, ro_c,
        blend, junction_segments, sides, inward=False, reverse_axis_side=False,
    )
    b_outer_neck = _h_add_junction_transition(
        verts, faces, ports['B'][0], center_b, axis, ro_b, ro_c,
        blend, junction_segments, sides, inward=False, reverse_axis_side=True,
    )
    _connect_loops(faces, a_outer_neck, b_outer_neck, inward=False)

    if hollow:
        a_inner_neck = _h_add_junction_transition(
            verts, faces, ports['A'][1], center_a, axis, ri_a, ri_c,
            blend, junction_segments, sides, inward=True, reverse_axis_side=False,
        )
        b_inner_neck = _h_add_junction_transition(
            verts, faces, ports['B'][1], center_b, axis, ri_b, ri_c,
            blend, junction_segments, sides, inward=True, reverse_axis_side=True,
        )
        _connect_loops(faces, a_inner_neck, b_inner_neck, inward=True)

    # Weld split-station seams and exact saddle endpoint coincidences only.
    verts, faces = _weld_mesh_data(
        verts, faces,
        tolerance=max(1.0e-8, min(od_a, od_b, cross_od) * 1.0e-7),
        merge_min_x=None,
    )

    plane_rot = float(settings.plane_rotation)
    if abs(plane_rot) > 1.0e-12:
        verts = [_h_rotate_x_point(v, plane_rot) for v in verts]

    inlet_a = Vector((0.0, spacing * 0.5, 0.0))
    outlet_a = Vector((length, spacing * 0.5, 0.0))
    inlet_b = Vector((0.0, -spacing * 0.5, 0.0))
    outlet_b = Vector((length, -spacing * 0.5, 0.0))
    tangent = Vector((1.0, 0.0, 0.0))
    if abs(plane_rot) > 1.0e-12:
        inlet_a = Vector(_h_rotate_x_point(tuple(inlet_a), plane_rot))
        outlet_a = Vector(_h_rotate_x_point(tuple(outlet_a), plane_rot))
        inlet_b = Vector(_h_rotate_x_point(tuple(inlet_b), plane_rot))
        outlet_b = Vector(_h_rotate_x_point(tuple(outlet_b), plane_rot))
        tangent = _h_rotate_x_vector(tangent, plane_rot)

    clear_length = max(0.0, (spacing - ro_a - ro_b) / abs_dy)
    ports_meta = {
        'A_IN': (inlet_a, tangent.copy()), 'A_OUT': (outlet_a, tangent.copy()),
        'B_IN': (inlet_b, tangent.copy()), 'B_OUT': (outlet_b, tangent.copy()),
    }
    return verts, faces, spacing, tap_a, tap_b, angle, clear_length, ports_meta


def rebuild_h_pipe_object(obj):
    if not obj or obj.name in _rebuild_guard or not getattr(obj, "exhaust_h_pipe", None):
        return
    settings = obj.exhaust_h_pipe
    if not settings.is_h_pipe:
        return
    _rebuild_guard.add(obj.name)
    try:
        verts, faces, spacing, tap_a, tap_b, angle, clear_len, ports_meta = h_pipe_mesh_data(settings)
        _replace_mesh(obj, verts, faces)
        settings.computed_crossover_length = clear_len
        settings.computed_tap_a = tap_a
        settings.computed_tap_b = tap_b
        settings.computed_effective_angle = angle

        p0, t0 = ports_meta['A_IN']
        p1, t1 = ports_meta['A_OUT']
        _set_connector_metadata(obj, p0, t0, p1, t1)
        for key, (p, t) in ports_meta.items():
            obj[f"exhaust_connector_{key.lower()}"] = tuple(p)
            obj[f"exhaust_connector_{key.lower()}_tangent"] = tuple(t)
        obj["exhaust_h_pipe_single_manifold_mesh"] = True
        obj["exhaust_h_pipe_true_crossover"] = True
        obj["exhaust_h_pipe_effective_spacing"] = spacing
    finally:
        _rebuild_guard.discard(obj.name)


def reducer_mesh_data(inlet_od, outlet_od, length, wall, sides):
    sides = max(6, int(sides))
    length = max(1.0e-6, length)
    ro0 = max(1.0e-6, inlet_od * 0.5)
    ro1 = max(1.0e-6, outlet_od * 0.5)
    wall = max(0.0, wall)
    ri0 = max(0.0, ro0 - min(wall, ro0 * 0.999))
    ri1 = max(0.0, ro1 - min(wall, ro1 * 0.999))
    hollow = ri0 > 1.0e-6 and ri1 > 1.0e-6

    verts = _ring(0.0, ro0, sides) + _ring(length, ro1, sides)
    if hollow:
        verts += _ring(0.0, ri0, sides) + _ring(length, ri1, sides)
    faces = []
    for j in range(sides):
        k = (j + 1) % sides
        faces.append((j, k, sides + k, sides + j))
        if hollow:
            i0 = 2 * sides
            i1 = 3 * sides
            faces.append((i0 + j, i1 + j, i1 + k, i0 + k))
            faces.append((j, i0 + j, i0 + k, k))
            faces.append((sides + j, sides + k, i1 + k, i1 + j))
    return verts, faces


def rebuild_reducer_object(obj):
    if not obj or obj.name in _rebuild_guard or not getattr(obj, "exhaust_reducer", None):
        return
    settings = obj.exhaust_reducer
    if not settings.is_reducer:
        return
    _rebuild_guard.add(obj.name)
    try:
        verts, faces = reducer_mesh_data(settings.inlet_od, settings.outlet_od, settings.length, settings.wall_thickness, settings.profile_segments)
        _replace_mesh(obj, verts, faces)
        _set_connector_metadata(
            obj,
            Vector((0.0, 0.0, 0.0)), Vector((1.0, 0.0, 0.0)),
            Vector((settings.length, 0.0, 0.0)), Vector((1.0, 0.0, 0.0)),
        )
    finally:
        _rebuild_guard.discard(obj.name)




# -----------------------------------------------------------------------------
# Fabrication pie-cut bend
# -----------------------------------------------------------------------------

def _pie_cut_layout(settings):
    """Return centers, section axes, boundary-plane normals and sizing data.

    The bend is made from straight cylindrical sections.  Adjacent section axes
    differ by a constant weld angle, and each shared miter plane bisects those
    axes.  This is actual miter geometry rather than a faceted approximation of
    a torus.
    """
    sections = max(2, int(settings.pie_sections))
    total = max(math.radians(1.0), abs(float(settings.bend_angle)))
    signed_total = total if settings.bend_angle >= 0.0 else -total
    delta = signed_total / (sections - 1)
    radius = max(1.0e-6, float(settings.equivalent_clr))
    section_length = 2.0 * radius * math.sin(abs(delta) * 0.5)
    section_length = max(section_length, 1.0e-6)

    start_t = Vector((1.0, 0.0, 0.0))
    base_up = Vector((0.0, 0.0, 1.0))
    bend_dir = _safe_normal(_rotation(start_t, settings.clocking) @ base_up, base_up)
    sign = 1.0 if signed_total >= 0.0 else -1.0
    signed_dir = bend_dir * sign
    turn_axis = _safe_normal(start_t.cross(signed_dir), Vector((0.0, -1.0, 0.0)))

    axes = []
    for i in range(sections):
        a = abs(delta) * i
        axes.append(_safe_normal(_rotation(turn_axis, a) @ start_t, start_t))

    centers = [Vector((0.0, 0.0, 0.0))]
    for d in axes:
        centers.append(centers[-1] + d * section_length)

    planes = [axes[0].copy()]
    for i in range(1, sections):
        planes.append(_safe_normal(axes[i - 1] + axes[i], axes[i - 1]))
    planes.append(axes[-1].copy())
    return centers, axes, planes, turn_axis, delta, section_length


def _pie_seam_loop(center, cylinder_axis, plane_normal, turn_axis, radius, sides):
    """Intersection loop of a circular cylinder and a miter plane.

    The loop is an ellipse in 3D.  Using the constant turn axis as angular zero
    keeps the ordering consistent across both cylinders sharing an interior seam.
    """
    d = _safe_normal(cylinder_axis, Vector((1.0, 0.0, 0.0)))
    n = _safe_normal(plane_normal, d)
    u = _safe_normal(turn_axis - d * turn_axis.dot(d), Vector((0.0, 0.0, 1.0)))
    v = _safe_normal(d.cross(u), Vector((0.0, 1.0, 0.0)))
    denom = n.dot(d)
    if abs(denom) < 1.0e-6:
        denom = 1.0e-6 if denom >= 0.0 else -1.0e-6
    loop = []
    for j in range(max(12, int(sides))):
        a = 2.0 * math.pi * j / max(12, int(sides))
        radial_unit = u * math.cos(a) + v * math.sin(a)
        radial = radial_unit * radius
        shift = -n.dot(radial) / denom
        loop.append(center + d * shift + radial)
    return loop


def pie_cut_seam_loops(settings, guide_offset=0.0):
    """Outer miter seam loops for viewport-only weld guides.

    Returns only the internal weld seams, not the two free end cuts.
    """
    centers, axes, planes, turn_axis, _delta, _length = _pie_cut_layout(settings)
    sides = max(12, int(settings.profile_segments))
    ro = max(1.0e-6, settings.outside_diameter * 0.5 + max(0.0, guide_offset))
    loops = []
    for boundary in range(1, len(centers) - 1):
        # Either adjacent cylinder produces the same physical miter ellipse.
        loops.append(_pie_seam_loop(centers[boundary], axes[boundary - 1], planes[boundary], turn_axis, ro, sides))
    return loops


def pie_cut_mesh_data(settings):
    centers, axes, planes, turn_axis, delta, section_length = _pie_cut_layout(settings)
    sides = max(12, int(settings.profile_segments))
    ro = max(1.0e-6, settings.outside_diameter * 0.5)
    wall = max(0.0, min(settings.wall_thickness, ro * 0.999))
    ri = max(0.0, ro - wall)
    hollow = ri > 1.0e-6

    outer_loops = []
    inner_loops = []
    boundary_count = len(centers)
    for i in range(boundary_count):
        # Interior miter boundary is shared by its two neighboring cylinders.
        # Use the upstream section to define the loop; the set lies on both.
        axis = axes[0] if i == 0 else axes[-1] if i == boundary_count - 1 else axes[i - 1]
        outer_loops.append(_pie_seam_loop(centers[i], axis, planes[i], turn_axis, ro, sides))
        if hollow:
            inner_loops.append(_pie_seam_loop(centers[i], axis, planes[i], turn_axis, ri, sides))

    verts = []
    outer_ids = []
    inner_ids = []
    for loop in outer_loops:
        ids = []
        for p in loop:
            ids.append(len(verts)); verts.append(tuple(p))
        outer_ids.append(ids)
    if hollow:
        for loop in inner_loops:
            ids = []
            for p in loop:
                ids.append(len(verts)); verts.append(tuple(p))
            inner_ids.append(ids)

    faces = []
    for i in range(boundary_count - 1):
        oa, ob = outer_ids[i], outer_ids[i + 1]
        for j in range(sides):
            k = (j + 1) % sides
            faces.append((oa[j], oa[k], ob[k], ob[j]))
        if hollow:
            ia, ib = inner_ids[i], inner_ids[i + 1]
            for j in range(sides):
                k = (j + 1) % sides
                faces.append((ia[j], ib[j], ib[k], ia[k]))

    if hollow:
        # Square free ends; interior miter seams remain shared vertices.
        for boundary, reverse in ((0, True), (boundary_count - 1, False)):
            o, inn = outer_ids[boundary], inner_ids[boundary]
            for j in range(sides):
                k = (j + 1) % sides
                if reverse:
                    faces.append((o[j], inn[j], inn[k], o[k]))
                else:
                    faces.append((o[j], o[k], inn[k], inn[j]))

    return verts, faces, centers, axes, delta, section_length


def rebuild_pie_cut_object(obj):
    if not obj or obj.name in _rebuild_guard or not getattr(obj, "exhaust_pie_cut", None):
        return
    settings = obj.exhaust_pie_cut
    if not settings.is_pie_cut:
        return
    _rebuild_guard.add(obj.name)
    try:
        verts, faces, centers, axes, delta, section_length = pie_cut_mesh_data(settings)
        _replace_mesh(obj, verts, faces)
        settings.computed_weld_angle = abs(delta)
        settings.computed_section_length = section_length
        settings.computed_centerline_length = section_length * max(2, int(settings.pie_sections))
        _set_connector_metadata(obj, centers[0], axes[0], centers[-1], axes[-1])
        obj["exhaust_part_type"] = "PIE_CUT_BEND"
    finally:
        _rebuild_guard.discard(obj.name)


def rebuild_any(obj):
    if not obj:
        return
    if getattr(obj, "exhaust_route", None) and obj.exhaust_route.is_route:
        rebuild_route_object(obj)
    elif getattr(obj, "exhaust_collector", None) and obj.exhaust_collector.is_collector:
        rebuild_collector_object(obj)
    elif getattr(obj, "exhaust_y_pipe", None) and obj.exhaust_y_pipe.is_y_pipe:
        rebuild_y_pipe_object(obj)
    elif getattr(obj, "exhaust_x_pipe", None) and obj.exhaust_x_pipe.is_x_pipe:
        rebuild_x_pipe_object(obj)
    elif getattr(obj, "exhaust_h_pipe", None) and obj.exhaust_h_pipe.is_h_pipe:
        rebuild_h_pipe_object(obj)
    elif getattr(obj, "exhaust_pie_cut", None) and obj.exhaust_pie_cut.is_pie_cut:
        rebuild_pie_cut_object(obj)
    elif getattr(obj, "exhaust_reducer", None) and obj.exhaust_reducer.is_reducer:
        rebuild_reducer_object(obj)
