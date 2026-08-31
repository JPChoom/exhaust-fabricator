"""Conservative tube-envelope collision helpers for Header primary routing.

v0.14.0 replaces the earlier single-radius sampled-polyline test with a
per-segment capsule chain.  Straights are represented exactly.  Circular
mandrel bends are represented by chord capsules whose radii are inflated only
by that chord's arc sagitta, so the union conservatively contains the true
circular tube sweep instead of cutting inside it.
"""
import math
from mathutils import Vector

_EPS = 1.0e-12


def segment_segment_distance(p1, q1, p2, q2):
    """Shortest distance between two finite 3D line segments."""
    p1 = Vector(p1); q1 = Vector(q1); p2 = Vector(p2); q2 = Vector(q2)
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(d1.dot(d1))
    e = float(d2.dot(d2))
    f = float(d2.dot(r))

    if a <= _EPS and e <= _EPS:
        return (p1 - p2).length
    if a <= _EPS:
        s = 0.0
        t = max(0.0, min(1.0, f / e if e > _EPS else 0.0))
    else:
        c = float(d1.dot(r))
        if e <= _EPS:
            t = 0.0
            s = max(0.0, min(1.0, -c / a))
        else:
            b = float(d1.dot(d2))
            denom = a * e - b * b
            if abs(denom) > _EPS:
                s = max(0.0, min(1.0, (b * f - c * e) / denom))
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t = 1.0
                s = max(0.0, min(1.0, (b - c) / a))

    c1 = p1 + d1 * s
    c2 = p2 + d2 * t
    return (c1 - c2).length


def polyline_min_distance(points_a, points_b, early_below=None):
    """Legacy polyline distance helper retained for compatibility."""
    if not points_a or not points_b or len(points_a) < 2 or len(points_b) < 2:
        return float('inf')
    best = float('inf')
    threshold = float(early_below) if early_below is not None else None
    for i in range(len(points_a) - 1):
        a0, a1 = points_a[i], points_a[i + 1]
        for j in range(len(points_b) - 1):
            d = segment_segment_distance(a0, a1, points_b[j], points_b[j + 1])
            if d < best:
                best = d
                if threshold is not None and best < threshold:
                    return best
    return best


def capsule_clearance(points_a, radius_a, points_b, radius_b, extra_clearance=0.0):
    """Legacy uniform-radius polyline clearance; negative means overlap."""
    required = max(0.0, float(radius_a)) + max(0.0, float(radius_b)) + max(0.0, float(extra_clearance))
    d = polyline_min_distance(points_a, points_b, early_below=required)
    return d - required


def _matrix_max_scale(matrix):
    """Conservative radius scale for an arbitrary affine Matrix."""
    try:
        m = matrix.to_3x3()
        return max((m @ Vector((1.0, 0.0, 0.0))).length,
                   (m @ Vector((0.0, 1.0, 0.0))).length,
                   (m @ Vector((0.0, 0.0, 1.0))).length,
                   1.0e-12)
    except Exception:
        return 1.0


def transform_capsule_chain(chain, matrix):
    """Transform [(p0,p1,r), ...], conservatively scaling capsule radii."""
    scale = _matrix_max_scale(matrix)
    return [(matrix @ Vector(a), matrix @ Vector(b), max(0.0, float(r)) * scale)
            for a, b, r in (chain or [])]


def _segment_bounds(p0, p1, radius, pad_extra=0.0):
    """Axis-aligned bounds of one capsule segment, inflated by radius + pad_extra."""
    pad = max(0.0, float(radius)) + max(0.0, float(pad_extra))
    mn = Vector((min(p0[0], p1[0]) - pad, min(p0[1], p1[1]) - pad, min(p0[2], p1[2]) - pad))
    mx = Vector((max(p0[0], p1[0]) + pad, max(p0[1], p1[1]) + pad, max(p0[2], p1[2]) + pad))
    return mn, mx


def capsule_chain_bounds(chain, extra=0.0):
    """Precompute per-segment + overall bounds for a capsule chain, once.

    Returns `((overall_min, overall_max), [(p0, p1, r, seg_min, seg_max), ...])`,
    or None for an empty chain.  `extra` is baked into every segment's pad here;
    `capsule_chain_clearance` always bakes the *other* chain's pad as zero, so
    the two sides sum to exactly the requested clearance -- see its docstring.
    Pass the result back in as `chain_b_bounds` to avoid rebuilding these
    bounds every time the same fixed chain (e.g. a sibling-primary obstacle)
    is queried against many different candidates.
    """
    items = []
    overall_min = Vector((float('inf'),) * 3)
    overall_max = Vector((float('-inf'),) * 3)
    for p0, p1, r in (chain or []):
        p0 = Vector(p0); p1 = Vector(p1)
        mn, mx = _segment_bounds(p0, p1, r, extra)
        items.append((p0, p1, max(0.0, float(r)), mn, mx))
        overall_min.x = min(overall_min.x, mn.x); overall_min.y = min(overall_min.y, mn.y); overall_min.z = min(overall_min.z, mn.z)
        overall_max.x = max(overall_max.x, mx.x); overall_max.y = max(overall_max.y, mx.y); overall_max.z = max(overall_max.z, mx.z)
    if not items:
        return None
    return (overall_min, overall_max), items


def capsule_chain_clearance(chain_a, chain_b, extra_clearance=0.0, early_exit=True, chain_b_bounds=None):
    """Minimum surface clearance between two variable-radius capsule chains.

    Negative means the represented tube envelopes overlap.  The extra clearance
    is added once between the two physical surfaces.

    A cheap bounding-box broad phase runs first at the whole-chain level, then
    per segment pair, before falling back to the exact segment/segment distance
    test -- most candidate/obstacle pairs during a blind search are nowhere
    near each other, so this skips the expensive math for them entirely.

    Pass `chain_b_bounds` (from `capsule_chain_bounds(chain_b, extra=0.0)`)
    when the same chain_b is queried repeatedly against many different chain_a
    values -- e.g. a fixed sibling-primary obstacle checked against thousands
    of solver candidates -- so its bounds are built once instead of on every
    call.

    Safety contract (the part callers actually rely on): a pair is only ever
    skipped by the broad phase once its own inflated bounding boxes are proven
    disjoint, which is only possible when that pair's true clearance is
    positive -- so a real overlap can never be pruned away, and the sign of
    the result (collision vs. clear) always matches the un-pruned brute-force
    answer.  What is NOT preserved: the exact magnitude of a comfortably-clear
    result.  When nothing overlaps this may return `float('inf')` instead of
    the true (positive) distance, and even when the loop runs to completion,
    the reported "closest" safe pair can be a conservative overestimate if the
    single truly-closest pair happened to get bbox-pruned in favor of a
    different, also-safe one.  Both are one-directional: the reported value
    can only be as-or-more optimistic than the truth, never less, so a
    negative/colliding answer is always exact and a "no collision" answer is
    always correct even when its margin is approximate.
    """
    if not chain_a or not chain_b:
        return float('inf')
    extra = max(0.0, float(extra_clearance))

    bounds_a = capsule_chain_bounds(chain_a, extra)
    bounds_b = chain_b_bounds if chain_b_bounds is not None else capsule_chain_bounds(chain_b, 0.0)
    if bounds_a is None or bounds_b is None:
        return float('inf')
    (a_mn, a_mx), a_items = bounds_a
    (b_mn, b_mx), b_items = bounds_b
    if not _aabb_overlap(a_mn, a_mx, b_mn, b_mx):
        return float('inf')

    best = float('inf')
    for a0, a1, ar, amn, amx in a_items:
        for b0, b1, br, bmn, bmx in b_items:
            if not _aabb_overlap(amn, amx, bmn, bmx):
                continue
            d = segment_segment_distance(a0, a1, b0, b1)
            clr = d - (ar + br + extra)
            if clr < best:
                best = clr
                if early_exit and best < -1.0e-10:
                    return best
    return best


def route_local_capsule_chain(settings, include_finish_extension=True):
    """Build a conservative local-space capsule chain for one Route.

    Mandrel bend chord radii include their local arc sagitta, guaranteeing that
    the chain contains the true circular centerline sweep between samples.
    Pie-cut bends are already straight centerline sections and need no sagitta.
    """
    from .geometry import (
        _segment_bend_basis, _route_pie_layout, _rotation, _safe_normal,
        route_end_treatment_spec, route_connection_hardware_spec,
    )

    tube_r = max(0.0, float(settings.outside_diameter) * 0.5)
    p = Vector((0.0, 0.0, 0.0))
    tangent = Vector((1.0, 0.0, 0.0))
    up = Vector((0.0, 0.0, 1.0))
    chain = []

    for seg in settings.segments:
        if seg.kind == 'STRAIGHT':
            L = max(0.0, float(seg.length))
            if L > 1.0e-10:
                q = p + tangent * L
                chain.append((p.copy(), q.copy(), tube_r))
                p = q
            continue

        angle = abs(float(seg.angle))
        if angle < 1.0e-10:
            continue

        if getattr(seg, 'bend_style', 'MANDREL') == 'PIE_CUT':
            centers, _axes, _planes, _axis, tangent, up, _delta, _section_length = _route_pie_layout(p, tangent, up, seg)
            for a, b in zip(centers[:-1], centers[1:]):
                if (b - a).length > 1.0e-10:
                    chain.append((a.copy(), b.copy(), tube_r))
            p = centers[-1].copy()
            continue

        R = max(1.0e-6, float(seg.radius))
        bend_dir, signed_dir, axis = _segment_bend_basis(tangent, up, seg)
        center = p + signed_dir * R
        radial0 = p - center
        steps = max(2, int(getattr(seg, 'resolution', 16)))
        step_angle = angle / steps
        sagitta = R * (1.0 - math.cos(step_angle * 0.5))
        chord_r = tube_r + sagitta + 1.0e-10
        prev = p.copy()
        for i in range(1, steps + 1):
            rot = _rotation(axis, angle * (i / steps))
            q = center + (rot @ radial0)
            chain.append((prev.copy(), q.copy(), chord_r))
            prev = q
        rot_end = _rotation(axis, angle)
        tangent = _safe_normal(rot_end @ tangent)
        up = _safe_normal(rot_end @ bend_dir)
        up = _safe_normal(up - tangent * up.dot(tangent), Vector((0.0, 0.0, 1.0)))
        p = prev.copy()

    if include_finish_extension:
        try:
            end_spec = route_end_treatment_spec(settings, 'end')
            end_hw = route_connection_hardware_spec(settings, 'end', end_spec)
            extension = max(0.0, float(end_spec.get('extension', 0.0) + end_hw.get('extension', 0.0)))
            if extension > 1.0e-10:
                q = p + tangent * extension
                # Finish hardware can be wider than the tube.  Conservatively
                # account for its maximum exposed connector envelope only over
                # the finish extension, not along the whole primary.
                ext_r = max(tube_r,
                            max(0.0, float(end_spec.get('od', 0.0))) * 0.5,
                            max(0.0, float(end_hw.get('connector_od', 0.0))) * 0.5)
                chain.append((p.copy(), q.copy(), ext_r))
                p = q
        except Exception:
            pass

    return chain


def route_world_capsule_chain(route_obj, include_finish_extension=True):
    chain = route_local_capsule_chain(route_obj.exhaust_route, include_finish_extension)
    return transform_capsule_chain(chain, route_obj.matrix_world)


def route_world_polyline(route_obj, include_finish_extension=True):
    """Compatibility centerline polyline used by non-collision UI helpers."""
    from .geometry import route_points, route_seam_frames, route_end_treatment_spec, route_connection_hardware_spec
    pts = [route_obj.matrix_world @ p for p in route_points(route_obj.exhaust_route)]
    if include_finish_extension:
        try:
            end_spec = route_end_treatment_spec(route_obj.exhaust_route, 'end')
            end_hw = route_connection_hardware_spec(route_obj.exhaust_route, 'end', end_spec)
            extension = float(end_spec.get('extension', 0.0) + end_hw.get('extension', 0.0))
            if extension > 1.0e-9:
                frames = route_seam_frames(route_obj.exhaust_route, include_end=True)
                if frames:
                    p, t = frames[-1][0], frames[-1][1]
                    pts.append(route_obj.matrix_world @ (p + t * extension))
        except Exception:
            pass
    return pts


def transform_polyline(points, matrix):
    return [matrix @ Vector(p) for p in points]


def primary_collision_pairs(routes, extra_clearance=0.0):
    """Return (index_a, index_b, clearance) for violating primary pairs."""
    routes = list(routes or [])
    pairs = []
    cache = {}
    for r in routes:
        try:
            cache[r.name] = route_world_capsule_chain(r, include_finish_extension=True)
        except Exception:
            cache[r.name] = []
    for i in range(len(routes)):
        a = routes[i]
        if not getattr(a, 'exhaust_route', None):
            continue
        for j in range(i + 1, len(routes)):
            b = routes[j]
            if not getattr(b, 'exhaust_route', None):
                continue
            clr = capsule_chain_clearance(cache.get(a.name, []), cache.get(b.name, []), float(extra_clearance))
            if clr < -1.0e-8:
                pairs.append((i, j, float(clr)))
    return pairs

# ---------------------------------------------------------------------------
# Arbitrary mesh keep-out collision helpers (v0.14.0)
# ---------------------------------------------------------------------------

def _aabb_overlap(amin, amax, bmin, bmax):
    return not (amax.x < bmin.x or amin.x > bmax.x or
                amax.y < bmin.y or amin.y > bmax.y or
                amax.z < bmin.z or amin.z > bmax.z)


def _closest_point_triangle(p, a, b, c):
    """Closest point on triangle ABC to point P (Ericson region test)."""
    p = Vector(p); a = Vector(a); b = Vector(b); c = Vector(c)
    ab = b - a; ac = c - a; ap = p - a
    d1 = ab.dot(ap); d2 = ac.dot(ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return a.copy()
    bp = p - b
    d3 = ab.dot(bp); d4 = ac.dot(bp)
    if d3 >= 0.0 and d4 <= d3:
        return b.copy()
    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / max(_EPS, d1 - d3)
        return a + ab * v
    cp = p - c
    d5 = ab.dot(cp); d6 = ac.dot(cp)
    if d6 >= 0.0 and d5 <= d6:
        return c.copy()
    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / max(_EPS, d2 - d6)
        return a + ac * w
    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        w = (d4 - d3) / max(_EPS, (d4 - d3) + (d5 - d6))
        return b + (c - b) * w
    denom = max(_EPS, va + vb + vc)
    v = vb / denom
    w = vc / denom
    return a + ab * v + ac * w


def _segment_triangle_intersection_t(p0, p1, a, b, c):
    """Moller-Trumbore segment/triangle intersection; returns t in [0,1]."""
    p0 = Vector(p0); p1 = Vector(p1); a = Vector(a); b = Vector(b); c = Vector(c)
    d = p1 - p0
    e1 = b - a
    e2 = c - a
    h = d.cross(e2)
    det = e1.dot(h)
    if abs(det) <= 1.0e-11:
        return None
    inv = 1.0 / det
    s = p0 - a
    u = inv * s.dot(h)
    if u < -1.0e-10 or u > 1.0 + 1.0e-10:
        return None
    q = s.cross(e1)
    v = inv * d.dot(q)
    if v < -1.0e-10 or u + v > 1.0 + 1.0e-10:
        return None
    t = inv * e2.dot(q)
    if t < -1.0e-10 or t > 1.0 + 1.0e-10:
        return None
    return max(0.0, min(1.0, float(t)))


def _closest_point_triangle_precomp(p, a, b, c, ab, ac):
    """Same algorithm as `_closest_point_triangle`, given precomputed edge
    vectors `ab = b - a`, `ac = c - a` -- for callers (like
    `segment_triangle_distance`) testing more than one point against the same
    fixed triangle, so the edge subtractions aren't redone for every point."""
    ap = p - a
    d1 = ab.dot(ap); d2 = ac.dot(ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return a.copy()
    bp = p - b
    d3 = ab.dot(bp); d4 = ac.dot(bp)
    if d3 >= 0.0 and d4 <= d3:
        return b.copy()
    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / max(_EPS, d1 - d3)
        return a + ab * v
    cp = p - c
    d5 = ab.dot(cp); d6 = ac.dot(cp)
    if d6 >= 0.0 and d5 <= d6:
        return c.copy()
    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / max(_EPS, d2 - d6)
        return a + ac * w
    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        w = (d4 - d3) / max(_EPS, (d4 - d3) + (d5 - d6))
        return b + (c - b) * w
    denom = max(_EPS, va + vb + vc)
    v = vb / denom
    w = vc / denom
    return a + ab * v + ac * w


def _segment_triangle_intersection_t_precomp(p0, d, a, ab, ac):
    """Same algorithm as `_segment_triangle_intersection_t`, given the
    segment direction `d = p1 - p0` and triangle edges `ab = b - a`,
    `ac = c - a` already computed by the caller."""
    h = d.cross(ac)
    det = ab.dot(h)
    if abs(det) <= 1.0e-11:
        return None
    inv = 1.0 / det
    s = p0 - a
    u = inv * s.dot(h)
    if u < -1.0e-10 or u > 1.0 + 1.0e-10:
        return None
    q = s.cross(ab)
    v = inv * d.dot(q)
    if v < -1.0e-10 or u + v > 1.0 + 1.0e-10:
        return None
    t = inv * ac.dot(q)
    if t < -1.0e-10 or t > 1.0 + 1.0e-10:
        return None
    return max(0.0, min(1.0, float(t)))


def _segment_segment_distance_precomp(p1, d1, a_coef, p2, q2):
    """Same algorithm as `segment_segment_distance(p1, p1+d1, p2, q2)`, given
    the first segment's direction `d1` and `a_coef = d1.dot(d1)` already
    computed -- `segment_triangle_distance` calls this three times (once per
    triangle edge) against the SAME first segment, so neither needs redoing
    per edge."""
    d2 = q2 - p2
    r = p1 - p2
    a = a_coef
    e = float(d2.dot(d2))
    f = float(d2.dot(r))

    if a <= _EPS and e <= _EPS:
        return (p1 - p2).length
    if a <= _EPS:
        s = 0.0
        t = max(0.0, min(1.0, f / e if e > _EPS else 0.0))
    else:
        c = float(d1.dot(r))
        if e <= _EPS:
            t = 0.0
            s = max(0.0, min(1.0, -c / a))
        else:
            b = float(d1.dot(d2))
            denom = a * e - b * b
            if abs(denom) > _EPS:
                s = max(0.0, min(1.0, (b * f - c * e) / denom))
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t = 1.0
                s = max(0.0, min(1.0, (b - c) / a))

    c1 = p1 + d1 * s
    c2 = p2 + d2 * t
    return (c1 - c2).length


def segment_triangle_distance(p0, p1, a, b, c, early_exit_below=None):
    """Exact minimum distance between a finite segment and a triangle.

    Shares the segment direction and triangle edge vectors across all six
    internal sub-tests (one plane intersection test, two closest-point-on-
    triangle solves for the endpoints, three edge-distance checks) instead of
    each recomputing them independently -- same algorithm, same results, just
    without redoing identical Vector subtractions/dot products several times
    per call. Verified against the original (unshared) implementation across
    thousands of random and degenerate cases; see test_segment_triangle_distance.py.

    `early_exit_below`, if given, lets this stop as soon as ANY sub-test
    proves the true distance is at most that value, skipping the rest -- safe
    because every sub-test only ever finds a distance >= the true minimum, so
    one witness of "<=  threshold" already proves the true minimum is too,
    without needing to also find the single worst (most precise) witness.
    This only changes the return value when a caller opts in by passing a
    threshold (typically the collision-radius requirement, from a caller that
    already only cares whether the true distance clears that requirement, not
    its exact value once it doesn't) -- with the default of None, behavior is
    identical to computing all six sub-tests, as before.
    """
    p0 = Vector(p0); p1 = Vector(p1)
    a = Vector(a); b = Vector(b); c = Vector(c)
    d = p1 - p0
    ab = b - a
    ac = c - a

    if _segment_triangle_intersection_t_precomp(p0, d, a, ab, ac) is not None:
        return 0.0

    best = (p0 - _closest_point_triangle_precomp(p0, a, b, c, ab, ac)).length
    if early_exit_below is not None and best <= early_exit_below:
        return best
    d1v = (p1 - _closest_point_triangle_precomp(p1, a, b, c, ab, ac)).length
    if d1v < best:
        best = d1v
    if early_exit_below is not None and best <= early_exit_below:
        return best

    a_coef = float(d.dot(d))
    for edge_p, edge_q in ((a, b), (b, c), (c, a)):
        dseg = _segment_segment_distance_precomp(p0, d, a_coef, edge_p, edge_q)
        if dseg < best:
            best = dseg
        if early_exit_below is not None and best <= early_exit_below:
            return best
    return float(best)


def _grid_key(v, cell):
    return (math.floor(v.x / cell), math.floor(v.y / cell), math.floor(v.z / cell))


def mesh_obstacle_from_object(obj, world_to_space, cell_size=0.0508, depsgraph=None):
    """Evaluate a Blender mesh and build a triangle spatial hash in target space.

    `world_to_space` normally converts world coordinates to the Route-local
    coordinate system used by the routing solver.  Using an identity matrix
    produces a world-space obstacle suitable for post-build verification.
    """
    if obj is None or getattr(obj, 'type', None) != 'MESH':
        return None
    try:
        import bpy
        if depsgraph is None:
            depsgraph = bpy.context.evaluated_depsgraph_get()
        oe = obj.evaluated_get(depsgraph)
        me = oe.to_mesh()
        if me is None:
            return None
        try:
            me.calc_loop_triangles()
            xform = world_to_space @ oe.matrix_world
            verts = [xform @ v.co for v in me.vertices]
            tris = []
            bb_min = Vector((float('inf'),) * 3)
            bb_max = Vector((float('-inf'),) * 3)
            for lt in me.loop_triangles:
                a, b, c = (verts[i].copy() for i in lt.vertices)
                tmin = Vector((min(a.x,b.x,c.x), min(a.y,b.y,c.y), min(a.z,b.z,c.z)))
                tmax = Vector((max(a.x,b.x,c.x), max(a.y,b.y,c.y), max(a.z,b.z,c.z)))
                bb_min.x = min(bb_min.x, tmin.x); bb_min.y = min(bb_min.y, tmin.y); bb_min.z = min(bb_min.z, tmin.z)
                bb_max.x = max(bb_max.x, tmax.x); bb_max.y = max(bb_max.y, tmax.y); bb_max.z = max(bb_max.z, tmax.z)
                tris.append((a,b,c,tmin,tmax))
        finally:
            oe.to_mesh_clear()
    except Exception:
        return None
    if not tris:
        return None

    cell = max(1.0e-4, float(cell_size))
    grid = {}
    large = []
    max_cells_per_tri = 2048
    for idx, (_a,_b,_c,tmin,tmax) in enumerate(tris):
        k0 = _grid_key(tmin, cell); k1 = _grid_key(tmax, cell)
        nx = k1[0]-k0[0]+1; ny = k1[1]-k0[1]+1; nz = k1[2]-k0[2]+1
        if nx * ny * nz > max_cells_per_tri:
            large.append(idx)
            continue
        for ix in range(k0[0], k1[0]+1):
            for iy in range(k0[1], k1[1]+1):
                for iz in range(k0[2], k1[2]+1):
                    grid.setdefault((ix,iy,iz), []).append(idx)
    return {
        'name': obj.name,
        'triangles': tris,
        'grid': grid,
        'large_triangles': large,
        'cell_size': cell,
        'bbox_min': bb_min,
        'bbox_max': bb_max,
    }


def _mesh_query_indices(obstacle, qmin, qmax):
    if obstacle is None:
        return []
    if not _aabb_overlap(qmin, qmax, obstacle['bbox_min'], obstacle['bbox_max']):
        return []
    cell = obstacle['cell_size']
    k0 = _grid_key(qmin, cell); k1 = _grid_key(qmax, cell)
    found = set(obstacle.get('large_triangles', []))
    grid = obstacle.get('grid', {})
    for ix in range(k0[0], k1[0]+1):
        for iy in range(k0[1], k1[1]+1):
            for iz in range(k0[2], k1[2]+1):
                found.update(grid.get((ix,iy,iz), ()))
    return found


def _ray_cell_indices(obstacle, p, q):
    """Union of triangle indices registered in every grid cell segment p->q passes through.

    Conservative supercover walk (step size = half the cell size, so no cell
    the ray passes through can be skipped) rather than a precise DDA, since we
    only need the complete candidate SET for an exact intersection test
    afterward, not an ordered/early-exiting traversal.

    Correctness: `mesh_obstacle_from_object` registers each triangle in every
    grid cell its own bounding box overlaps.  If the ray truly intersects a
    triangle, that intersection point lies both on the ray and inside the
    triangle's bounding box, so the cell containing that point has the
    triangle registered -- and this walk visits every cell along the ray, so
    it is visited too.  No true intersection can be pruned away; only
    triangles whose cells the ray never enters are skipped.
    """
    cell = obstacle['cell_size']
    d = q - p
    length = d.length
    found = set(obstacle.get('large_triangles', ()))
    grid = obstacle.get('grid', {})
    if length < 1.0e-12:
        found.update(grid.get(_grid_key(p, cell), ()))
        return found
    step_len = max(cell * 0.5, 1.0e-6)
    steps = max(1, int(math.ceil(length / step_len)))
    dirn = d / length
    visited_keys = set()
    for i in range(steps + 1):
        pt = p + dirn * min(length, i * step_len)
        visited_keys.add(_grid_key(pt, cell))
    for key in visited_keys:
        found.update(grid.get(key, ()))
    return found


def _point_inside_triangle_mesh(point, obstacle):
    """Best-effort closed-mesh inside test used for deep solid keep-outs."""
    p = Vector(point)
    bmin = obstacle['bbox_min']; bmax = obstacle['bbox_max']
    if not (bmin.x <= p.x <= bmax.x and bmin.y <= p.y <= bmax.y and bmin.z <= p.z <= bmax.z):
        return False
    # A deliberately skewed ray reduces vertex/edge degeneracies.  Count unique
    # intersection parameters because adjacent triangles can share one hit.
    # The ray only needs to clear this mesh's own bounding box (the bbox
    # diagonal is the farthest any interior point could be from an exit,
    # regardless of direction); once a straight ray leaves a bounded region
    # moving away from it, it cannot re-enter, so a modest margin past the
    # diagonal is exactly as correct as a much longer ray, just far cheaper
    # for the grid walk below (whose cost scales with ray length, unlike the
    # brute-force triangle scan this replaces).
    direction = Vector((1.0, 0.3713907, 0.218117)).normalized()
    span = max(1.0e-3, (bmax - bmin).length)
    q = p + direction * (span * 1.5)
    tris = obstacle['triangles']
    hits = []
    for idx in _ray_cell_indices(obstacle, p, q):
        a, b, c, _mn, _mx = tris[idx]
        t = _segment_triangle_intersection_t(p, q, a, b, c)
        if t is not None and t > 1.0e-9:
            hits.append(t)
    if not hits:
        return False
    hits.sort()
    unique = []
    for t in hits:
        if not unique or abs(t - unique[-1]) > 1.0e-7:
            unique.append(t)
    return bool(len(unique) % 2)


def capsule_chain_mesh_clearance(chain, obstacle, extra_clearance=0.0, early_exit=True):
    """Surface clearance between a variable-radius capsule chain and mesh.

    Negative means the physical tube envelope plus requested keep-out clearance
    intersects the evaluated mesh.  Segment/triangle distance is exact after a
    spatial-hash broad phase; mandrel bends are already conservatively inflated
    by the caller's sagitta-aware capsule chain.
    """
    if not chain or not obstacle:
        return float('inf')
    extra = max(0.0, float(extra_clearance))
    best = float('inf')
    tris = obstacle['triangles']
    for p0, p1, radius in chain:
        p0 = Vector(p0); p1 = Vector(p1)
        required = max(0.0, float(radius)) + extra
        mn = Vector((min(p0.x,p1.x)-required, min(p0.y,p1.y)-required, min(p0.z,p1.z)-required))
        mx = Vector((max(p0.x,p1.x)+required, max(p0.y,p1.y)+required, max(p0.z,p1.z)+required))
        indices = _mesh_query_indices(obstacle, mn, mx)
        if not indices:
            mid = (p0 + p1) * 0.5
            if _point_inside_triangle_mesh(mid, obstacle):
                return -required if required > 0.0 else -1.0e-9
            continue
        seg_best = float('inf')
        # Once ANY triangle proves this segment collides (d <= required), the
        # exact worst-case witness no longer matters -- only the sign does,
        # matching this function's own early_exit contract -- so let each
        # segment_triangle_distance call stop as soon as it can prove that,
        # rather than always computing its full six-way exact minimum.
        # Must match the outer rejection tolerance exactly (`clr < -1e-10`
        # below), not just `d <= required`: an early-exit witness is only
        # guaranteed to preserve the OUTER pass/fail decision when it proves
        # the true minimum is BELOW that same strict threshold, since any
        # single witness only proves "true minimum <= this witness", not
        # equality. A witness that merely satisfies d <= required (without
        # the -1e-10 margin) could report a hair's-width-safe value while a
        # still-unexamined triangle holds the true (clearly negative) minimum.
        dist_threshold = (required - 1.0e-10) if early_exit else None
        for idx in indices:
            a,b,c,tmin,tmax = tris[idx]
            # Cheap triangle AABB check before exact segment/triangle distance.
            if not _aabb_overlap(mn, mx, tmin, tmax):
                continue
            d = segment_triangle_distance(p0, p1, a, b, c, early_exit_below=dist_threshold)
            clr = d - required
            if clr < seg_best:
                seg_best = clr
            if clr < best:
                best = clr
                if early_exit and clr < -1.0e-10:
                    return float(clr)
        # A closed keep-out can completely contain a centerline segment while
        # its surface remains farther away than the tube radius.  In that case
        # surface-distance tests alone would look clear, so use a parity test on
        # the segment midpoint as the solid-volume fallback.
        mid = (p0 + p1) * 0.5
        if seg_best >= -1.0e-10 and _point_inside_triangle_mesh(mid, obstacle):
            return -required if required > 0.0 else -1.0e-9
    return float(best)


def route_keepout_collisions(routes, keepout_objs, extra_clearance=0.0, depsgraph=None):
    """Return (route_index, obstacle_name, clearance) for current layout hits."""
    from mathutils import Matrix
    routes = list(routes or [])
    keepout_objs = [o for o in (keepout_objs or []) if o is not None and getattr(o, 'type', None) == 'MESH']
    if not routes or not keepout_objs:
        return []
    if depsgraph is None:
        try:
            import bpy
            depsgraph = bpy.context.evaluated_depsgraph_get()
        except Exception:
            depsgraph = None
    world_obstacles = []
    for obj in keepout_objs:
        obs = mesh_obstacle_from_object(obj, Matrix.Identity(4), cell_size=0.0508, depsgraph=depsgraph)
        if obs:
            world_obstacles.append(obs)
    hits = []
    for i, route in enumerate(routes):
        try:
            chain = route_world_capsule_chain(route, include_finish_extension=True)
        except Exception:
            chain = []
        for obs in world_obstacles:
            clr = capsule_chain_mesh_clearance(chain, obs, extra_clearance)
            if clr < -1.0e-8:
                hits.append((i, obs.get('name','Keep-Out'), float(clr)))
    return hits
