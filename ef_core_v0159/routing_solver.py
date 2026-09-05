"""Assisted Header-primary routing for Exhaust Fabricator v0.14.3.

The solver deliberately produces fabrication-readable Route segments: straight
runs and constant-CLR mandrel bends.  A 4-line / 3-corner virtual polyline is
filleted with one constant radius, which guarantees exact endpoint position and
tangent whenever a feasible candidate exists.  The middle waypoint can be
pushed sideways to add equal-length dogleg distance.
"""
import math
from mathutils import Vector, Matrix
from .collision import capsule_chain_clearance, capsule_chain_mesh_clearance, transform_capsule_chain
from .profiling import stage

_EPS = 1.0e-9


def _safe_normal(v, fallback=Vector((1.0, 0.0, 0.0))):
    if v.length < _EPS:
        return fallback.copy()
    return v.normalized()


def _clamp(x, a, b):
    return max(a, min(b, x))


def _rotation(axis, angle):
    return Matrix.Rotation(float(angle), 4, _safe_normal(axis, Vector((1.0, 0.0, 0.0))))


def _angle_between(a, b):
    a = _safe_normal(a)
    b = _safe_normal(b)
    return math.acos(_clamp(float(a.dot(b)), -1.0, 1.0))


def _signed_angle_around(a, b, axis):
    """Signed angle from a to b around normalized axis."""
    a = _safe_normal(a)
    b = _safe_normal(b)
    axis = _safe_normal(axis)
    return math.atan2(float(axis.dot(a.cross(b))), _clamp(float(a.dot(b)), -1.0, 1.0))


def _turn_from_to(tangent, up, desired):
    """Return (angle, clocking, tangent_end, up_end) using Route bend semantics."""
    t = _safe_normal(tangent)
    u = _safe_normal(up - t * up.dot(t), Vector((0.0, 0.0, 1.0)))
    d = _safe_normal(desired, t)
    theta = _angle_between(t, d)
    if theta < 1.0e-7:
        return 0.0, 0.0, t.copy(), u.copy()
    if theta > math.pi - 1.0e-6:
        return None
    axis = _safe_normal(t.cross(d), Vector((0.0, 1.0, 0.0)))
    bend_dir = _safe_normal(axis.cross(t), u)
    clock = _signed_angle_around(u, bend_dir, t)
    rot = _rotation(axis, theta)
    t2 = _safe_normal(rot @ t, d)
    # Mirrors geometry._segment_bend_basis / route_points update semantics.
    u2 = _safe_normal(rot @ bend_dir, u)
    u2 = _safe_normal(u2 - t2 * u2.dot(t2), Vector((0.0, 0.0, 1.0)))
    return theta, clock, t2, u2


def _candidate(target_point, target_tangent, radius, a, b, offset, dogleg_clocking,
               min_straight, max_bend_angle, start_radius=None):
    """Build one exact-pose 3-corner filleted-polyline candidate.

    S=(0,0,0), start tangent=+X.  A lies on the start ray, B lies backwards
    from the target along its requested final tangent, and M is the midpoint of
    AB shifted in a clockable lateral direction.  Filleting A/M/B gives 4
    straights + 3 mandrel bends.

    `start_radius`, when given, is used only for the first bend (at corner A,
    nearest the search's own start pose) instead of `radius`; the other two
    bends (at M and B) always use `radius`. This mirrors a common real-world
    fabrication practice: a tighter CLR right off the cylinder head flange to
    clear packaging (steering shaft, frame rail, motor mount), transitioning
    to a standard/sweeping CLR for the rest of the primary. Leaving it None
    (the default) reproduces the exact prior single-radius behavior.
    """
    S = Vector((0.0, 0.0, 0.0))
    t0 = Vector((1.0, 0.0, 0.0))
    up0 = Vector((0.0, 0.0, 1.0))
    T = Vector(target_point)
    tf = _safe_normal(Vector(target_tangent), t0)
    R = max(1.0e-6, float(radius))
    R1 = max(1.0e-6, float(start_radius)) if start_radius is not None else R
    radii = (R1, R, R)

    lateral = _rotation(t0, dogleg_clocking) @ up0
    lateral = _safe_normal(lateral - t0 * lateral.dot(t0), up0)

    A = S + t0 * max(0.0, float(a))
    B = T - tf * max(0.0, float(b))
    M = (A + B) * 0.5 + lateral * float(offset)

    v1 = M - A
    v2 = B - M
    d1_len = v1.length
    d2_len = v2.length
    if d1_len < 1.0e-7 or d2_len < 1.0e-7:
        return None
    d1 = v1 / d1_len
    d2 = v2 / d2_len

    theta1 = _angle_between(t0, d1)
    theta2 = _angle_between(d1, d2)
    theta3 = _angle_between(d2, tf)
    thetas = (theta1, theta2, theta3)
    if any(t >= math.pi - 1.0e-5 for t in thetas):
        return None
    if any(t > max_bend_angle + 1.0e-8 for t in thetas):
        return None

    trims = []
    for theta, r in zip(thetas, radii):
        if theta < 1.0e-7:
            trims.append(0.0)
        else:
            trims.append(r * math.tan(theta * 0.5))
    q1, q2, q3 = trims

    L0 = float(a) - q1
    L1 = d1_len - q1 - q2
    L2 = d2_len - q2 - q3
    L3 = float(b) - q3
    lengths = (L0, L1, L2, L3)
    minL = max(0.0, float(min_straight))
    # Zero-angle bends are allowed to collapse an adjacent line, but otherwise
    # keep fabrication straights above the requested minimum.
    for L in lengths:
        if L < minL - 1.0e-8:
            return None

    # Convert desired line directions into Route clocking values, carrying the
    # actual Route up-vector through each bend.
    t = t0.copy(); up = up0.copy()
    turns = []
    for desired in (d1, d2, tf):
        turn = _turn_from_to(t, up, desired)
        if turn is None:
            return None
        angle, clock, t, up = turn
        turns.append((angle, clock))

    total = sum(lengths) + sum(r * theta for r, theta in zip(radii, thetas))
    return {
        'lengths': lengths,
        'turns': turns,
        'radius': R,
        'radii': radii,
        'total_length': float(total),
        'offset': float(offset),
        'a': float(a),
        'b': float(b),
        'max_angle': max(thetas),
        'target_point': T.copy(),
        'target_tangent': tf.copy(),
        'guide_probes': (A.copy(), M.copy(), B.copy()),
    }


def _linspace(lo, hi, count):
    if count <= 1 or abs(hi - lo) < 1.0e-12:
        return [0.5 * (lo + hi)]
    return [lo + (hi - lo) * i / (count - 1) for i in range(count)]



def candidate_centerline_points(candidate, bend_samples=12):
    """Sample the exact Route centerline represented by a solver candidate."""
    if not candidate:
        return []
    p = Vector((0.0, 0.0, 0.0))
    tangent = Vector((1.0, 0.0, 0.0))
    up = Vector((0.0, 0.0, 1.0))
    radii = candidate.get('radii') or (candidate['radius'],) * 3
    points = [p.copy()]
    lengths = candidate['lengths']
    turns = candidate['turns']
    steps = max(3, int(bend_samples))
    for i in range(4):
        L = max(0.0, float(lengths[i]))
        if L > 1.0e-9:
            p = p + tangent * L
            points.append(p.copy())
        if i >= 3:
            continue
        angle, clock = turns[i]
        angle = abs(float(angle))
        if angle <= 1.0e-9:
            continue
        R = max(1.0e-6, float(radii[i]))
        clock_rot = _rotation(tangent, float(clock))
        bend_dir = _safe_normal(clock_rot @ up, up)
        bend_dir = _safe_normal(bend_dir - tangent * bend_dir.dot(tangent), up)
        axis = _safe_normal(tangent.cross(bend_dir), Vector((0.0, 1.0, 0.0)))
        center = p + bend_dir * R
        radial0 = p - center
        for k in range(1, steps + 1):
            rot = _rotation(axis, angle * (k / steps))
            points.append((center + (rot @ radial0)).copy())
        rot_end = _rotation(axis, angle)
        tangent = _safe_normal(rot_end @ tangent, tangent)
        up = _safe_normal(rot_end @ bend_dir, up)
        up = _safe_normal(up - tangent * up.dot(tangent), Vector((0.0, 0.0, 1.0)))
        p = points[-1].copy()
    return points



def candidate_capsule_chain(candidate, tube_radius, bend_samples=12):
    """Conservative local-space capsule chain for a solver candidate.

    Each bend chord is inflated by its circular-arc sagitta so the collision
    envelope contains the true mandrel-bend sweep rather than cutting inside it.
    """
    if not candidate:
        return []
    p = Vector((0.0, 0.0, 0.0))
    tangent = Vector((1.0, 0.0, 0.0))
    up = Vector((0.0, 0.0, 1.0))
    radii = candidate.get('radii') or (candidate['radius'],) * 3
    base_r = max(0.0, float(tube_radius))
    chain = []
    lengths = candidate['lengths']
    turns = candidate['turns']
    steps = max(2, int(bend_samples))
    for i in range(4):
        L = max(0.0, float(lengths[i]))
        if L > 1.0e-9:
            q = p + tangent * L
            chain.append((p.copy(), q.copy(), base_r))
            p = q
        if i >= 3:
            continue
        angle, clock = turns[i]
        angle = abs(float(angle))
        if angle <= 1.0e-9:
            continue
        R = max(1.0e-6, float(radii[i]))
        clock_rot = _rotation(tangent, float(clock))
        bend_dir = _safe_normal(clock_rot @ up, up)
        bend_dir = _safe_normal(bend_dir - tangent * bend_dir.dot(tangent), up)
        axis = _safe_normal(tangent.cross(bend_dir), Vector((0.0, 1.0, 0.0)))
        center = p + bend_dir * R
        radial0 = p - center
        step_angle = angle / steps
        sagitta = R * (1.0 - math.cos(step_angle * 0.5))
        chord_r = base_r + sagitta + 1.0e-10
        prev = p.copy()
        for k in range(1, steps + 1):
            rot = _rotation(axis, angle * (k / steps))
            q = center + (rot @ radial0)
            chain.append((prev.copy(), q.copy(), chord_r))
            prev = q
        rot_end = _rotation(axis, angle)
        tangent = _safe_normal(rot_end @ tangent, tangent)
        up = _safe_normal(rot_end @ bend_dir, up)
        up = _safe_normal(up - tangent * up.dot(tangent), Vector((0.0, 0.0, 1.0)))
        p = prev.copy()
    return chain


def _candidate_collision_clearance(candidate, candidate_radius, obstacles, extra_clearance, bend_samples,
                                    route_matrix_world=None):
    """Return conservative surface clearance to primary and mesh obstacles.

    A keep-out obstacle built once in world space (see operators.py's
    `_build_world_keepout_obstacles`) is tagged `'space': 'WORLD'`.  For those,
    this transforms the small per-candidate capsule chain into world space
    instead -- cheap, since a candidate chain is only a few dozen segments,
    versus re-evaluating/re-triangulating/re-gridding the whole keep-out mesh
    per primary the way rebuilding it in route-local space would require.
    """
    if not obstacles:
        return float('inf')
    with stage('candidate_capsule_chain'):
        chain = candidate_capsule_chain(candidate, candidate_radius, bend_samples=bend_samples)
    chain_world = None
    best = float('inf')
    for obstacle in obstacles:
        kind = str(obstacle.get('type', 'CAPSULE')).upper()
        clearance = max(0.0, float(obstacle.get('clearance', extra_clearance)))
        if kind == 'MESH':
            with stage('keepout_collision_check'):
                if obstacle.get('space') == 'WORLD' and route_matrix_world is not None:
                    if chain_world is None:
                        with stage('candidate_world_transform'):
                            chain_world = transform_capsule_chain(chain, route_matrix_world)
                    clr = capsule_chain_mesh_clearance(chain_world, obstacle.get('mesh'), clearance)
                else:
                    clr = capsule_chain_mesh_clearance(chain, obstacle.get('mesh'), clearance)
        else:
            ochain = obstacle.get('chain') or []
            with stage('primary_collision_check'):
                clr = capsule_chain_clearance(chain, ochain, clearance, chain_b_bounds=obstacle.get('bounds'))
        if clr < best:
            best = clr
        if clr < -1.0e-8:
            return clr
    return best


def solve_primary_route(target_point, target_tangent, target_length, radius,
                        min_straight=0.25, max_bend_angle=math.radians(120.0),
                        max_dogleg_offset=0.20, dogleg_clocking=0.0,
                        quality='NORMAL', match_length=True, shortest_route=False,
                        length_tolerance=0.0, collision_obstacles=None,
                        candidate_radius=0.0, collision_clearance=0.0,
                        avoid_collisions=False, auto_clocking_search=True, route_bend_resolution=16,
                        guide_probe_points=None, guide_influence=0.0, route_matrix_world=None,
                        dense_search=False, start_radius=None):
    """Search for an exact endpoint/tangent Route candidate.

    `start_radius`, when given, uses a different CLR for only the first bend
    (the one nearest the search's own start pose) instead of `radius` --
    matching the common fabrication practice of a tighter bend right off the
    cylinder head flange for packaging clearance, then a standard/sweeping
    CLR for the rest of the primary. Fixed by the caller, not searched or
    refined, since it represents a real physical constraint (a specific CLR
    tube/die available, or a specific clearance needed) rather than a free
    optimization variable. Leaving it None reproduces the exact prior
    single-radius behavior.

    Returns a candidate dict or None.  Position and tangent are guaranteed by
    construction.  With match_length enabled, optimization approaches the
    requested target length.  Otherwise shortest_route can make actual
    centerline length the primary objective; the fallback mode prefers compact,
    gentle feasible geometry.

    `dense_search`, when True, uses the full (pre-v0.14.13) coarse clocking
    grid and skips locally refining clocking afterward -- a slower but more
    "conservative" search whose per-primary result is closer to what an
    earlier, unperturbed grid would have found. This exists specifically as a
    fallback: v0.14.13 found that the faster default search (a coarser
    clocking grid, compensated by also refining clocking locally) can find a
    genuinely *better* per-primary candidate that nonetheless occupies the
    shared 3D volume differently than before -- and in a Header whose full
    primary set is already solved with near-zero packing slack, that shift
    alone (with no bug anywhere) can be enough to make a later primary in the
    same solve order infeasible, even though every individual candidate along
    the way was fully valid on its own. See collision.py's module docstring
    history / the v0.14.13 handoff entry for the empirical case that exposed
    this (a primary landing at 0.00006" clearance under the old dense search,
    which the faster search legitimately improved to 0.00057" clearance --
    yet that specific shift blocked every tried solve order). Callers doing a
    multi-primary group solve should retry with `dense_search=True` only after
    the fast default has been given a full chance and failed outright, not
    use it as a first resort.

    `dense_search=True` also restores the full pre-v0.14.7 collision-search
    bend-sample density (see `collision_bend_samples` below) instead of the
    coarser default. This is a different flavor of the same underlying risk:
    a coarser proxy chain's sagitta inflation is always *safe* (it never lets
    a real collision through), but it is also more conservative than the
    exact postbuild gate, so it can reject a candidate that just barely
    clears a keep-out at full resolution purely from that extra margin, not
    from an actual collision. In a tightly-packed Header that's the same
    category of failure as the clocking case above -- an over-conservative
    rejection of an otherwise-valid marginal candidate can make a later
    primary infeasible -- so it gets the same fast-default / dense-fallback
    treatment.

    `dense_search=True` also restores the original, denser per-quality a/b/
    offset coarse grid (`qmap` below) instead of the reduced default. This
    grid is the single biggest cost driver (candidate count scales roughly as
    n^3), so it carries the largest speed benefit of the three `dense_search`
    dimensions, but the same risk logic applies: local refinement of a, b,
    and offset (already present regardless of `dense_search`, since it
    predates this change) compensates for a coarser coarse grid in the common
    case, but a coarser grid can still accept or reject a different marginal
    candidate than the dense grid would in a near-zero-slack Header.
    """
    T = Vector(target_point)
    tf = _safe_normal(Vector(target_tangent), Vector((1.0, 0.0, 0.0)))
    distance = T.length
    R = max(1.0e-6, float(radius))
    minL = max(0.0, float(min_straight))
    max_off = max(0.0, float(max_dogleg_offset))
    target_len = max(0.0, float(target_length))
    length_tol = max(0.0, float(length_tolerance))
    obstacles = list(collision_obstacles or [])
    candidate_radius = max(0.0, float(candidate_radius))
    collision_clearance = max(0.0, float(collision_clearance))
    max_angle = _clamp(float(max_bend_angle), math.radians(5.0), math.radians(175.0))
    guide_pts = [Vector(p) for p in (guide_probe_points or [])][:3]
    guide_weight = _clamp(float(guide_influence), 0.0, 1.0) if len(guide_pts) == 3 else 0.0

    # The a/b/offset coarse grid is the single largest remaining cost driver:
    # candidate count scales as n^2 * off_steps ~ n^3, so cutting n compounds
    # fast. The existing local-refinement pass below already hill-climbs a,
    # b, and offset around the coarse winner (it predates this change and
    # needed no new dimension, unlike clocking in v0.14.13), so a coarser
    # coarse grid here is compensated the same way that refinement already
    # compensates for a coarse grid in general -- not a new mechanism, just
    # leaning on it harder. Carries the same risk already documented for
    # clocking (v0.14.13) and collision_bend_samples (v0.14.14): a coarser
    # grid can accept/reject a different marginal candidate than the dense
    # grid would, which in a near-zero-slack multi-primary Header can make a
    # later primary infeasible even though nothing found along the way was
    # actually wrong. Same treatment: fast default, `dense_search=True`
    # restores the original per-quality density, and the existing
    # `_solve_header_primary_set` fast-then-dense fallback covers it for free.
    qmap = (
        {'FAST': 6, 'NORMAL': 9, 'HIGH': 13} if dense_search
        else {'FAST': 4, 'NORMAL': 6, 'HIGH': 9}
    )
    n = qmap.get(str(quality), 9 if dense_search else 6)
    if guide_weight > 0.0:
        # A user guide supplies useful coarse routing intent, so fewer blind a/b
        # samples are necessary. The range is narrowed, never collapsed.
        n = max(4, int(round(n * (1.0 - 0.30 * guide_weight))))
    # Collision-search bend sampling is independent of the final built Route's
    # bend resolution: the sagitta-based radius inflation in
    # candidate_capsule_chain stays conservative at ANY sample count, so a
    # coarser proxy chain is still safe here and never gets less conservative
    # than the exact postbuild clearance gate that verifies the real geometry
    # afterward.  Previously this piggybacked on route_bend_resolution (which
    # exists for visual smoothness, e.g. 16-32+), needlessly multiplying the
    # segment count -- and therefore the pairwise collision cost -- of every
    # single candidate evaluated during the search.
    #
    # A coarser proxy chain is "safe" in the sense of never letting a real
    # collision through (sagitta inflation over-covers the true sweep at any
    # sample count), but that same inflation also makes the search's own
    # accept/reject test MORE conservative than the exact postbuild gate: a
    # candidate that just barely clears a keep-out at full resolution can get
    # rejected here purely from a coarser chord's larger sagitta margin, not
    # from an actual collision. In a tightly-packed Header, rejecting an
    # otherwise-valid marginal candidate is exactly the kind of thing that can
    # make a later primary's own solve infeasible -- the same category of
    # sensitivity `dense_search` already exists to recover for the clocking
    # grid (see this function's docstring), so it gets the same treatment
    # here: fewer samples (and therefore more conservative margin) by default,
    # falling back to the full pre-v0.14.7 density -- least conservative,
    # closest to the true swept geometry -- only when the fast search's group
    # solve found no complete solution at all.
    if dense_search:
        bend_samples = max(2, int(route_bend_resolution), {'FAST': 8, 'NORMAL': 12, 'HIGH': 18}.get(str(quality), 12))
    else:
        collision_bend_samples = {'FAST': 4, 'NORMAL': 6, 'HIGH': 10}.get(str(quality), 6)
        bend_samples = max(2, min(int(route_bend_resolution), collision_bend_samples))
    if avoid_collisions and auto_clocking_search:
        # The coarse clocking grid multiplies the entire a/b/offset grid,
        # making it the single largest cost lever here. The default
        # (dense_search=False) trims it and compensates by also refining
        # clocking locally below -- cheaper in the common case, but see this
        # function's docstring for the specific way that can interact with
        # multi-primary packing feasibility. dense_search=True restores the
        # original, more conservative density with no clocking refinement.
        clock_count = (
            {'FAST': 5, 'NORMAL': 9, 'HIGH': 17} if dense_search
            else {'FAST': 3, 'NORMAL': 5, 'HIGH': 9}
        ).get(str(quality), 9 if dense_search else 5)
        clock_half_range = math.pi * 0.5
        if guide_weight > 0.0:
            clock_count = max(5, int(round(clock_count * (1.0 - 0.25 * guide_weight))))
            # Still allow substantial deviation from the sketch when hard
            # collision / CLR constraints require it.
            clock_half_range *= (1.0 - 0.35 * guide_weight)
        clock_vals = [dogleg_clocking + d for d in _linspace(-clock_half_range, clock_half_range, clock_count)]
    else:
        clock_vals = [float(dogleg_clocking)]
    # Search virtual corner distances broadly enough to allow long doglegs but
    # keep the initial coarse grid bounded for interactive use.
    # Equal-length search may need a broad range to deliberately add length.
    # Shortest-route search must not inherit that target-length bias, otherwise
    # a large Header reference can make the search unnecessarily long-legged.
    if match_length:
        search_len = target_len
        length_scale = max(distance, search_len, 6.0 * R, 4.0 * minL, 0.05)
        run_hi = max(0.45 * distance, 0.55 * search_len, 3.0 * R, minL + R)
    else:
        length_scale = max(distance, 6.0 * R, 4.0 * minL, 0.05)
        # Allow enough virtual-run range for strongly offset / opposed tangents
        # while keeping the grid bounded for interactive use.
        run_hi = max(1.25 * distance, 4.0 * R, minL + R)
    run_hi = min(run_hi, max(length_scale * 1.50, run_hi))
    run_lo = minL

    a_lo, a_hi = run_lo, run_hi
    b_lo, b_hi = run_lo, run_hi
    off_lo, off_hi = -max_off, max_off
    if guide_weight > 0.0:
        gp25, gp50, gp75 = guide_pts
        a_hint = _clamp(float(gp25.x), run_lo, run_hi)
        b_hint = _clamp(float((T - gp75).dot(tf)), run_lo, run_hi)
        range_factor = max(0.45, 1.0 - 0.55 * guide_weight)
        half_run = 0.5 * (run_hi - run_lo) * range_factor
        a_lo, a_hi = max(run_lo, a_hint-half_run), min(run_hi, a_hint+half_run)
        b_lo, b_hi = max(run_lo, b_hint-half_run), min(run_hi, b_hint+half_run)
        if a_hi-a_lo < R*0.25:
            a_lo, a_hi = run_lo, run_hi
        if b_hi-b_lo < R*0.25:
            b_lo, b_hi = run_lo, run_hi
        if max_off > 1.0e-9:
            pref_lat = _rotation(Vector((1.0,0.0,0.0)), dogleg_clocking) @ Vector((0.0,0.0,1.0))
            A_hint = Vector((a_hint, 0.0, 0.0))
            B_hint = T - tf * b_hint
            off_hint = float((gp50 - (A_hint+B_hint)*0.5).dot(pref_lat))
            half_off = max(max_off * 0.25, max_off * range_factor)
            off_lo = max(-max_off, off_hint-half_off)
            off_hi = min(max_off, off_hint+half_off)
            if off_hi-off_lo < max_off*0.20:
                off_lo, off_hi = -max_off, max_off

    a_vals = _linspace(a_lo, a_hi, n)
    b_vals = _linspace(b_lo, b_hi, n)
    off_steps = max(5, 2 * n - 1)
    off_vals = _linspace(off_lo, off_hi, off_steps) if max_off > 1.0e-9 else [0.0]

    best = None

    collision_rejections = 0

    def consider(c, used_clocking):
        nonlocal best, collision_rejections
        if c is None:
            return
        min_clearance = float('inf')
        if avoid_collisions and obstacles:
            min_clearance = _candidate_collision_clearance(
                c, candidate_radius, obstacles, collision_clearance, bend_samples,
                route_matrix_world=route_matrix_world,
            )
            if min_clearance < -1.0e-8:
                collision_rejections += 1
                return
        c['min_primary_clearance_margin'] = float(min_clearance)
        c['min_primary_clearance'] = float(min_clearance) if math.isfinite(min_clearance) else float('inf')
        c['dogleg_clocking_used'] = float(used_clocking)
        clock_delta = abs(_signed_angle_around(
            Vector((0.0, math.cos(dogleg_clocking), math.sin(dogleg_clocking))),
            Vector((0.0, math.cos(used_clocking), math.sin(used_clocking))),
            Vector((1.0, 0.0, 0.0)),
        ))
        guide_error = 0.0
        if guide_weight > 0.0:
            probes = c.get('guide_probes') or ()
            if len(probes) == 3:
                guide_error = sum((Vector(a)-Vector(b)).length for a,b in zip(probes, guide_pts)) / 3.0
        c['guide_error'] = float(guide_error)
        # Influence scales the soft guide term only; it never precedes collision,
        # endpoint, equal-length or shortest-route hard objectives.
        guide_term = guide_error * guide_weight
        if match_length:
            len_err = abs(c['total_length'] - target_len)
            # A route inside the Header tolerance is considered equal-length.
            # Inside that accepted band, prefer the shortest actual route first,
            # then the smallest excursion / preferred dogleg plane / bend angle.
            outside_tol = max(0.0, len_err - length_tol)
            if outside_tol <= 1.0e-12:
                key = (0.0, c['total_length'], guide_term, abs(c['offset']), clock_delta, c['max_angle'], -min_clearance)
            else:
                key = (outside_tol, len_err, c['total_length'], guide_term, abs(c['offset']), clock_delta, c['max_angle'], -min_clearance)
        elif shortest_route:
            key = (c['total_length'], guide_term, abs(c['offset']), clock_delta, c['max_angle'], -min_clearance)
        else:
            compact = abs(c['offset']) + c['max_angle'] * length_scale * 0.05
            key = (compact + guide_term, guide_term, c['max_angle'], c['total_length'], clock_delta, -min_clearance)
        if best is None or key < best[0]:
            best = (key, c)

    with stage('candidate_search_coarse'):
        for used_clocking in clock_vals:
            for a in a_vals:
                for b in b_vals:
                    for off in off_vals:
                        with stage('candidate_generation'):
                            c = _candidate(T, tf, R, a, b, off, used_clocking, minL, max_angle, start_radius=start_radius)
                        consider(c, used_clocking)

    if best is None:
        if guide_weight > 0.0:
            # The guide is explicitly soft. If the narrowed guide-biased search
            # cannot satisfy the hard geometry/collision rules, automatically
            # fall back to the full legacy search space before declaring failure.
            return solve_primary_route(
                T, tf, target_len, R, minL, max_angle, max_off, dogleg_clocking,
                quality, match_length, shortest_route, length_tol, obstacles,
                candidate_radius, collision_clearance, avoid_collisions,
                auto_clocking_search, route_bend_resolution, None, 0.0,
                route_matrix_world=route_matrix_world, dense_search=dense_search,
                start_radius=start_radius,
            )
        return None

    # Local coordinate refinement around the coarse best.
    c = best[1]
    da = max((a_hi - a_lo) / max(1, n - 1), R * 0.25)
    db = max((b_hi - b_lo) / max(1, n - 1), R * 0.25)
    do = max((off_hi - off_lo) / max(1, off_steps - 1), R * 0.20) if max_off > 0 else 0.0
    # Clocking is only refined in the fast (non-dense) search: dense_search
    # already used the full clocking grid, so there is no compensation to
    # apply, and skipping this dimension there keeps the fallback path as
    # close as possible to the original, pre-v0.14.13 search it stands in for.
    refine_clocking = (not dense_search) and avoid_collisions and auto_clocking_search and len(clock_vals) > 1
    dc = (clock_half_range / max(1, len(clock_vals) - 1)) if refine_clocking else 0.0
    with stage('candidate_search_refine'):
        for _ in range(5):
            base = best[1]
            base_clock = float(base.get('dogleg_clocking_used', dogleg_clocking))
            for sa in (-1.0, 0.0, 1.0):
                for sb in (-1.0, 0.0, 1.0):
                    for so in ((-1.0, 0.0, 1.0) if do > 0 else (0.0,)):
                        for sc in ((-1.0, 0.0, 1.0) if dc > 0 else (0.0,)):
                            a = _clamp(base['a'] + sa * da, a_lo, a_hi)
                            b = _clamp(base['b'] + sb * db, b_lo, b_hi)
                            off = _clamp(base['offset'] + so * do, off_lo, off_hi)
                            used_clocking = base_clock + sc * dc
                            with stage('candidate_generation'):
                                c = _candidate(T, tf, R, a, b, off, used_clocking, minL, max_angle, start_radius=start_radius)
                            consider(c, used_clocking)
            da *= 0.45; db *= 0.45; do *= 0.45; dc *= 0.45

    result = best[1]
    result['length_error'] = float(result['total_length'] - target_len) if match_length else 0.0
    result['requested_length'] = float(target_len) if match_length else 0.0
    result['objective'] = 'EQUAL_LENGTH' if match_length else ('SHORTEST' if shortest_route else 'FEASIBLE')
    result['collision_rejections'] = int(collision_rejections)
    result['collision_avoidance'] = bool(avoid_collisions)
    return result


def candidate_segments(candidate, bend_resolution=16):
    """Convert a candidate into Route-segment dictionaries."""
    if not candidate:
        return []
    radii = candidate.get('radii') or (candidate['radius'],) * 3
    lengths = candidate['lengths']
    turns = candidate['turns']
    result = []
    for i in range(4):
        L = max(0.0, float(lengths[i]))
        if L > 1.0e-7:
            result.append({'kind': 'STRAIGHT', 'length': L})
        if i < 3:
            angle, clock = turns[i]
            if abs(angle) > 1.0e-7:
                result.append({
                    'kind': 'BEND', 'bend_style': 'MANDREL', 'radius': float(radii[i]),
                    'angle': float(angle), 'clocking': float(clock),
                    'resolution': max(2, int(bend_resolution)),
                })
    return result
