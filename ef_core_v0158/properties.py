import math
import bpy
from bpy.props import (
    BoolProperty, CollectionProperty, EnumProperty, FloatProperty,
    IntProperty, PointerProperty, StringProperty,
)

INCH = 0.0254


def _update_route(self, context):
    try:
        from .geometry import rebuild_route_object
        rebuild_route_object(self.id_data)
    except Exception:
        pass


def _update_collector(self, context):
    try:
        from .geometry import rebuild_collector_object
        rebuild_collector_object(self.id_data)
    except Exception:
        pass


def _update_reducer(self, context):
    try:
        from .geometry import rebuild_reducer_object
        rebuild_reducer_object(self.id_data)
    except Exception:
        pass


def _update_y_pipe(self, context):
    try:
        from .geometry import rebuild_y_pipe_object
        rebuild_y_pipe_object(self.id_data)
    except Exception:
        pass


def _update_x_pipe(self, context):
    try:
        from .geometry import rebuild_x_pipe_object
        rebuild_x_pipe_object(self.id_data)
    except Exception:
        pass


def _update_h_pipe(self, context):
    try:
        from .geometry import rebuild_h_pipe_object
        rebuild_h_pipe_object(self.id_data)
    except Exception:
        pass


def _update_pie_cut(self, context):
    try:
        from .geometry import rebuild_pie_cut_object
        rebuild_pie_cut_object(self.id_data)
    except Exception:
        pass


def _update_active_segment(self, context):
    try:
        from .guides import tag_view3d_redraw
        tag_view3d_redraw()
    except Exception:
        pass



def header_primary_objects(header_obj):
    """Return Header-owned primary Route objects in stable primary-index order.

    Header membership intentionally uses Blender's parent/child relationship and
    object metadata instead of a nested CollectionProperty.  This keeps add-on
    registration independent of nested RNA collection types and makes Header
    objects resilient when files are reopened or routes are renamed.
    """
    if not header_obj:
        return []
    result = []
    for obj in getattr(header_obj, "children", ()):
        route_settings = getattr(obj, "exhaust_route", None)
        if route_settings and route_settings.is_route:
            result.append(obj)
    result.sort(key=lambda o: (int(o.get("exhaust_header_primary_index", 10**6)), o.name))
    return result




def collector_inlet_connectors(collector_obj):
    """Return per-primary collector inlet connector metadata in local space.

    Each item is a dict with index, point, tangent, od and id.  The radial
    collector generator writes these properties every time it rebuilds.
    """
    if not collector_obj:
        return []
    cs = getattr(collector_obj, "exhaust_collector", None)
    if not cs or not cs.is_collector:
        return []
    count = int(collector_obj.get("exhaust_collector_inlet_count", collector_obj.get("exhaust_collector_count", 0)))
    result = []
    for i in range(max(0, count)):
        pk = f"exhaust_collector_inlet_{i}"
        tk = f"exhaust_collector_inlet_{i}_tangent"
        if pk not in collector_obj or tk not in collector_obj:
            continue
        result.append({
            "index": i,
            "point": tuple(collector_obj[pk]),
            "tangent": tuple(collector_obj[tk]),
            "od": float(collector_obj.get(f"exhaust_collector_inlet_{i}_od", cs.primary_od)),
            "id": float(collector_obj.get(f"exhaust_collector_inlet_{i}_id", max(0.0, cs.primary_od - 2.0 * cs.wall_thickness))),
        })
    return result


def header_target_collector(header_obj):
    hs = getattr(header_obj, "exhaust_header", None) if header_obj else None
    if not hs or not hs.is_header or not hs.collector_target_name:
        return None
    obj = bpy.data.objects.get(hs.collector_target_name)
    cs = getattr(obj, "exhaust_collector", None) if obj else None
    return obj if cs and cs.is_collector else None


def header_port_index(header_obj, primary_index, collector_count=None):
    """Map a Header primary index to a radial collector inlet index."""
    hs = header_obj.exhaust_header
    if collector_count is None:
        target = header_target_collector(header_obj)
        collector_count = len(collector_inlet_connectors(target)) if target else 0
    n = max(0, int(collector_count))
    if n <= 0:
        return -1
    direction = -1 if hs.collector_reverse_order else 1
    return (int(hs.collector_port_offset) + direction * int(primary_index)) % n


def header_collector_status(header_obj):
    """Live world-space Header-primary to collector-port alignment metrics.

    Returns one dict per mapped primary with position error and tangent-angle
    error.  This is intentionally computed on demand so moving either object in
    Blender updates the UI/guides without adding depsgraph callbacks.
    """
    target = header_target_collector(header_obj)
    if target is None:
        return []
    ports = collector_inlet_connectors(target)
    routes = header_primary_objects(header_obj)
    if not ports or not routes:
        return []
    from mathutils import Vector
    import math as _math
    out = []
    for i, route in enumerate(routes):
        if i >= len(ports):
            break
        pi = header_port_index(header_obj, i, len(ports))
        if not (0 <= pi < len(ports)):
            continue
        port = ports[pi]
        if "exhaust_connector_end" not in route or "exhaust_connector_end_tangent" not in route:
            continue
        rp = route.matrix_world @ Vector(route["exhaust_connector_end"])
        rt = (route.matrix_world.to_3x3() @ Vector(route["exhaust_connector_end_tangent"])).normalized()
        cp = target.matrix_world @ Vector(port["point"])
        ct = (target.matrix_world.to_3x3() @ Vector(port["tangent"])).normalized()
        dot = max(-1.0, min(1.0, float(rt.dot(ct))))
        angle = _math.acos(dot)
        out.append({
            "primary_index": i, "port_index": pi, "route": route, "collector": target,
            "route_point": rp, "route_tangent": rt, "port_point": cp, "port_tangent": ct,
            "position_error": float((rp-cp).length), "angle_error": float(angle),
        })
    return out


def refresh_header_stats(header_obj):
    settings = getattr(header_obj, "exhaust_header", None) if header_obj else None
    if not settings or not settings.is_header:
        return
    routes = header_primary_objects(header_obj)
    lengths = [float(r.exhaust_route.centerline_length) for r in routes]
    settings.actual_primary_count = len(lengths)
    if not lengths:
        settings.average_length = 0.0
        settings.minimum_length = 0.0
        settings.maximum_length = 0.0
        settings.length_spread = 0.0
        settings.all_within_tolerance = False
        return
    avg = sum(lengths) / len(lengths)
    settings.average_length = avg
    settings.minimum_length = min(lengths)
    settings.maximum_length = max(lengths)
    settings.length_spread = settings.maximum_length - settings.minimum_length
    target = avg if settings.target_mode == 'AVERAGE' else max(0.0, settings.target_length)
    tol = max(0.0, settings.length_tolerance)
    settings.all_within_tolerance = all(abs(v - target) <= tol + 1.0e-9 for v in lengths)


def _update_header_stats(self, context):
    try:
        refresh_header_stats(self.id_data)
    except Exception:
        pass


def _update_header_flange(self, context):
    """Rebuild the one shared cylinder-head flange owned by a Header."""
    try:
        header = self.id_data
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            return
        from .geometry import rebuild_header_flange_object
        rebuild_header_flange_object(header)
    except Exception:
        pass


def _update_header_tube_size(self, context):
    """Propagate Header primary OD/wall to its managed Route children."""
    try:
        header = self.id_data
        if not header or not getattr(header, "exhaust_header", None) or not header.exhaust_header.is_header:
            return
        from .geometry import rebuild_route_object
        for route in header_primary_objects(header):
            rs = route.exhaust_route
            # Header primaries always begin at the shared head flange.  Preserve
            # the normal Route properties for backward compatibility, but force
            # their start-side fabrication options to the neutral state.
            rs.start_end_type = 'PLAIN'
            rs.start_connection_type = 'NONE'
            # Assigning these normally triggers rebuild callbacks; assigning both
            # and then explicitly rebuilding keeps the final result deterministic.
            rs.outside_diameter = self.primary_od
            rs.wall_thickness = self.wall_thickness
            rebuild_route_object(route)
        from .geometry import rebuild_header_flange_object
        rebuild_header_flange_object(header)
        refresh_header_stats(header)
    except Exception:
        pass


class EXHAUST_PG_Header(bpy.types.PropertyGroup):
    is_header: BoolProperty(default=False)
    desired_primary_count: IntProperty(
        name="Primary Count", default=4, min=2, max=12,
        description="How many primary pipes (the individual tubes leaving each exhaust port) this Header should have. "
                    "Changing this number doesn't add or remove pipes by itself -- click Apply Count below to do that",
    )
    primary_od: FloatProperty(
        name="Primary OD", subtype='DISTANCE', default=1.75 * INCH, min=0.001,
        update=_update_header_tube_size,
        description="Outside diameter (thickness) of every primary pipe. All primaries on this Header share the same size",
    )
    wall_thickness: FloatProperty(
        name="Wall Thickness", subtype='DISTANCE', default=0.065 * INCH, min=0.0,
        update=_update_header_tube_size,
        description="Thickness of the pipe's metal wall. Doesn't change the outside size -- only the inside opening. "
                    "Thicker walls are stronger and quieter but heavier",
    )
    initial_primary_length: FloatProperty(
        name="Initial Primary Length", subtype='DISTANCE', default=24.0 * INCH, min=0.001,
        description="Starting length given to each primary pipe the moment it's created. "
                    "Purely a starting point -- routing or generating a pipe later changes its actual length",
    )
    port_spacing: FloatProperty(
        name="Port Spacing", subtype='DISTANCE', default=2.0 * INCH, min=0.0,
        description="Distance between the centers of neighboring exhaust ports, used when arranging primary start positions",
    )
    flange_thickness: FloatProperty(
        name="Flange Thickness", subtype='DISTANCE', default=0.375 * INCH, min=0.001,
        update=_update_header_flange,
        description="How thick the single metal flange plate is -- the plate that bolts to the cylinder head and holds every primary pipe",
    )
    flange_edge_margin: FloatProperty(
        name="Edge Margin", subtype='DISTANCE', default=0.375 * INCH, min=0.0,
        update=_update_header_flange,
        description="Extra metal left around the outside of the pipe holes, so the flange plate has enough material to stay strong",
    )
    flange_corner_radius: FloatProperty(
        name="Corner Radius", subtype='DISTANCE', default=0.250 * INCH, min=0.0,
        update=_update_header_flange,
        description="How rounded the flange plate's corners are. 0 gives sharp square corners; higher values round them off",
    )
    flange_bore_clearance: FloatProperty(
        name="Port Bore Clearance", subtype='DISTANCE', default=0.0, min=0.0,
        update=_update_header_flange,
        description="Extra room added around each pipe hole in the flange, beyond the pipe's own size, to make fitting/welding easier",
    )
    flange_corner_segments: IntProperty(
        name="Corner Segments", default=8, min=1, max=64,
        update=_update_header_flange,
        description="How smooth the rounded flange corners look. Higher numbers look smoother but add more geometry to the mesh",
    )
    flange_width: FloatProperty(name="Flange Width", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    flange_height: FloatProperty(name="Flange Height", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    flange_plane_deviation: FloatProperty(name="Start Plane Deviation", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})

    collector_target_name: StringProperty(
        name="Collector Target", default="",
        description="Name of the Collector object this Header's pipes are aimed at. "
                    "Set automatically by the Assign Selected Collector button",
    )
    collector_port_offset: IntProperty(
        name="Port Offset", default=0, min=0, max=11,
        description="Which collector inlet Primary 1 plugs into. The rest of the primaries follow around the collector from there. "
                    "Change this to rotate which pipe goes to which inlet",
        update=_update_active_segment,
    )
    collector_reverse_order: BoolProperty(
        name="Reverse Port Order", default=False,
        description="Match primaries to collector inlets going the opposite way around (clockwise instead of counter-clockwise, or vice versa)",
        update=_update_active_segment,
    )
    collector_position_tolerance: FloatProperty(
        name="Position Tolerance", subtype='DISTANCE', default=0.125 * INCH, min=0.0,
        description="How far a pipe end is allowed to sit from its collector inlet and still count as \"connected\" in the alignment check. Smaller = stricter",
    )
    collector_angle_tolerance: FloatProperty(
        name="Angle Tolerance", subtype='ANGLE', default=math.radians(2.0), min=0.0, soft_max=math.radians(20.0),
        description="How far a pipe end is allowed to point away from straight-into-the-collector and still count as \"aligned\". Smaller = stricter",
    )
    show_collector_targets: BoolProperty(
        name="Show Collector Targets", default=True,
        description="Draw small viewport-only rings and lines showing where each pipe should end up at the collector. "
                    "Just a visual guide -- never shows up in the actual pipe geometry",
        update=_update_active_segment,
    )
    collector_target_guide_offset: FloatProperty(
        name="Target Guide Offset", subtype='DISTANCE', default=0.030 * INCH, min=0.0,
        description="How far outside the pipe surface those viewport target rings are drawn, just so they don't visually overlap the pipe",
        update=_update_active_segment,
    )

    # Assisted equal-length primary routing.  The solver only emits ordinary
    # Route Straight + Mandrel Bend segments; these settings constrain that
    # fabrication-readable route rather than introducing a hidden spline.
    solver_clr: FloatProperty(
        name="Routing CLR", subtype='DISTANCE', default=3.0 * INCH, min=0.001,
        description="How tight or gentle the bends are that the assisted router creates. "
                    "\"CLR\" = Centerline Radius: a smaller number makes sharp/tight bends, a larger number makes wide/gentle bends",
    )
    solver_min_straight: FloatProperty(
        name="Minimum Straight", subtype='DISTANCE', default=0.50 * INCH, min=0.0,
        description="The shortest straight section the router will leave between two bends (or before/after a bend), so bends don't crowd into each other",
    )
    solver_max_bend_angle: FloatProperty(
        name="Maximum Bend Angle", subtype='ANGLE', default=math.radians(135.0),
        min=math.radians(5.0), max=math.radians(175.0),
        description="The sharpest single turn the router is allowed to create. Any route that would need a tighter bend than this is rejected as invalid",
    )
    solver_max_dogleg_offset: FloatProperty(
        name="Max Dogleg Offset", subtype='DISTANCE', default=8.0 * INCH, min=0.0,
        description="How far sideways the router may detour a pipe (a \"dogleg\") purely to add extra length, e.g. when matching all primaries to the same length",
    )
    solver_dogleg_clocking: FloatProperty(
        name="Dogleg Clocking", subtype='ANGLE', default=0.0,
        soft_min=-math.pi, soft_max=math.pi,
        description="Which direction (like a clock face, looking down the pipe) the router prefers to swing that sideways dogleg detour",
    )
    solver_bend_resolution: IntProperty(
        name="Bend Segments", default=16, min=2, max=128,
        description="How smooth the bends look on a generated pipe. Higher numbers look smoother but add more geometry to the mesh",
    )
    solver_quality: EnumProperty(
        name="Search Quality",
        description="How hard the router searches for a routing solution before giving up",
        items=[
            ('FAST', "Fast", "Quicker but rougher search -- good while experimenting with settings"),
            ('NORMAL', "Normal", "A good balance of speed and thoroughness for most Headers"),
            ('HIGH', "High", "Slower, more thorough search -- try this if Fast/Normal can't find a route that fits"),
        ],
        default='NORMAL',
    )
    solver_match_length: BoolProperty(
        name="Match Equal-Length Target", default=True,
        description="Try to make every generated pipe the same length as the target set below, instead of just taking the shortest path to the collector",
    )
    solver_shortest_route: BoolProperty(
        name="Shortest Possible Route", default=True,
        description="When Match Equal-Length Target is off, make the pipe as short as possible instead of any particular length "
                    "(all other limits like CLR, collisions, and reaching the collector still apply)",
    )
    solver_avoid_primary_collisions: BoolProperty(
        name="Avoid Other Primaries", default=True,
        description="Don't let a generated pipe's outer surface pass through another primary pipe on this Header",
    )
    solver_primary_clearance: FloatProperty(
        name="Primary Clearance", subtype='DISTANCE', default=0.0, min=0.0,
        description="Minimum gap to leave between pipes, on top of simply not overlapping. 0 allows pipes to just barely touch without passing through each other",
    )
    solver_auto_clocking_search: BoolProperty(
        name="Auto Search Dogleg Clocking", default=True,
        description="If the preferred Dogleg Clocking direction would cause a collision, automatically try other bend directions before giving up",
    )
    solver_avoid_keepouts: BoolProperty(
        name="Avoid Keep-Out Objects", default=True,
        description="Don't let a generated pipe's outer surface pass through any object you've marked as a keep-out (like an oil pan or frame rail)",
    )
    solver_keepout_clearance: FloatProperty(
        name="Keep-Out Clearance", subtype='DISTANCE', default=0.250 * INCH, min=0.0,
        description="Minimum gap to leave between a generated pipe and any keep-out object",
    )
    solver_adjust_collector_phase: BoolProperty(
        name="Auto Adjust Collector Radial Phase", default=True,
        description="Let Generate All Primaries also try slightly rotating the whole collector while searching, "
                    "in case a small rotation makes it possible to route every pipe cleanly",
    )
    solver_collector_phase_range: FloatProperty(
        name="Phase Search Range", subtype='ANGLE', default=math.radians(90.0), min=0.0, max=math.pi,
        description="How far Generate All Primaries is allowed to rotate the collector while searching, to either side of its current rotation",
    )
    solver_phase_auto_remap: BoolProperty(
        name="Auto Remap Ports While Phasing", default=True,
        description="While trying different collector rotations, also let the pipe-to-inlet matchups shuffle to whichever arrangement needs the least turning",
    )
    solver_order_success_cap: IntProperty(
        name="Orders To Compare", default=3, min=1, max=64,
        description="Generate All Primaries tries solving the primaries in different orders. This stops it from trying more orders once this many "
                    "complete, collision-free results have already been found, and keeps the best one. Raising it compares more alternatives "
                    "(and takes proportionally longer) but doesn't change whether a solution is found at all -- only how many good ones get compared",
    )
    solver_max_solve_seconds: FloatProperty(
        name="Max Search Time (s)", subtype='TIME', unit='TIME', default=60.0, min=5.0, soft_max=300.0,
        description="A safety time limit so Generate All Primaries doesn't search forever on a Header that has no valid solution. "
                    "Note this limit is applied TWICE in a row (a fast search, then a slower fallback search if the first one comes up empty), "
                    "so a genuinely impossible Header can take up to roughly 2x this value before it finally reports failure. The check only happens "
                    "between attempts, never in the middle of one, so an attempt that's almost done is allowed to finish rather than being cut off "
                    "just short of success -- meaning actual run time can run a bit over this number too. Doesn't apply to Generate Active Primary, "
                    "which only solves one pipe at a time and isn't part of this larger search",
    )
    solver_use_guide_splines: BoolProperty(
        name="Use Primary Guide Splines", default=False,
        description="Let you sketch a rough path for a pipe with an editable curve, then have the router loosely follow your sketch instead of guessing blind. "
                    "Hard limits (bend radius, collisions, reaching the collector) always win over your sketch",
    )
    solver_guide_influence: FloatProperty(
        name="Guide Influence", default=0.75, min=0.0, max=1.0, subtype='FACTOR',
        description="How closely the router sticks to your guide sketch. 0 ignores the sketch entirely; 1 follows it closely (still obeys all hard limits)",
    )
    solver_profiling_enabled: BoolProperty(
        name="Profile Solve Performance", default=True,
        description="Record how long each part of the next solve takes, so you can see what's slow. Purely for diagnosing performance -- turn off once you don't need it",
    )
    solver_profile_report: StringProperty(
        name="Last Solve Diagnostics", default="",
        description="Timing breakdown captured during the most recently profiled solve",
    )

    target_mode: EnumProperty(
        name="Length Reference",
        description="What every primary pipe's length is measured against for the equal-length check",
        items=[
            ('TARGET', "Target Length", "Compare every pipe's length against one number you type in below"),
            ('AVERAGE', "Current Average", "Compare every pipe's length against whatever the current average length happens to be"),
        ],
        default='TARGET', update=_update_header_stats,
    )
    target_length: FloatProperty(
        name="Target Primary Length", subtype='DISTANCE', default=32.0 * INCH, min=0.0,
        update=_update_header_stats,
        description="The length every primary pipe should try to match, used when Length Reference is set to Target Length",
    )
    length_tolerance: FloatProperty(
        name="Equal-Length Tolerance", subtype='DISTANCE', default=0.125 * INCH, min=0.0,
        update=_update_header_stats,
        description="How far a pipe's length is allowed to be from the target and still count as \"close enough\" / equal-length",
    )
    active_primary: IntProperty(default=0, min=0)
    actual_primary_count: IntProperty(name="Managed Primaries", default=0, options={'SKIP_SAVE'})
    average_length: FloatProperty(name="Average Length", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    minimum_length: FloatProperty(name="Shortest Primary", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    maximum_length: FloatProperty(name="Longest Primary", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    length_spread: FloatProperty(name="Length Spread", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    all_within_tolerance: BoolProperty(name="All Within Tolerance", default=False, options={'SKIP_SAVE'})


class EXHAUST_PG_Segment(bpy.types.PropertyGroup):
    kind: EnumProperty(
        name="Type",
        description="Whether this piece of the pipe is a straight run or a bend",
        items=[('STRAIGHT', "Straight", "A straight length of tube"), ('BEND', "Bend", "A curved bend in the pipe")],
        default='STRAIGHT', update=_update_route,
    )
    length: FloatProperty(
        name="Length", subtype='DISTANCE', default=6.0 * INCH, min=0.0, update=_update_route,
        description="How long this straight section is",
    )
    bend_style: EnumProperty(
        name="Bend Type",
        description="How this bend is fabricated",
        items=[
            ('MANDREL', "Mandrel", "A smooth, constant-radius bend made on a mandrel bender -- the common, clean-looking exhaust bend"),
            ('PIE_CUT', "Pie-Cut", "A bend built from several straight tube sections welded together at angled (mitered) cuts, like a segmented elbow"),
        ],
        default='MANDREL', update=_update_route,
    )
    radius: FloatProperty(
        name="CLR", subtype='DISTANCE', default=3.0 * INCH, min=0.001, update=_update_route,
        description="Centerline Radius: how tight or gentle this bend is. Smaller = sharper/tighter bend, larger = wider/gentler bend",
    )
    angle: FloatProperty(
        name="Bend Angle", subtype='ANGLE', default=math.radians(45.0), soft_min=-math.pi, soft_max=math.pi, update=_update_route,
        description="How far this bend turns, e.g. 90 degrees for a quarter turn. Negative values turn the opposite direction",
    )
    clocking: FloatProperty(
        name="Clocking", subtype='ANGLE', default=0.0, soft_min=-math.pi, soft_max=math.pi, update=_update_route,
        description="Which direction this bend curves, like a clock face when looking down the pipe. "
                    "Rotating this spins the bend around the pipe's own axis without changing how sharp it is",
    )
    resolution: IntProperty(
        name="Bend Segments", default=16, min=2, max=256, update=_update_route,
        description="How smooth this bend looks. Higher numbers look smoother but add more geometry to the mesh",
    )
    pie_sections: IntProperty(
        name="Pie Sections", default=6, min=2, max=48,
        description="Number of straight mitered tube pieces welded together to make this bend. More sections = a smoother-looking curve but more weld seams",
        update=_update_route,
    )
    show_pie_weld_seams: BoolProperty(
        name="Show Pie Weld Seams", default=True,
        description="Draw viewport-only rings marking where the miter welds go on this pie-cut bend. Just a visual guide -- never part of the actual mesh",
    )


class EXHAUST_PG_Route(bpy.types.PropertyGroup):
    is_route: BoolProperty(default=False)
    outside_diameter: FloatProperty(
        name="Outside Diameter", subtype='DISTANCE', default=3.0 * INCH, min=0.001, update=_update_route,
        description="Outside diameter (thickness) of this pipe",
    )
    wall_thickness: FloatProperty(
        name="Wall Thickness", subtype='DISTANCE', default=0.065 * INCH, min=0.0, update=_update_route,
        description="Thickness of the pipe's metal wall. Doesn't change the outside size -- only the inside opening",
    )
    profile_segments: IntProperty(
        name="Profile Segments", default=32, min=6, max=256, update=_update_route,
        description="How round this pipe's cross-section looks. Higher numbers look smoother/rounder but add more geometry to the mesh",
    )
    show_segment_seams: BoolProperty(
        name="Show Segment Seams",
        description="Draw viewport-only rings at route segment boundaries; guides never render or export",
        default=True,
    )
    seam_guide_offset: FloatProperty(
        name="Seam Guide Offset",
        description="Distance the viewport seam ring sits above the outside pipe surface",
        subtype='DISTANCE',
        default=0.030 * INCH,
        min=0.0,
        soft_max=0.250 * INCH,
    )
    segments: CollectionProperty(type=EXHAUST_PG_Segment)
    active_segment: IntProperty(default=0, min=0, update=_update_active_segment)
    centerline_length: FloatProperty(name="Centerline Length", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})

    # Route end fabrication treatments.  These are part of the same route mesh,
    # not separate fitting objects.  Start and End are independent so a route can
    # be plain on one side and swaged/slip-fit on the other.
    start_end_type: EnumProperty(
        name="Start End",
        description="How the pipe's start (inlet) end is finished",
        items=[
            ('PLAIN', "Plain", "A simple square cut at the pipe's normal size -- no expansion, reduction, or socket"),
            ('EXPANDED', "Expanded / Swaged", "Smoothly flares the end outward to a larger outside diameter"),
            ('REDUCED', "Reduced / Necked", "Smoothly necks the end down to a smaller outside diameter"),
            ('SLIP_SOCKET', "Slip Socket (Female)", "Flares the end just enough for another tube of this pipe's size to slide inside it"),
        ],
        default='PLAIN', update=_update_route,
    )
    end_end_type: EnumProperty(
        name="Finish End",
        description="How the pipe's finish (outlet) end is finished",
        items=[
            ('PLAIN', "Plain", "A simple square cut at the pipe's normal size -- no expansion, reduction, or socket"),
            ('EXPANDED', "Expanded / Swaged", "Smoothly flares the end outward to a larger outside diameter"),
            ('REDUCED', "Reduced / Necked", "Smoothly necks the end down to a smaller outside diameter"),
            ('SLIP_SOCKET', "Slip Socket (Female)", "Flares the end just enough for another tube of this pipe's size to slide inside it"),
        ],
        default='PLAIN', update=_update_route,
    )

    start_expanded_od: FloatProperty(
        name="Expanded OD", subtype='DISTANCE', default=3.5 * INCH, min=0.001, update=_update_route,
        description="Outside diameter the start end flares out to. Should be larger than the pipe's normal Outside Diameter",
    )
    end_expanded_od: FloatProperty(
        name="Expanded OD", subtype='DISTANCE', default=3.5 * INCH, min=0.001, update=_update_route,
        description="Outside diameter the finish end flares out to. Should be larger than the pipe's normal Outside Diameter",
    )
    start_reduced_od: FloatProperty(
        name="Reduced OD", subtype='DISTANCE', default=2.5 * INCH, min=0.001, update=_update_route,
        description="Outside diameter the start end necks down to. Should be smaller than the pipe's normal Outside Diameter",
    )
    end_reduced_od: FloatProperty(
        name="Reduced OD", subtype='DISTANCE', default=2.5 * INCH, min=0.001, update=_update_route,
        description="Outside diameter the finish end necks down to. Should be smaller than the pipe's normal Outside Diameter",
    )

    start_transition_length: FloatProperty(
        name="Transition Length", subtype='DISTANCE', default=0.75 * INCH, min=0.0, update=_update_route,
        description="How long the tapered section is where the start end flares/necks to its new size. Longer = a gentler, gradual taper",
    )
    end_transition_length: FloatProperty(
        name="Transition Length", subtype='DISTANCE', default=0.75 * INCH, min=0.0, update=_update_route,
        description="How long the tapered section is where the finish end flares/necks to its new size. Longer = a gentler, gradual taper",
    )
    start_collar_length: FloatProperty(
        name="Straight End Length", subtype='DISTANCE', default=1.5 * INCH, min=0.0, update=_update_route,
        description="Length of straight pipe left at the start end's new size, after the taper -- gives a socket or hose clamp somewhere flat to grip",
    )
    end_collar_length: FloatProperty(
        name="Straight End Length", subtype='DISTANCE', default=1.5 * INCH, min=0.0, update=_update_route,
        description="Length of straight pipe left at the finish end's new size, after the taper -- gives a socket or hose clamp somewhere flat to grip",
    )
    start_slip_clearance: FloatProperty(
        name="Diametral Clearance", subtype='DISTANCE', default=0.020 * INCH, min=0.0, update=_update_route,
        description="Extra inside-diameter room added beyond the mating tube's outside size, so it can actually slide into this socket",
    )
    end_slip_clearance: FloatProperty(
        name="Diametral Clearance", subtype='DISTANCE', default=0.020 * INCH, min=0.0, update=_update_route,
        description="Extra inside-diameter room added beyond the mating tube's outside size, so it can actually slide into this socket",
    )
    end_treatment_segments: IntProperty(
        name="End Transition Segments", default=8, min=2, max=64, update=_update_route,
        description="How smooth the expanded/reduced/slip-socket tapers look along the pipe's length. Higher = smoother but more geometry",
    )

    # Optional endpoint connection hardware.  Hardware is generated directly on
    # the exposed route end after any swage/reducer/slip treatment.
    start_connection_type: EnumProperty(
        name="Start Connection",
        description="What kind of bolt-together or clamp-together hardware to add at the pipe's start end",
        items=[
            ('NONE', "None", "No connection hardware -- just the plain/expanded/reduced/socket pipe end"),
            ('VBAND', "V-Band Flange", "A round clamp-style flange (two matching halves held together by an outer V-band clamp) -- quick to assemble/disassemble"),
            ('WELD_FLANGE', "Round Weld Flange", "A simple round weld-on flange plate with no bolt holes modeled"),
            ('FLAT_2BOLT', "2-Bolt Flat Flange", "The common automotive oval flange with two bolt holes, one on each side"),
            ('FLAT_3BOLT', "3-Bolt Flat Flange", "A rounded-triangle automotive flange with three bolt holes"),
        ],
        default='NONE', update=_update_route,
    )
    end_connection_type: EnumProperty(
        name="Finish Connection",
        description="What kind of bolt-together or clamp-together hardware to add at the pipe's finish end",
        items=[
            ('NONE', "None", "No connection hardware -- just the plain/expanded/reduced/socket pipe end"),
            ('VBAND', "V-Band Flange", "A round clamp-style flange (two matching halves held together by an outer V-band clamp) -- quick to assemble/disassemble"),
            ('WELD_FLANGE', "Round Weld Flange", "A simple round weld-on flange plate with no bolt holes modeled"),
            ('FLAT_2BOLT', "2-Bolt Flat Flange", "The common automotive oval flange with two bolt holes, one on each side"),
            ('FLAT_3BOLT', "3-Bolt Flat Flange", "A rounded-triangle automotive flange with three bolt holes"),
        ],
        default='NONE', update=_update_route,
    )

    start_flange_od: FloatProperty(
        name="Flange OD", subtype='DISTANCE', default=4.0 * INCH, min=0.001, update=_update_route,
        description="Outside diameter of the round flange or clamp face at the start end",
    )
    end_flange_od: FloatProperty(
        name="Flange OD", subtype='DISTANCE', default=4.0 * INCH, min=0.001, update=_update_route,
        description="Outside diameter of the round flange or clamp face at the finish end",
    )
    start_flange_thickness: FloatProperty(
        name="Flange Thickness", subtype='DISTANCE', default=0.375 * INCH, min=0.001, update=_update_route,
        description="How thick the flange plate at the start end is",
    )
    end_flange_thickness: FloatProperty(
        name="Flange Thickness", subtype='DISTANCE', default=0.375 * INCH, min=0.001, update=_update_route,
        description="How thick the flange plate at the finish end is",
    )
    start_bolt_circle: FloatProperty(
        name="Bolt Circle Diameter", subtype='DISTANCE', default=3.25 * INCH, min=0.001, update=_update_route,
        description="Diameter of the circle the bolt holes sit on, at the start end",
    )
    end_bolt_circle: FloatProperty(
        name="Bolt Circle Diameter", subtype='DISTANCE', default=3.25 * INCH, min=0.001, update=_update_route,
        description="Diameter of the circle the bolt holes sit on, at the finish end",
    )
    start_bolt_hole_diameter: FloatProperty(
        name="Bolt Hole Diameter", subtype='DISTANCE', default=0.4375 * INCH, min=0.001, update=_update_route,
        description="Diameter of each bolt hole at the start end (should match your bolt size)",
    )
    end_bolt_hole_diameter: FloatProperty(
        name="Bolt Hole Diameter", subtype='DISTANCE', default=0.4375 * INCH, min=0.001, update=_update_route,
        description="Diameter of each bolt hole at the finish end (should match your bolt size)",
    )
    start_bolt_phase: FloatProperty(
        name="Bolt Pattern Rotation", subtype='ANGLE', default=0.0, update=_update_route,
        description="Rotates the bolt-hole pattern around the pipe at the start end, without moving the flange itself",
    )
    end_bolt_phase: FloatProperty(
        name="Bolt Pattern Rotation", subtype='ANGLE', default=0.0, update=_update_route,
        description="Rotates the bolt-hole pattern around the pipe at the finish end, without moving the flange itself",
    )

    # Automotive flat-flange outline dimensions.  These follow the common A/B/C
    # notation used by exhaust-flange suppliers rather than pretending the
    # plate has one circular OD.
    start_2bolt_height: FloatProperty(
        name="A - Overall Height", subtype='DISTANCE', default=3.75 * INCH, min=0.001, update=_update_route,
        description="Overall height (the \"A\" dimension on a supplier's flange drawing) of the 2-bolt flange plate at the start end",
    )
    end_2bolt_height: FloatProperty(
        name="A - Overall Height", subtype='DISTANCE', default=3.75 * INCH, min=0.001, update=_update_route,
        description="Overall height (the \"A\" dimension on a supplier's flange drawing) of the 2-bolt flange plate at the finish end",
    )
    start_2bolt_width: FloatProperty(
        name="B - Overall Width", subtype='DISTANCE', default=5.20 * INCH, min=0.001, update=_update_route,
        description="Overall width (the \"B\" dimension) of the 2-bolt flange plate at the start end",
    )
    end_2bolt_width: FloatProperty(
        name="B - Overall Width", subtype='DISTANCE', default=5.20 * INCH, min=0.001, update=_update_route,
        description="Overall width (the \"B\" dimension) of the 2-bolt flange plate at the finish end",
    )
    start_2bolt_spacing: FloatProperty(
        name="C - Bolt Center Spacing", subtype='DISTANCE', default=4.50 * INCH, min=0.001, update=_update_route,
        description="Distance between the two bolt hole centers (the \"C\" dimension) at the start end",
    )
    end_2bolt_spacing: FloatProperty(
        name="C - Bolt Center Spacing", subtype='DISTANCE', default=4.50 * INCH, min=0.001, update=_update_route,
        description="Distance between the two bolt hole centers (the \"C\" dimension) at the finish end",
    )

    start_3bolt_height: FloatProperty(
        name="A - Overall Height", subtype='DISTANCE', default=4.40 * INCH, min=0.001, update=_update_route,
        description="Overall height (the \"A\" dimension) of the 3-bolt flange plate at the start end",
    )
    end_3bolt_height: FloatProperty(
        name="A - Overall Height", subtype='DISTANCE', default=4.40 * INCH, min=0.001, update=_update_route,
        description="Overall height (the \"A\" dimension) of the 3-bolt flange plate at the finish end",
    )
    start_3bolt_width: FloatProperty(
        name="B - Overall Width", subtype='DISTANCE', default=4.40 * INCH, min=0.001, update=_update_route,
        description="Overall width (the \"B\" dimension) of the 3-bolt flange plate at the start end",
    )
    end_3bolt_width: FloatProperty(
        name="B - Overall Width", subtype='DISTANCE', default=4.40 * INCH, min=0.001, update=_update_route,
        description="Overall width (the \"B\" dimension) of the 3-bolt flange plate at the finish end",
    )

    start_vband_neck_length: FloatProperty(
        name="Weld Neck Length", subtype='DISTANCE', default=0.50 * INCH, min=0.0, update=_update_route,
        description="Length of plain pipe left between the tube end and where the V-band clamp lip begins, at the start end",
    )
    end_vband_neck_length: FloatProperty(
        name="Weld Neck Length", subtype='DISTANCE', default=0.50 * INCH, min=0.0, update=_update_route,
        description="Length of plain pipe left between the tube end and where the V-band clamp lip begins, at the finish end",
    )
    start_vband_taper_length: FloatProperty(
        name="V-Band Taper Length", subtype='DISTANCE', default=0.16 * INCH, min=0.001, update=_update_route,
        description="Length of the tapered lip the V-band clamp grips onto, at the start end",
    )
    end_vband_taper_length: FloatProperty(
        name="V-Band Taper Length", subtype='DISTANCE', default=0.16 * INCH, min=0.001, update=_update_route,
        description="Length of the tapered lip the V-band clamp grips onto, at the finish end",
    )
    start_vband_face_width: FloatProperty(
        name="V-Band Face Width", subtype='DISTANCE', default=0.22 * INCH, min=0.001, update=_update_route,
        description="Width of the flat clamping face at the very tip of the V-band lip, at the start end",
    )
    end_vband_face_width: FloatProperty(
        name="V-Band Face Width", subtype='DISTANCE', default=0.22 * INCH, min=0.001, update=_update_route,
        description="Width of the flat clamping face at the very tip of the V-band lip, at the finish end",
    )
    hardware_profile_segments: IntProperty(
        name="Hardware Profile Segments", default=64, min=16, max=256, update=_update_route,
        description="How round the flange/V-band hardware and its bolt holes look. Higher = smoother but more geometry",
    )


class EXHAUST_PG_Collector(bpy.types.PropertyGroup):
    is_collector: BoolProperty(default=False)
    primary_count: IntProperty(
        name="Primary Count", default=4, min=2, max=12, update=_update_collector,
        description="How many individual primary pipes merge into this collector -- should match the Header's primary count",
    )
    primary_od: FloatProperty(
        name="Primary OD", subtype='DISTANCE', default=1.75 * INCH, min=0.001, update=_update_collector,
        description="Outside diameter each incoming primary pipe is expected to be, at the point it enters the collector",
    )
    outlet_od: FloatProperty(
        name="Outlet OD", subtype='DISTANCE', default=3.0 * INCH, min=0.001, update=_update_collector,
        description="Outside diameter of the single pipe that exits the collector after all the primaries merge",
    )
    wall_thickness: FloatProperty(
        name="Wall Thickness", subtype='DISTANCE', default=0.065 * INCH, min=0.0, update=_update_collector,
        description="Thickness of the collector's metal wall",
    )
    collector_length: FloatProperty(
        name="Collector Length", subtype='DISTANCE', default=8.0 * INCH, min=0.001, update=_update_collector,
        description="Overall length of the collector body, from where the primaries enter to where the outlet begins",
    )
    transition_length: FloatProperty(
        name="Smooth Transition Length", subtype='DISTANCE', default=3.0 * INCH, min=0.001, update=_update_collector,
        description="How much of the collector's length is used to smoothly merge the primaries into one shape before the round outlet begins",
    )
    merge_lobe_strength: FloatProperty(
        name="Merge Lobe Strength",
        description="How much each primary's own rounded shape still shows through right where they first merge together, "
                    "before blending into one smooth body. Higher keeps the individual pipe shapes more visible for longer",
        default=0.72, min=0.05, max=0.95, update=_update_collector,
    )
    outlet_length: FloatProperty(
        name="Outlet Length", subtype='DISTANCE', default=3.0 * INCH, min=0.0, update=_update_collector,
        description="Length of straight round pipe at the collector's outlet, after the merged shape has become fully round",
    )
    auto_radial_spread: BoolProperty(
        name="Auto-Pack Inlets", default=True, update=_update_collector,
        description="Automatically space the primary inlets as tightly as possible around the collector, based on their size and the gap below. "
                    "Turn off to set the spacing by hand instead",
    )
    radial_gap: FloatProperty(
        name="Inlet Gap", subtype='DISTANCE', default=0.125 * INCH, min=0.0, update=_update_collector,
        description="Minimum gap left between neighboring primary pipes where they enter the collector, used while Auto-Pack Inlets is on",
    )
    radial_spread: FloatProperty(
        name="Radial Spread", subtype='DISTANCE', default=2.0 * INCH, min=0.0, update=_update_collector,
        description="Distance from the collector's center axis to each primary inlet's center, set by hand when Auto-Pack Inlets is off",
    )
    radial_phase: FloatProperty(
        name="Radial Phase", subtype='ANGLE', default=0.0, update=_update_collector,
        description="Rotates the whole ring of primary inlets around the collector's axis, without changing their spacing",
    )
    computed_radial_spread: FloatProperty(name="Computed Spread", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    branch_samples: IntProperty(
        name="Branch Segments", default=24, min=4, max=256, update=_update_collector,
        description="How smooth each primary's own curve looks as it enters the collector. Higher = smoother but more geometry",
    )
    transition_segments: IntProperty(
        name="Transition Segments", default=20, min=4, max=256, update=_update_collector,
        description="How smooth the merge from separate primaries into one round body looks along the collector's length. Higher = smoother but more geometry",
    )
    profile_segments: IntProperty(
        name="Profile Segments", default=48, min=12, max=256, update=_update_collector,
        description="How round the collector's cross-section looks. Higher = smoother/rounder but more geometry",
    )
    computed_body_start: FloatProperty(name="Computed Body Start", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    computed_join_radius: FloatProperty(name="Computed Join Radius", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})

    # Collector outlet fabrication. Header primaries terminate directly at the
    # collector inlets; end treatments and connection hardware therefore belong
    # on the collector's single downstream outlet rather than on each primary.
    outlet_end_type: EnumProperty(
        name="Outlet End",
        description="How the collector's downstream outlet end is finished",
        items=[
            ('PLAIN', "Plain", "A simple square cut at the outlet's normal size"),
            ('EXPANDED', "Expanded / Swaged", "Smoothly flares the outlet outward to a larger outside diameter"),
            ('REDUCED', "Reduced / Necked", "Smoothly necks the outlet down to a smaller outside diameter"),
            ('SLIP_SOCKET', "Slip Socket (Female)", "Flares the outlet just enough for another tube to slide inside it"),
        ],
        default='PLAIN', update=_update_collector,
    )
    outlet_expanded_od: FloatProperty(
        name="Expanded OD", subtype='DISTANCE', default=3.5 * INCH, min=0.001, update=_update_collector,
        description="Outside diameter the outlet flares out to. Should be larger than the outlet's normal Outlet OD",
    )
    outlet_reduced_od: FloatProperty(
        name="Reduced OD", subtype='DISTANCE', default=2.5 * INCH, min=0.001, update=_update_collector,
        description="Outside diameter the outlet necks down to. Should be smaller than the outlet's normal Outlet OD",
    )
    outlet_transition_length: FloatProperty(
        name="Transition Length", subtype='DISTANCE', default=0.75 * INCH, min=0.0, update=_update_collector,
        description="How long the tapered section is where the outlet flares/necks to its new size. Longer = a gentler, gradual taper",
    )
    outlet_collar_length: FloatProperty(
        name="Straight End Length", subtype='DISTANCE', default=1.5 * INCH, min=0.0, update=_update_collector,
        description="Length of straight pipe left at the outlet's new size, after the taper -- gives a socket or hose clamp somewhere flat to grip",
    )
    outlet_slip_clearance: FloatProperty(
        name="Diametral Clearance", subtype='DISTANCE', default=0.020 * INCH, min=0.0, update=_update_collector,
        description="Extra inside-diameter room added beyond the mating tube's outside size, so it can actually slide into this socket",
    )
    outlet_treatment_segments: IntProperty(
        name="Outlet Transition Segments", default=8, min=2, max=64, update=_update_collector,
        description="How smooth the expanded/reduced/slip-socket taper looks along the outlet's length. Higher = smoother but more geometry",
    )

    outlet_connection_type: EnumProperty(
        name="Outlet Connection",
        description="What kind of bolt-together or clamp-together hardware to add at the collector's outlet",
        items=[
            ('NONE', "None", "No connection hardware -- just the plain/expanded/reduced/socket outlet end"),
            ('VBAND', "V-Band Flange", "A round clamp-style flange (two matching halves held together by an outer V-band clamp) -- quick to assemble/disassemble"),
            ('WELD_FLANGE', "Round Weld Flange", "A simple round weld-on flange plate with no bolt holes modeled"),
            ('FLAT_2BOLT', "2-Bolt Flat Flange", "The common automotive oval flange with two bolt holes, one on each side"),
            ('FLAT_3BOLT', "3-Bolt Flat Flange", "A rounded-triangle automotive flange with three bolt holes"),
        ],
        default='NONE', update=_update_collector,
    )
    outlet_flange_od: FloatProperty(
        name="Flange OD", subtype='DISTANCE', default=4.0 * INCH, min=0.001, update=_update_collector,
        description="Outside diameter of the round flange or clamp face at the outlet",
    )
    outlet_flange_thickness: FloatProperty(
        name="Flange Thickness", subtype='DISTANCE', default=0.375 * INCH, min=0.001, update=_update_collector,
        description="How thick the flange plate at the outlet is",
    )
    outlet_bolt_hole_diameter: FloatProperty(
        name="Bolt Hole Diameter", subtype='DISTANCE', default=0.4375 * INCH, min=0.001, update=_update_collector,
        description="Diameter of each bolt hole at the outlet (should match your bolt size)",
    )
    outlet_bolt_phase: FloatProperty(
        name="Bolt Pattern Rotation", subtype='ANGLE', default=0.0, update=_update_collector,
        description="Rotates the bolt-hole pattern around the outlet, without moving the flange itself",
    )
    outlet_2bolt_height: FloatProperty(
        name="A - Overall Height", subtype='DISTANCE', default=3.75 * INCH, min=0.001, update=_update_collector,
        description="Overall height (the \"A\" dimension on a supplier's flange drawing) of the 2-bolt outlet flange",
    )
    outlet_2bolt_width: FloatProperty(
        name="B - Overall Width", subtype='DISTANCE', default=5.20 * INCH, min=0.001, update=_update_collector,
        description="Overall width (the \"B\" dimension) of the 2-bolt outlet flange",
    )
    outlet_2bolt_spacing: FloatProperty(
        name="C - Bolt Center Spacing", subtype='DISTANCE', default=4.50 * INCH, min=0.001, update=_update_collector,
        description="Distance between the two bolt hole centers (the \"C\" dimension) on the outlet flange",
    )
    outlet_3bolt_height: FloatProperty(
        name="A - Overall Height", subtype='DISTANCE', default=4.40 * INCH, min=0.001, update=_update_collector,
        description="Overall height (the \"A\" dimension) of the 3-bolt outlet flange",
    )
    outlet_3bolt_width: FloatProperty(
        name="B - Overall Width", subtype='DISTANCE', default=4.40 * INCH, min=0.001, update=_update_collector,
        description="Overall width (the \"B\" dimension) of the 3-bolt outlet flange",
    )
    outlet_vband_neck_length: FloatProperty(
        name="Weld Neck Length", subtype='DISTANCE', default=0.50 * INCH, min=0.0, update=_update_collector,
        description="Length of plain pipe left between the outlet end and where the V-band clamp lip begins",
    )
    outlet_vband_taper_length: FloatProperty(
        name="V-Band Taper Length", subtype='DISTANCE', default=0.16 * INCH, min=0.001, update=_update_collector,
        description="Length of the tapered lip the V-band clamp grips onto",
    )
    outlet_vband_face_width: FloatProperty(
        name="V-Band Face Width", subtype='DISTANCE', default=0.22 * INCH, min=0.001, update=_update_collector,
        description="Width of the flat clamping face at the very tip of the V-band lip",
    )
    outlet_hardware_profile_segments: IntProperty(
        name="Hardware Profile Segments", default=64, min=16, max=256, update=_update_collector,
        description="How round the outlet flange/V-band hardware and its bolt holes look. Higher = smoother but more geometry",
    )


class EXHAUST_PG_YPipe(bpy.types.PropertyGroup):
    is_y_pipe: BoolProperty(default=False)
    topology: EnumProperty(
        name="Topology",
        description="Automotive Y-pipe construction / routing family",
        items=[
            ('SWEPT', "Swept Y", "Validated smooth curved-branch Y-pipe baseline"),
            ('CLASSIC', "Classic / Straight-Leg", "Straighter inlet legs with a compact terminal sweep into the merge"),
            ('PARALLEL', "Parallel Merge", "Parallel inlet tangents that sweep together into a long divider-style merge"),
            ('TANGENT', "Tangent / Side Entry", "One dominant run with the second branch entering from the side"),
            ('FORMED', "Formed / Organic", "Broad continuously formed Y with softer branch and merge curvature"),
            ('CUSTOM', "Custom", "Independent angles, inlet spacing, outlet bias, and straightness"),
        ],
        default='SWEPT', update=_update_y_pipe,
    )
    symmetric: BoolProperty(
        name="Symmetric",
        description="Use identical inlet diameters and merge angles on both branches",
        default=True, update=_update_y_pipe,
    )
    inlet_a_od: FloatProperty(
        name="Inlet A OD", subtype='DISTANCE', default=2.5 * INCH, min=0.001, update=_update_y_pipe,
        description="Outside diameter of the first (A) inlet pipe",
    )
    inlet_b_od: FloatProperty(
        name="Inlet B OD", subtype='DISTANCE', default=2.5 * INCH, min=0.001, update=_update_y_pipe,
        description="Outside diameter of the second (B) inlet pipe",
    )
    outlet_od: FloatProperty(
        name="Outlet OD", subtype='DISTANCE', default=3.0 * INCH, min=0.001, update=_update_y_pipe,
        description="Outside diameter of the single pipe that exits after A and B merge",
    )
    wall_thickness: FloatProperty(
        name="Wall Thickness", subtype='DISTANCE', default=0.065 * INCH, min=0.0, update=_update_y_pipe,
        description="Thickness of the Y-pipe's metal wall",
    )
    inlet_a_angle: FloatProperty(
        name="Inlet A Merge Angle", subtype='ANGLE', default=math.radians(15.0),
        min=0.0, soft_max=math.radians(60.0), update=_update_y_pipe,
        description="How sharply inlet A angles in before merging. Larger angles merge more abruptly",
    )
    inlet_b_angle: FloatProperty(
        name="Inlet B Merge Angle", subtype='ANGLE', default=math.radians(15.0),
        min=0.0, soft_max=math.radians(60.0), update=_update_y_pipe,
        description="How sharply inlet B angles in before merging. Larger angles merge more abruptly",
    )
    branch_length: FloatProperty(
        name="Inlet Branch Length", subtype='DISTANCE', default=6.0 * INCH, min=0.001, update=_update_y_pipe,
        description="Length of each inlet branch before it starts merging with the other",
    )
    merge_length: FloatProperty(
        name="Merge Transition Length", subtype='DISTANCE', default=6.0 * INCH, min=0.001, update=_update_y_pipe,
        description="Length of the section where the two inlets blend together into one shape",
    )
    outlet_length: FloatProperty(
        name="Outlet Length", subtype='DISTANCE', default=4.0 * INCH, min=0.0, update=_update_y_pipe,
        description="Length of straight round pipe after the merge is complete",
    )
    branch_samples: IntProperty(
        name="Branch Segments", default=24, min=4, max=256, update=_update_y_pipe,
        description="How smooth each inlet branch's curve looks. Higher = smoother but more geometry",
    )
    transition_segments: IntProperty(
        name="Transition Segments", default=24, min=6, max=256, update=_update_y_pipe,
        description="How smooth the merge from two inlets into one shape looks along its length. Higher = smoother but more geometry",
    )
    profile_segments: IntProperty(
        name="Profile Segments", default=48, min=12, max=256, update=_update_y_pipe,
        description="How round the pipe's cross-section looks. Higher = smoother/rounder but more geometry",
    )
    inlet_spacing: FloatProperty(
        name="Inlet Center Spacing", subtype='DISTANCE', default=5.5 * INCH, min=0.001,
        description="Center-to-center inlet spacing used by Parallel Merge and Custom modes",
        update=_update_y_pipe,
    )
    main_branch: EnumProperty(
        name="Main Run",
        description="In Tangent / Side Entry mode, which inlet is the dominant, straight-through pipe (the other one enters from the side)",
        items=[('A', "Branch A", "Branch A runs straight through; Branch B enters from the side"), ('B', "Branch B", "Branch B runs straight through; Branch A enters from the side")],
        default='A', update=_update_y_pipe,
    )
    outlet_bias: FloatProperty(
        name="Outlet Bias", default=0.75, min=0.0, max=1.0,
        description="In Tangent mode, shifts the common outlet toward the dominant run; 0 is centered and 1 follows the main-run centerline",
        update=_update_y_pipe,
    )
    custom_outlet_bias: FloatProperty(
        name="Custom Outlet Bias", default=0.0, min=-1.0, max=1.0,
        description="Signed custom outlet-center bias: positive toward A, negative toward B",
        update=_update_y_pipe,
    )
    straight_fraction: FloatProperty(
        name="Straight-Leg Fraction", default=0.70, min=0.0, max=0.90,
        description="Fraction of the inlet branch held close to its entry tangent before the final sweep",
        update=_update_y_pipe,
    )
    computed_inlet_spacing: FloatProperty(name="Computed Inlet Spacing", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    computed_outlet_offset: FloatProperty(name="Computed Outlet Offset", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})


class EXHAUST_PG_XPipe(bpy.types.PropertyGroup):
    is_x_pipe: BoolProperty(default=False)
    topology: EnumProperty(
        name="Topology",
        description="Overall automotive X-pipe crossover style",
        items=[
            ('CLASSIC', "Classic X", "Compact conventional X crossover"),
            ('SWEPT', "Swept X", "Longer, broader communicating crossover"),
            ('PARALLEL', "Parallel Entry", "Parallel inlet/outlet tangents with a smooth central crossover"),
            ('CUSTOM', "Custom", "Independent branch angles and crossover profile control"),
        ],
        default='CLASSIC', update=_update_x_pipe,
    )
    symmetric: BoolProperty(
        name="Symmetric",
        description="Use identical A/B tube diameters and mirrored branch angles",
        default=True, update=_update_x_pipe,
    )
    pipe_a_od: FloatProperty(
        name="Pipe A OD", subtype='DISTANCE', default=2.5 * INCH, min=0.001, update=_update_x_pipe,
        description="Outside diameter of pipe A, running through the crossover",
    )
    pipe_b_od: FloatProperty(
        name="Pipe B OD", subtype='DISTANCE', default=2.5 * INCH, min=0.001, update=_update_x_pipe,
        description="Outside diameter of pipe B, running through the crossover",
    )
    wall_thickness: FloatProperty(
        name="Wall Thickness", subtype='DISTANCE', default=0.065 * INCH, min=0.0, update=_update_x_pipe,
        description="Thickness of the X-pipe's metal wall",
    )

    inlet_spacing: FloatProperty(
        name="Inlet Center Spacing", subtype='DISTANCE', default=4.0 * INCH, min=0.001, update=_update_x_pipe,
        description="Center-to-center distance between the two inlet pipes",
    )
    outlet_spacing: FloatProperty(
        name="Outlet Center Spacing", subtype='DISTANCE', default=4.0 * INCH, min=0.001, update=_update_x_pipe,
        description="Center-to-center distance between the two outlet pipes",
    )

    inlet_a_angle: FloatProperty(
        name="Inlet A Angle", subtype='ANGLE', default=math.radians(12.0), min=0.0, soft_max=math.radians(60.0), update=_update_x_pipe,
        description="How sharply pipe A angles as it approaches the crossover",
    )
    inlet_b_angle: FloatProperty(
        name="Inlet B Angle", subtype='ANGLE', default=math.radians(12.0), min=0.0, soft_max=math.radians(60.0), update=_update_x_pipe,
        description="How sharply pipe B angles as it approaches the crossover",
    )
    outlet_a_angle: FloatProperty(
        name="Outlet A Angle", subtype='ANGLE', default=math.radians(12.0), min=0.0, soft_max=math.radians(60.0), update=_update_x_pipe,
        description="How sharply pipe A angles as it departs the crossover",
    )
    outlet_b_angle: FloatProperty(
        name="Outlet B Angle", subtype='ANGLE', default=math.radians(12.0), min=0.0, soft_max=math.radians(60.0), update=_update_x_pipe,
        description="How sharply pipe B angles as it departs the crossover",
    )

    inlet_branch_length: FloatProperty(
        name="Inlet Branch Length", subtype='DISTANCE', default=6.0 * INCH, min=0.001, update=_update_x_pipe,
        description="Length of straight pipe on the inlet side before the crossover begins",
    )
    crossover_length: FloatProperty(
        name="Crossover Length", subtype='DISTANCE', default=8.0 * INCH, min=0.001, update=_update_x_pipe,
        description="Length of the central section where pipes A and B cross and communicate with each other",
    )
    outlet_branch_length: FloatProperty(
        name="Outlet Branch Length", subtype='DISTANCE', default=6.0 * INCH, min=0.001, update=_update_x_pipe,
        description="Length of straight pipe on the outlet side after the crossover ends",
    )
    crossover_opening: FloatProperty(
        name="Crossover Opening",
        description="How deeply the two internal flow passages communicate through the center of the X. "
                    "Higher lets more exhaust flow cross between the two pipes",
        default=0.55, min=0.05, max=1.0, update=_update_x_pipe,
    )
    custom_profile: FloatProperty(
        name="Crossover Profile",
        description="Shape of the crossover opening in Custom mode: lower values create a broader merge, higher values a tighter, more pinched center",
        default=1.25, min=0.35, max=3.0, update=_update_x_pipe,
    )
    plane_rotation: FloatProperty(
        name="Plane Rotation", subtype='ANGLE',
        description="Rotates the whole X-pipe crossover around its lengthwise axis, like turning a steering wheel",
        default=0.0, update=_update_x_pipe,
    )

    branch_samples: IntProperty(
        name="Branch Segments", default=24, min=4, max=256, update=_update_x_pipe,
        description="How smooth each inlet/outlet branch's curve looks. Higher = smoother but more geometry",
    )
    crossover_segments: IntProperty(
        name="Crossover Segments", default=36, min=10, max=384, update=_update_x_pipe,
        description="How smooth the central crossover section looks along its length. Higher = smoother but more geometry",
    )
    profile_segments: IntProperty(
        name="Profile Segments", default=48, min=12, max=256, update=_update_x_pipe,
        description="How round the pipe's cross-section looks. Higher = smoother/rounder but more geometry",
    )
    computed_inlet_spacing: FloatProperty(name="Effective Inlet Spacing", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    computed_outlet_spacing: FloatProperty(name="Effective Outlet Spacing", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})


class EXHAUST_PG_HPipe(bpy.types.PropertyGroup):
    is_h_pipe: BoolProperty(default=False)
    symmetric: BoolProperty(
        name="Symmetric Main Pipes",
        description="Use the same outside diameter for both main exhaust pipes",
        default=True, update=_update_h_pipe,
    )
    pipe_a_od: FloatProperty(
        name="Pipe A OD", subtype='DISTANCE', default=2.5 * INCH, min=0.001, update=_update_h_pipe,
        description="Outside diameter of the first main exhaust pipe",
    )
    pipe_b_od: FloatProperty(
        name="Pipe B OD", subtype='DISTANCE', default=2.5 * INCH, min=0.001, update=_update_h_pipe,
        description="Outside diameter of the second main exhaust pipe (only used when Symmetric Main Pipes is off)",
    )
    wall_thickness: FloatProperty(
        name="Wall Thickness", subtype='DISTANCE', default=0.065 * INCH, min=0.0, update=_update_h_pipe,
        description="Thickness of the main pipes' and crossover's metal wall",
    )
    main_length: FloatProperty(
        name="Main Pipe Length", subtype='DISTANCE', default=24.0 * INCH, min=0.01, update=_update_h_pipe,
        description="Length of each of the two parallel main exhaust pipes",
    )
    main_spacing: FloatProperty(
        name="Main Center Spacing", subtype='DISTANCE', default=6.0 * INCH, min=0.001,
        description="Center-to-center spacing between the two parallel main pipes", update=_update_h_pipe,
    )
    crossover_od: FloatProperty(
        name="Crossover OD", subtype='DISTANCE', default=2.0 * INCH, min=0.001, update=_update_h_pipe,
        description="Outside diameter of the small balance tube connecting the two main pipes",
    )
    crossover_position: FloatProperty(
        name="Crossover Position", subtype='DISTANCE', default=12.0 * INCH, min=0.0,
        description="Nominal longitudinal center position of the H crossover", update=_update_h_pipe,
    )
    crossover_angle: FloatProperty(
        name="Crossover Angle", subtype='ANGLE', default=0.0,
        soft_min=math.radians(-45.0), soft_max=math.radians(45.0),
        description="Signed angle away from a perpendicular H crossover", update=_update_h_pipe,
    )
    junction_blend_length: FloatProperty(
        name="Junction Blend Length", subtype='DISTANCE', default=0.75 * INCH, min=0.0,
        description="Length used to transition each cylindrical saddle opening into the round crossover tube", update=_update_h_pipe,
    )
    plane_rotation: FloatProperty(
        name="Plane Rotation", subtype='ANGLE', default=0.0,
        description="Clock the two-main-pipe/crossover plane around the longitudinal axis", update=_update_h_pipe,
    )
    main_segments: IntProperty(
        name="Main Longitudinal Segments", default=28, min=6, max=256, update=_update_h_pipe,
        description="How many segments make up each main pipe's length, including the saddle cutout for the crossover. Higher = smoother saddle shape",
    )
    junction_segments: IntProperty(
        name="Junction Segments", default=5, min=1, max=32, update=_update_h_pipe,
        description="How smooth the blend looks where the crossover tube meets each main pipe. Higher = smoother but more geometry",
    )
    profile_segments: IntProperty(
        name="Profile Segments", default=48, min=16, max=256, update=_update_h_pipe,
        description="How round the pipes' cross-sections look. Higher = smoother/rounder but more geometry",
    )
    computed_crossover_length: FloatProperty(name="Clear Crossover Length", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    computed_tap_a: FloatProperty(name="Tap A Position", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    computed_tap_b: FloatProperty(name="Tap B Position", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    computed_effective_angle: FloatProperty(name="Effective Angle", subtype='ANGLE', default=0.0, options={'SKIP_SAVE'})


class EXHAUST_PG_PieCut(bpy.types.PropertyGroup):
    is_pie_cut: BoolProperty(default=False)
    outside_diameter: FloatProperty(
        name="Outside Diameter", subtype='DISTANCE', default=3.0 * INCH, min=0.001, update=_update_pie_cut,
        description="Outside diameter (thickness) of this pipe",
    )
    wall_thickness: FloatProperty(
        name="Wall Thickness", subtype='DISTANCE', default=0.065 * INCH, min=0.0, update=_update_pie_cut,
        description="Thickness of the pipe's metal wall",
    )
    bend_angle: FloatProperty(
        name="Total Bend Angle", subtype='ANGLE', default=math.radians(90.0),
        min=math.radians(1.0), soft_max=math.radians(180.0), update=_update_pie_cut,
        description="How far this whole bend turns overall, e.g. 90 degrees for a quarter turn",
    )
    pie_sections: IntProperty(
        name="Pie Sections", default=6, min=2, max=48,
        description="Number of straight mitered tube pieces used to create the bend", update=_update_pie_cut,
    )
    equivalent_clr: FloatProperty(
        name="Equivalent CLR", subtype='DISTANCE', default=4.5 * INCH, min=0.001,
        description="Radius used to size the polygonal centerline; each section is a straight chord", update=_update_pie_cut,
    )
    clocking: FloatProperty(
        name="Clocking", subtype='ANGLE', default=0.0,
        description="Rotate the pie-cut bend plane around the inlet axis", update=_update_pie_cut,
    )
    profile_segments: IntProperty(
        name="Profile Segments", default=48, min=12, max=256, update=_update_pie_cut,
        description="How round the pipe's cross-section looks. Higher = smoother/rounder but more geometry",
    )
    show_weld_seams: BoolProperty(
        name="Show Weld Seams", default=True,
        description="Draw viewport-only miter weld seams; guides never render or export",
    )
    seam_guide_offset: FloatProperty(
        name="Seam Guide Offset", subtype='DISTANCE', default=0.020 * INCH, min=0.0, soft_max=0.250 * INCH,
        description="Offset the viewport weld guide slightly above the tube surface",
    )
    computed_weld_angle: FloatProperty(name="Angle Per Weld", subtype='ANGLE', default=0.0, options={'SKIP_SAVE'})
    computed_section_length: FloatProperty(name="Section Centerline Length", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})
    computed_centerline_length: FloatProperty(name="Centerline Length", subtype='DISTANCE', default=0.0, options={'SKIP_SAVE'})

class EXHAUST_PG_Reducer(bpy.types.PropertyGroup):
    is_reducer: BoolProperty(default=False)
    inlet_od: FloatProperty(
        name="Inlet OD", subtype='DISTANCE', default=2.5 * INCH, min=0.001, update=_update_reducer,
        description="Outside diameter at the reducer's inlet end",
    )
    outlet_od: FloatProperty(
        name="Outlet OD", subtype='DISTANCE', default=3.0 * INCH, min=0.001, update=_update_reducer,
        description="Outside diameter at the reducer's outlet end. Larger than Inlet OD makes an expander, smaller makes a reducer",
    )
    length: FloatProperty(
        name="Length", subtype='DISTANCE', default=4.0 * INCH, min=0.001, update=_update_reducer,
        description="How long the tapered transition is between the inlet and outlet sizes. Longer = a gentler, gradual taper",
    )
    wall_thickness: FloatProperty(
        name="Wall Thickness", subtype='DISTANCE', default=0.065 * INCH, min=0.0, update=_update_reducer,
        description="Thickness of the reducer's metal wall",
    )
    profile_segments: IntProperty(
        name="Profile Segments", default=32, min=6, max=256, update=_update_reducer,
        description="How round the reducer's cross-section looks. Higher = smoother/rounder but more geometry",
    )


CLASSES = (
    EXHAUST_PG_Header,
    EXHAUST_PG_Segment,
    EXHAUST_PG_Route,
    EXHAUST_PG_Collector,
    EXHAUST_PG_YPipe,
    EXHAUST_PG_XPipe,
    EXHAUST_PG_HPipe,
    EXHAUST_PG_PieCut,
    EXHAUST_PG_Reducer,
)


def register_properties():
    registered = []
    pointer_names = []
    try:
        for cls in CLASSES:
            bpy.utils.register_class(cls)
            registered.append(cls)
        pointer_defs = (
            ("exhaust_header", EXHAUST_PG_Header),
            ("exhaust_route", EXHAUST_PG_Route),
            ("exhaust_collector", EXHAUST_PG_Collector),
            ("exhaust_y_pipe", EXHAUST_PG_YPipe),
            ("exhaust_x_pipe", EXHAUST_PG_XPipe),
            ("exhaust_h_pipe", EXHAUST_PG_HPipe),
            ("exhaust_pie_cut", EXHAUST_PG_PieCut),
            ("exhaust_reducer", EXHAUST_PG_Reducer),
        )
        for name, cls in pointer_defs:
            setattr(bpy.types.Object, name, PointerProperty(type=cls))
            pointer_names.append(name)
    except Exception:
        for name in reversed(pointer_names):
            try:
                delattr(bpy.types.Object, name)
            except Exception:
                pass
        for cls in reversed(registered):
            try:
                bpy.utils.unregister_class(cls)
            except Exception:
                pass
        raise


def unregister_properties():
    for name in (
        "exhaust_reducer", "exhaust_pie_cut", "exhaust_h_pipe", "exhaust_x_pipe",
        "exhaust_y_pipe", "exhaust_collector", "exhaust_route", "exhaust_header",
    ):
        if hasattr(bpy.types.Object, name):
            try:
                delattr(bpy.types.Object, name)
            except Exception:
                pass
    for cls in reversed(CLASSES):
        try:
            bpy.utils.unregister_class(cls)
        except Exception:
            pass
