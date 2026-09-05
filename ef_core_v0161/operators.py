import math
import time
import bpy
from mathutils import Vector, Matrix
from .properties import (
    INCH, header_primary_objects, refresh_header_stats, header_target_collector,
    collector_inlet_connectors, header_port_index, header_collector_status,
)
from .geometry import (rebuild_route_object, rebuild_collector_object, rebuild_y_pipe_object, rebuild_x_pipe_object, rebuild_h_pipe_object, rebuild_pie_cut_object, rebuild_reducer_object, rebuild_header_flange_object, rebuild_any, route_end_treatment_spec, route_connection_hardware_spec, collector_outlet_end_treatment_spec, route_seam_frames)
from .math_core import route_centerline_length
from .routing_solver import solve_primary_route, candidate_segments
from .guide_splines import (guide_for_route, guide_objects, create_or_reset_all, guides_to_current,
                            remove_all as remove_all_guides, guide_probe_points, guide_preferred_clocking,
                            sample_guide_local)
from .collision import (
    route_world_capsule_chain, transform_capsule_chain, capsule_chain_clearance,
    primary_collision_pairs, mesh_obstacle_from_object, capsule_chain_mesh_clearance,
    route_keepout_collisions, capsule_chain_bounds,
)
from .keepouts import keepout_names, keepout_objects, add_keepouts, remove_keepout, clear_keepouts
from .profiling import SolverProfile, stage, timed, mark, record_since


def _new_mesh_object(context, name):
    mesh = bpy.data.meshes.new(name + "_Mesh")
    obj = bpy.data.objects.new(name, mesh)
    context.collection.objects.link(obj)
    obj.location = context.scene.cursor.location
    for o in context.selected_objects:
        o.select_set(False)
    obj.select_set(True)
    context.view_layer.objects.active = obj
    return obj


def _initialize_default_route(obj, outside_diameter=None):
    s = obj.exhaust_route
    s.is_route = True
    s.outside_diameter = float(outside_diameter) if outside_diameter is not None else 3.0 * INCH
    s.wall_thickness = 0.065 * INCH
    a = s.segments.add(); a.kind = 'STRAIGHT'; a.length = 8.0 * INCH
    b = s.segments.add(); b.kind = 'BEND'; b.radius = 4.5 * INCH; b.angle = 1.57079632679; b.resolution = 24
    c = s.segments.add(); c.kind = 'STRAIGHT'; c.length = 8.0 * INCH
    rebuild_route_object(obj)
    return obj


def _align_object_start_to_target_end(mover, target):
    required_mover = ("exhaust_connector_start", "exhaust_connector_start_tangent")
    required_target = ("exhaust_connector_end", "exhaust_connector_end_tangent")
    if not all(k in mover for k in required_mover) or not all(k in target for k in required_target):
        return False
    sp_local = Vector(mover["exhaust_connector_start"])
    st_local = Vector(mover["exhaust_connector_start_tangent"]).normalized()
    ep_world = target.matrix_world @ Vector(target["exhaust_connector_end"])
    et_world = (target.matrix_world.to_3x3() @ Vector(target["exhaust_connector_end_tangent"])).normalized()
    rot = st_local.rotation_difference(et_world).to_matrix().to_4x4()
    moved_start = rot @ sp_local
    mover.matrix_world = Matrix.Translation(ep_world - moved_start) @ rot
    return True


def _copy_collector_connection_to_route_start(collector, route):
    cs = collector.exhaust_collector
    rs = route.exhaust_route
    end_spec = collector_outlet_end_treatment_spec(cs)
    rs.outside_diameter = max(1.0e-6, float(end_spec['od']))
    rs.wall_thickness = min(float(cs.wall_thickness), rs.outside_diameter * 0.499)
    kind = getattr(cs, 'outlet_connection_type', 'NONE')
    rs.start_connection_type = kind
    if kind != 'NONE':
        mapping = {
            'flange_od':'flange_od', 'flange_thickness':'flange_thickness',
            'bolt_hole_diameter':'bolt_hole_diameter', 'bolt_phase':'bolt_phase',
            '2bolt_height':'2bolt_height', '2bolt_width':'2bolt_width', '2bolt_spacing':'2bolt_spacing',
            '3bolt_height':'3bolt_height', '3bolt_width':'3bolt_width',
            'vband_neck_length':'vband_neck_length', 'vband_taper_length':'vband_taper_length',
            'vband_face_width':'vband_face_width',
        }
        for src, dst in mapping.items():
            src_name = 'outlet_' + src
            dst_name = 'start_' + dst
            if hasattr(cs, src_name) and hasattr(rs, dst_name):
                setattr(rs, dst_name, getattr(cs, src_name))
        rs.hardware_profile_segments = max(rs.hardware_profile_segments, int(getattr(cs, 'outlet_hardware_profile_segments', 64)))
    rebuild_route_object(route)


def _new_header_object(context, name):
    obj = bpy.data.objects.new(name, None)
    context.collection.objects.link(obj)
    obj.location = context.scene.cursor.location
    obj.empty_display_type = 'ARROWS'
    obj.empty_display_size = 1.5 * INCH
    return obj


def _create_header_primary(header_obj, index):
    hs = header_obj.exhaust_header
    mesh = bpy.data.meshes.new(f"{header_obj.name}_Primary_{index + 1:02d}_Mesh")
    route = bpy.data.objects.new(f"{header_obj.name}_Primary_{index + 1:02d}", mesh)
    collection = header_obj.users_collection[0] if header_obj.users_collection else bpy.context.collection
    collection.objects.link(route)
    route.parent = header_obj
    route.matrix_parent_inverse = Matrix.Identity(4)
    rs = route.exhaust_route
    rs.is_route = True
    rs.outside_diameter = hs.primary_od
    rs.wall_thickness = hs.wall_thickness
    # Header primaries terminate directly at the one shared cylinder-head
    # flange.  Start-side Route treatments/hardware are intentionally disabled.
    rs.start_end_type = 'PLAIN'
    rs.start_connection_type = 'NONE'
    rs.profile_segments = 32
    seg = rs.segments.add()
    seg.kind = 'STRAIGHT'
    seg.length = max(0.001, hs.initial_primary_length)
    route["exhaust_header_primary_index"] = int(index)
    route["exhaust_header_owner"] = header_obj.name
    rebuild_route_object(route)
    return route


def _reindex_header(header_obj):
    for i, route in enumerate(header_primary_objects(header_obj)):
        route["exhaust_header_primary_index"] = i
        route["exhaust_header_owner"] = header_obj.name


def _arrange_header_starts(header_obj):
    routes = header_primary_objects(header_obj)
    spacing = max(0.0, header_obj.exhaust_header.port_spacing)
    count = len(routes)
    for i, route in enumerate(routes):
        route.parent = header_obj
        route.matrix_parent_inverse = Matrix.Identity(4)
        route.location = Vector((0.0, (i - (count - 1) * 0.5) * spacing, 0.0))
        route.rotation_euler = (0.0, 0.0, 0.0)


def _apply_header_primary_count(header_obj):
    """Add/remove primary Routes until the Header has exactly Desired Primary Count.

    Shared by the manual "Apply Count" operator and the Primary Count property's
    own update callback, so dragging/typing a new count applies immediately
    without a separate button click.
    """
    hs = header_obj.exhaust_header
    desired = max(2, min(12, int(hs.desired_primary_count)))
    routes = header_primary_objects(header_obj)
    while len(routes) < desired:
        _create_header_primary(header_obj, len(routes))
        routes = header_primary_objects(header_obj)
    while len(routes) > desired:
        route = routes[-1]
        guide = guide_for_route(route)
        if guide is not None:
            data = guide.data
            bpy.data.objects.remove(guide, do_unlink=True)
            if data and data.users == 0:
                bpy.data.curves.remove(data)
        bpy.data.objects.remove(route, do_unlink=True)
        routes = header_primary_objects(header_obj)
    _reindex_header(header_obj)
    hs.active_primary = max(0, min(hs.active_primary, max(0, len(routes) - 1)))
    rebuild_header_flange_object(header_obj)
    refresh_header_stats(header_obj)


class EXHAUST_OT_AddHeader(bpy.types.Operator):
    bl_idname = "exhaust.add_header"
    bl_label = "Add Header"
    bl_description = "Create a Header: a set of primary pipes Blender keeps track of together, with live equal-length comparison and assisted routing"
    bl_options = {'REGISTER', 'UNDO'}
    primary_count: bpy.props.IntProperty(name="Primary Count", default=4, min=2, max=12)

    def execute(self, context):
        header = _new_header_object(context, f"Exhaust_Header_{self.primary_count}Cyl")
        hs = header.exhaust_header
        hs.is_header = True
        # Setting desired_primary_count fires its own update callback, which
        # creates exactly this many primaries and arranges/rebuilds the flange
        # for us -- a separate manual creation loop here would double them up.
        hs.desired_primary_count = self.primary_count
        hs.target_length = hs.initial_primary_length
        for obj in context.selected_objects:
            obj.select_set(False)
        header.select_set(True)
        context.view_layer.objects.active = header
        return {'FINISHED'}


class EXHAUST_OT_HeaderApplyCount(bpy.types.Operator):
    bl_idname = "exhaust.header_apply_count"
    bl_label = "Apply Primary Count"
    bl_description = "Add or remove primary pipes on this Header until it matches the Primary Count set above"
    bl_options = {'UNDO'}

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            return {'CANCELLED'}
        _apply_header_primary_count(header)
        return {'FINISHED'}


class EXHAUST_OT_HeaderArrangeStarts(bpy.types.Operator):
    bl_idname = "exhaust.header_arrange_starts"
    bl_label = "Arrange Primary Starts"
    bl_description = "Evenly space and line up every primary pipe's start position, using the Port Spacing setting above"
    bl_options = {'UNDO'}

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            return {'CANCELLED'}
        _arrange_header_starts(header)
        rebuild_header_flange_object(header)
        return {'FINISHED'}


class EXHAUST_OT_HeaderRefresh(bpy.types.Operator):
    bl_idname = "exhaust.header_refresh"
    bl_label = "Refresh Header Lengths"
    bl_description = "Recalculate every primary's length and update the equal-length numbers shown below"

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            return {'CANCELLED'}
        rebuild_header_flange_object(header)
        refresh_header_stats(header)
        return {'FINISHED'}


class EXHAUST_OT_HeaderSelectPrimary(bpy.types.Operator):
    bl_idname = "exhaust.header_select_primary"
    bl_label = "Edit Primary"
    bl_description = "Make this primary pipe the active object, so you can edit its segments directly"
    index: bpy.props.IntProperty(default=-1)

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            return {'CANCELLED'}
        routes = header_primary_objects(header)
        idx = self.index if self.index >= 0 else hs.active_primary
        if not (0 <= idx < len(routes)):
            return {'CANCELLED'}
        hs.active_primary = idx
        for obj in context.selected_objects:
            obj.select_set(False)
        routes[idx].select_set(True)
        context.view_layer.objects.active = routes[idx]
        return {'FINISHED'}


class EXHAUST_OT_SelectHeaderOwner(bpy.types.Operator):
    bl_idname = "exhaust.select_header_owner"
    bl_label = "Back to Header"
    bl_description = "Select the Header that owns this primary or guide, so you can get back to its overall settings"

    def execute(self, context):
        route = context.object
        if not route:
            return {'CANCELLED'}
        if route.type == 'CURVE' and bool(route.get('exhaust_primary_guide', False)):
            route = route.parent
            if route is None:
                return {'CANCELLED'}
        header = route.parent
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            owner_name = route.get("exhaust_header_owner", "")
            header = bpy.data.objects.get(owner_name) if owner_name else None
            hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            return {'CANCELLED'}
        for obj in context.selected_objects:
            obj.select_set(False)
        header.select_set(True)
        context.view_layer.objects.active = header
        return {'FINISHED'}


class EXHAUST_OT_HeaderSetTargetFromActive(bpy.types.Operator):
    bl_idname = "exhaust.header_target_from_active"
    bl_label = "Target = Active Primary"
    bl_description = "Use the active primary's current length as the new equal-length target for every primary"
    bl_options = {'UNDO'}

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            return {'CANCELLED'}
        routes = header_primary_objects(header)
        if not (0 <= hs.active_primary < len(routes)):
            return {'CANCELLED'}
        hs.target_mode = 'TARGET'
        hs.target_length = routes[hs.active_primary].exhaust_route.centerline_length
        refresh_header_stats(header)
        return {'FINISHED'}


class EXHAUST_OT_HeaderCopyActiveLayout(bpy.types.Operator):
    bl_idname = "exhaust.header_copy_active_layout"
    bl_label = "Copy Active Layout to All"
    bl_description = "Copy the active primary's exact segment layout (every straight and bend) onto every other primary pipe"
    bl_options = {'UNDO'}

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            return {'CANCELLED'}
        routes = header_primary_objects(header)
        if not (0 <= hs.active_primary < len(routes)):
            return {'CANCELLED'}
        src = routes[hs.active_primary].exhaust_route
        fields = ('kind','length','bend_style','radius','angle','clocking','resolution','pie_sections','show_pie_weld_seams')
        snap = [{f: getattr(seg, f) for f in fields} for seg in src.segments]
        for i, route in enumerate(routes):
            if i == hs.active_primary:
                continue
            dst = route.exhaust_route
            dst.segments.clear()
            for values in snap:
                seg = dst.segments.add()
                for key, value in values.items():
                    setattr(seg, key, value)
            dst.active_segment = min(src.active_segment, max(0, len(dst.segments)-1))
            rebuild_route_object(route)
        refresh_header_stats(header)
        return {'FINISHED'}


class EXHAUST_OT_HeaderAssignCollector(bpy.types.Operator):
    bl_idname = "exhaust.header_assign_collector"
    bl_label = "Assign Selected Collector"
    bl_description = "Assign the other selected Collector as where this Header's primaries should route to"
    bl_options = {'UNDO'}

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            self.report({'ERROR'}, "Make the Header active")
            return {'CANCELLED'}
        candidates = []
        for obj in context.selected_objects:
            if obj == header:
                continue
            cs = getattr(obj, "exhaust_collector", None)
            if cs and cs.is_collector:
                candidates.append(obj)
        if len(candidates) != 1:
            self.report({'ERROR'}, "Select the Header plus exactly one Collector, with the Header active")
            return {'CANCELLED'}
        collector = candidates[0]
        # Ensure the per-inlet metadata is current before assigning it.
        rebuild_collector_object(collector)
        hs.collector_target_name = collector.name
        hs.collector_port_offset = min(hs.collector_port_offset, max(0, int(collector.exhaust_collector.primary_count)-1))
        try:
            from .guides import tag_view3d_redraw
            tag_view3d_redraw()
        except Exception:
            pass
        return {'FINISHED'}


class EXHAUST_OT_HeaderClearCollector(bpy.types.Operator):
    bl_idname = "exhaust.header_clear_collector"
    bl_label = "Clear Collector Target"
    bl_description = "Unassign the Collector this Header is currently targeting"
    bl_options = {'UNDO'}

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            return {'CANCELLED'}
        hs.collector_target_name = ""
        try:
            from .guides import tag_view3d_redraw
            tag_view3d_redraw()
        except Exception:
            pass
        return {'FINISHED'}


class EXHAUST_OT_HeaderAddKeepouts(bpy.types.Operator):
    bl_idname = "exhaust.header_add_keepouts"
    bl_label = "Add Selected Keep-Outs"
    bl_description = "Mark the other selected mesh objects as things pipes should route around (like an oil pan or frame rail)"
    bl_options = {'UNDO'}

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            self.report({'ERROR'}, "Make the Header active, then also select one or more mesh keep-out objects")
            return {'CANCELLED'}
        primary_set = set(header_primary_objects(header))
        collector = header_target_collector(header)
        candidates = []
        for obj in context.selected_objects:
            if obj == header or obj in primary_set or obj == collector or getattr(obj, 'type', None) != 'MESH':
                continue
            if obj.get('exhaust_header_shared_flange', False) or obj.get('exhaust_header_owner', '') == header.name:
                continue
            # Keep-outs are intended to be ordinary packaging/reference meshes,
            # not another Exhaust Fabricator part accidentally selected.
            is_exhaust = False
            for attr, flag in (("exhaust_route","is_route"),("exhaust_collector","is_collector"),
                               ("exhaust_y_pipe","is_y_pipe"),("exhaust_x_pipe","is_x_pipe"),
                               ("exhaust_h_pipe","is_h_pipe"),("exhaust_reducer","is_reducer"),
                               ("exhaust_pie_cut","is_pie_cut")):
                pg = getattr(obj, attr, None)
                if pg and getattr(pg, flag, False):
                    is_exhaust = True
                    break
            if not is_exhaust:
                candidates.append(obj)
        if not candidates:
            self.report({'WARNING'}, "No ordinary mesh keep-out objects selected")
            return {'CANCELLED'}
        added = add_keepouts(header, candidates)
        self.report({'INFO'}, f"Added {added} Header keep-out object(s)")
        return {'FINISHED'}


class EXHAUST_OT_HeaderRemoveKeepout(bpy.types.Operator):
    bl_idname = "exhaust.header_remove_keepout"
    bl_label = "Remove Keep-Out"
    bl_description = "Remove this one object from the Header's keep-out list"
    bl_options = {'UNDO'}
    object_name: bpy.props.StringProperty(default="")

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            return {'CANCELLED'}
        remove_keepout(header, self.object_name)
        return {'FINISHED'}


class EXHAUST_OT_HeaderClearKeepouts(bpy.types.Operator):
    bl_idname = "exhaust.header_clear_keepouts"
    bl_label = "Clear Keep-Outs"
    bl_description = "Remove every keep-out object from this Header's list"
    bl_options = {'UNDO'}

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        if not hs or not hs.is_header:
            return {'CANCELLED'}
        clear_keepouts(header)
        return {'FINISHED'}


class EXHAUST_OT_HeaderAutoMapCollector(bpy.types.Operator):
    bl_idname = "exhaust.header_auto_map_collector"
    bl_label = "Auto Map Ports"
    bl_description = "Automatically pick which collector inlet each primary connects to, choosing whichever arrangement keeps pipe ends closest to their inlets"
    bl_options = {'UNDO'}

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        collector = header_target_collector(header) if hs and hs.is_header else None
        if not collector:
            self.report({'ERROR'}, "Assign a Collector first")
            return {'CANCELLED'}
        routes = header_primary_objects(header)
        ports = collector_inlet_connectors(collector)
        if not routes or not ports:
            return {'CANCELLED'}
        if len(routes) != len(ports):
            self.report({'WARNING'}, "Header/Collector counts differ; mapping uses available primaries only")
        route_points=[]
        for route in routes[:len(ports)]:
            if "exhaust_connector_end" not in route:
                continue
            route_points.append(route.matrix_world @ Vector(route["exhaust_connector_end"]))
        if not route_points:
            return {'CANCELLED'}
        port_world=[collector.matrix_world @ Vector(p["point"]) for p in ports]
        best=None
        for reverse in (False, True):
            direction=-1 if reverse else 1
            for offset in range(len(ports)):
                cost=0.0
                for i,rp in enumerate(route_points):
                    pi=(offset+direction*i)%len(ports)
                    d=(rp-port_world[pi]).length
                    cost += d*d
                key=(cost, int(reverse), offset)
                if best is None or key < best[0]:
                    best=(key, offset, reverse)
        if best:
            hs.collector_port_offset=best[1]
            hs.collector_reverse_order=best[2]
        try:
            from .guides import tag_view3d_redraw
            tag_view3d_redraw()
        except Exception:
            pass
        return {'FINISHED'}


class EXHAUST_OT_HeaderAlignCollectorToActive(bpy.types.Operator):
    bl_idname = "exhaust.header_align_collector_to_active"
    bl_label = "Align Collector to Active Primary"
    bl_description = "Move and rotate the assigned Collector so the active primary's pipe end lines up exactly with its matched inlet"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        header = context.object
        hs = getattr(header, "exhaust_header", None) if header else None
        collector = header_target_collector(header) if hs and hs.is_header else None
        if not collector:
            self.report({'ERROR'}, "Assign a Collector first")
            return {'CANCELLED'}
        routes=header_primary_objects(header)
        ports=collector_inlet_connectors(collector)
        idx=int(hs.active_primary)
        if not (0 <= idx < len(routes)) or not ports:
            return {'CANCELLED'}
        route=routes[idx]
        pi=header_port_index(header, idx, len(ports))
        if not (0 <= pi < len(ports)):
            return {'CANCELLED'}
        if "exhaust_connector_end" not in route or "exhaust_connector_end_tangent" not in route:
            return {'CANCELLED'}
        rp = route.matrix_world @ Vector(route["exhaust_connector_end"])
        rt = (route.matrix_world.to_3x3() @ Vector(route["exhaust_connector_end_tangent"])).normalized()
        port=ports[pi]
        cp_now = collector.matrix_world @ Vector(port["point"])
        ct_now = (collector.matrix_world.to_3x3() @ Vector(port["tangent"])).normalized()
        delta_q = ct_now.rotation_difference(rt)
        current_q = collector.matrix_world.to_quaternion()
        new_q = delta_q @ current_q
        scale = collector.matrix_world.to_scale()
        base = Matrix.LocRotScale(Vector((0.0,0.0,0.0)), new_q, scale)
        p_rot = base @ Vector(port["point"])
        loc = rp - p_rot
        collector.matrix_world = Matrix.LocRotScale(loc, new_q, scale)
        try:
            from .guides import tag_view3d_redraw
            tag_view3d_redraw()
        except Exception:
            pass
        return {'FINISHED'}


class EXHAUST_OT_HeaderSelectCollector(bpy.types.Operator):
    bl_idname = "exhaust.header_select_collector"
    bl_label = "Select Collector"
    bl_description = "Select the Collector this Header is currently targeting"

    def execute(self, context):
        header=context.object
        collector=header_target_collector(header)
        if not collector:
            return {'CANCELLED'}
        for obj in context.selected_objects:
            obj.select_set(False)
        collector.select_set(True)
        context.view_layer.objects.active=collector
        return {'FINISHED'}


def _header_from_context(context):
    obj = context.object
    hs = getattr(obj, "exhaust_header", None) if obj else None
    if hs and hs.is_header:
        return obj
    route = getattr(obj, "exhaust_route", None) if obj else None
    parent = obj.parent if obj and route and route.is_route else None
    phs = getattr(parent, "exhaust_header", None) if parent else None
    if phs and phs.is_header:
        return parent
    # Guide spline selected: guide -> primary Route -> Header.
    if obj and obj.type == 'CURVE' and bool(obj.get('exhaust_primary_guide', False)):
        route_obj = obj.parent
        header_obj = route_obj.parent if route_obj else None
        hset = getattr(header_obj, 'exhaust_header', None) if header_obj else None
        if hset and hset.is_header:
            return header_obj
    return None


def _header_solver_reference_length(header):
    hs = header.exhaust_header
    return float(hs.average_length if hs.target_mode == 'AVERAGE' else hs.target_length)


def _apply_solver_segments(route, candidate, bend_resolution):
    rs = route.exhaust_route
    rs.segments.clear()
    for spec in candidate_segments(candidate, bend_resolution):
        seg = rs.segments.add()
        seg.kind = spec['kind']
        if spec['kind'] == 'STRAIGHT':
            seg.length = spec['length']
        else:
            seg.bend_style = 'MANDREL'
            seg.radius = spec['radius']
            seg.angle = spec['angle']
            seg.clocking = spec['clocking']
            seg.resolution = spec['resolution']
    rs.active_segment = max(0, len(rs.segments) - 1)
    rebuild_route_object(route)


def _append_solver_segments(route, candidate, bend_resolution):
    """Add a solved candidate's segments AFTER whatever the Route already has.

    Used by "Complete to Collector" (v0.14.17): unlike `_apply_solver_segments`,
    this never clears the existing segment list, since those are the user's
    own manually-placed prefix and must survive exactly as drawn.
    """
    rs = route.exhaust_route
    for spec in candidate_segments(candidate, bend_resolution):
        seg = rs.segments.add()
        seg.kind = spec['kind']
        if spec['kind'] == 'STRAIGHT':
            seg.length = spec['length']
        else:
            seg.bend_style = 'MANDREL'
            seg.radius = spec['radius']
            seg.angle = spec['angle']
            seg.clocking = spec['clocking']
            seg.resolution = spec['resolution']
    rs.active_segment = max(0, len(rs.segments) - 1)
    rebuild_route_object(route)



def _snapshot_route_segments(route):
    rs = route.exhaust_route
    data = []
    for seg in rs.segments:
        data.append({
            'kind': seg.kind, 'length': float(seg.length),
            'bend_style': seg.bend_style, 'radius': float(seg.radius),
            'angle': float(seg.angle), 'clocking': float(seg.clocking),
            'resolution': int(seg.resolution), 'pie_sections': int(seg.pie_sections),
            'show_pie_weld_seams': bool(seg.show_pie_weld_seams),
        })
    return data, int(rs.active_segment)


def _restore_route_segments(route, snapshot):
    data, active = snapshot
    rs = route.exhaust_route
    rs.segments.clear()
    for spec in data:
        seg = rs.segments.add()
        seg.kind = spec['kind']
        seg.length = spec['length']
        seg.bend_style = spec['bend_style']
        seg.radius = spec['radius']
        seg.angle = spec['angle']
        seg.clocking = spec['clocking']
        seg.resolution = spec['resolution']
        seg.pie_sections = spec['pie_sections']
        seg.show_pie_weld_seams = spec['show_pie_weld_seams']
    rs.active_segment = max(0, min(active, len(rs.segments) - 1)) if len(rs.segments) else 0
    rebuild_route_object(route)


def _postbuild_primary_clearance(header, route, collision_routes=None):
    """Verify the actual rebuilt Route envelope, not just the solver candidate."""
    hs = header.exhaust_header
    if not hs.solver_avoid_primary_collisions:
        return float('inf'), None
    with stage('postbuild_primary_clearance'):
        source = list(collision_routes) if collision_routes is not None else header_primary_objects(header)
        route_chain = route_world_capsule_chain(route, include_finish_extension=True)
        best = float('inf')
        hit = None
        for other in source:
            if other is None or other == route:
                continue
            if not getattr(other, 'exhaust_route', None) or not other.exhaust_route.is_route:
                continue
            other_chain = route_world_capsule_chain(other, include_finish_extension=True)
            clr = capsule_chain_clearance(route_chain, other_chain, hs.solver_primary_clearance)
            if clr < best:
                best, hit = clr, other
    return best, hit


def _postbuild_keepout_clearance(header, route):
    """Verify the rebuilt Route against all Header-assigned keep-out meshes."""
    hs = header.exhaust_header
    if not hs.solver_avoid_keepouts:
        return float('inf'), None
    objs = keepout_objects(header)
    if not objs:
        return float('inf'), None
    with stage('postbuild_keepout_clearance'):
        try:
            depsgraph = bpy.context.evaluated_depsgraph_get()
        except Exception:
            depsgraph = None
        chain_world = route_world_capsule_chain(route, include_finish_extension=True)
        best = float('inf')
        hit = None
        ident = Matrix.Identity(4)
        tube_od = max(1.0e-4, float(route.exhaust_route.outside_diameter))
        cell = max(0.0254, tube_od, 2.0 * float(hs.solver_keepout_clearance))
        for obj in objs:
            obs = mesh_obstacle_from_object(obj, ident, cell_size=cell, depsgraph=depsgraph)
            if not obs:
                continue
            clr = capsule_chain_mesh_clearance(chain_world, obs, hs.solver_keepout_clearance)
            if clr < best:
                best, hit = clr, obj
            if clr < -1.0e-8:
                break
    return best, hit


def _header_keepout_hits(header):
    hs = header.exhaust_header
    if not hs.solver_avoid_keepouts:
        return []
    return route_keepout_collisions(
        header_primary_objects(header), keepout_objects(header),
        float(hs.solver_keepout_clearance),
    )


def _build_world_keepout_obstacles(header):
    """Build each keep-out mesh's spatial-hash obstacle ONCE, in world space.

    Evaluating a keep-out object's mesh, triangulating it, and building its
    broad-phase spatial hash grid (`mesh_obstacle_from_object`) is the
    expensive part of keep-out collision, and is completely independent of
    which primary or candidate is being tested -- it only depends on the
    keep-out object's own (evaluated) geometry. `_collision_obstacles_for_primary`
    used to rebuild this from scratch, in that specific route's local space,
    on every single call -- once per primary, per order attempt, per phase
    candidate, easily hundreds of times in one Generate All -- even though a
    world-space version built once here is equally usable by every primary:
    each candidate's own (much smaller, a few dozen segments) capsule chain
    is transformed into world space instead, in `_candidate_collision_clearance`.
    That turns an O(primaries x mesh triangles) cost into
    O(mesh triangles + primaries x candidate segments) -- a large win for
    complex real keep-out geometry (engine blocks, frame rails, etc., per
    handoff section 25), even though a trivial test cube barely shows it.

    Call this once per Generate All / Generate Active invocation, not per
    primary, and pass the result down through `_solve_one_header_primary` so
    every primary solved during that one call reuses it.
    """
    hs = header.exhaust_header
    if not hs.solver_avoid_keepouts:
        return []
    with stage('obstacle_build_keepout'):
        try:
            depsgraph = bpy.context.evaluated_depsgraph_get()
        except Exception:
            depsgraph = None
        tube_od = max(1.0e-4, float(hs.primary_od))
        cell = max(0.0254, tube_od, 2.0 * float(hs.solver_keepout_clearance))
        ident = Matrix.Identity(4)
        result = []
        for obj in keepout_objects(header):
            if obj is None:
                continue
            mesh_obs = mesh_obstacle_from_object(obj, ident, cell_size=cell, depsgraph=depsgraph)
            if mesh_obs:
                result.append({
                    'type': 'MESH',
                    'name': obj.name,
                    'mesh': mesh_obs,
                    'space': 'WORLD',
                    'clearance': float(hs.solver_keepout_clearance),
                })
        return result


def _collision_obstacles_for_primary(header, route, collision_routes=None, world_keepout_obstacles=None,
                                      local_matrix_world=None):
    """Build sibling-primary capsules (Route-local) and keep-out obstacles for one primary.

    Keep-out obstacles are taken from `world_keepout_obstacles` when the
    caller has already built them once for the whole Generate call (see
    `_build_world_keepout_obstacles`); only if that isn't provided (e.g. a
    caller outside the group-transaction path) do we fall back to building
    them fresh here, in this route's own local space, as before.

    `local_matrix_world` lets a caller solve in a frame other than the
    Route's own object origin -- e.g. "Complete to Collector" (v0.14.17)
    anchors the search at the end of the user's manually-placed prefix
    instead, and needs sibling/keep-out obstacles expressed relative to
    that frame rather than the Route origin. Defaults to `route.matrix_world`,
    reproducing the original behavior.
    """
    hs = header.exhaust_header
    local_matrix_world = local_matrix_world if local_matrix_world is not None else route.matrix_world
    inv = local_matrix_world.inverted()
    obstacles = []

    if hs.solver_avoid_primary_collisions:
        with stage('obstacle_build_primary'):
            source = list(collision_routes) if collision_routes is not None else header_primary_objects(header)
            for other in source:
                if other is None or other == route:
                    continue
                if not getattr(other, 'exhaust_route', None) or not other.exhaust_route.is_route:
                    continue
                chain_world = route_world_capsule_chain(other, include_finish_extension=True)
                if not chain_world:
                    continue
                chain_local = transform_capsule_chain(chain_world, inv)
                obstacles.append({
                    'type': 'CAPSULE',
                    'name': other.name,
                    'chain': chain_local,
                    'clearance': float(hs.solver_primary_clearance),
                    # This obstacle's chain is fixed for the entire candidate
                    # search that follows, so its collision bounds are built
                    # once here instead of on every one of the thousands of
                    # candidate evaluations that will query it.
                    'bounds': capsule_chain_bounds(chain_local, extra=0.0),
                })

    if hs.solver_avoid_keepouts:
        if world_keepout_obstacles is not None:
            obstacles.extend(world_keepout_obstacles)
        else:
            with stage('obstacle_build_keepout'):
                try:
                    depsgraph = bpy.context.evaluated_depsgraph_get()
                except Exception:
                    depsgraph = None
                tube_od = max(1.0e-4, float(route.exhaust_route.outside_diameter))
                cell = max(0.0254, tube_od, 2.0 * float(hs.solver_keepout_clearance))
                for obj in keepout_objects(header):
                    if obj is None or obj == route:
                        continue
                    mesh_obs = mesh_obstacle_from_object(obj, inv, cell_size=cell, depsgraph=depsgraph)
                    if mesh_obs:
                        obstacles.append({
                            'type': 'MESH',
                            'name': obj.name,
                            'mesh': mesh_obs,
                            'clearance': float(hs.solver_keepout_clearance),
                        })
    return obstacles


def _header_primary_collision_pairs(header, extra_clearance=None):
    if extra_clearance is None:
        extra_clearance = float(header.exhaust_header.solver_primary_clearance)
    return primary_collision_pairs(header_primary_objects(header), float(extra_clearance))


@timed('primary_solve')
def _solve_one_header_primary(header, primary_index, frozen_target_length=None, collision_routes=None,
                               dogleg_clocking_override=None, world_keepout_obstacles=None, dense_search=False):
    hs = header.exhaust_header
    collector = header_target_collector(header)
    if collector is None:
        return False, "Assign a Collector first", None
    routes = header_primary_objects(header)
    ports = collector_inlet_connectors(collector)
    if not (0 <= primary_index < len(routes)) or not ports:
        return False, "Invalid Header primary / Collector port", None
    route = routes[primary_index]
    pi = header_port_index(header, primary_index, len(ports))
    if not (0 <= pi < len(ports)):
        return False, "No mapped Collector inlet", None

    port = ports[pi]
    cp_world = collector.matrix_world @ Vector(port['point'])
    ct_world = (collector.matrix_world.to_3x3() @ Vector(port['tangent'])).normalized()

    # The routing solver operates on the core Route centerline.  If the finish
    # end has a swage/socket/flange, back the core target upstream so the actual
    # exposed connector still lands on the assigned Collector port.
    end_spec = route_end_treatment_spec(route.exhaust_route, 'end')
    end_hw = route_connection_hardware_spec(route.exhaust_route, 'end', end_spec)
    end_extension = float(end_spec['extension'] + end_hw['extension'])
    core_target_world = cp_world - ct_world * end_extension

    inv = route.matrix_world.inverted()
    target_local = inv @ core_target_world
    target_tangent_local = (inv.to_3x3() @ ct_world).normalized()

    reference_total = float(frozen_target_length if frozen_target_length is not None else _header_solver_reference_length(header))
    segment_target = max(0.0, reference_total - end_extension) if hs.solver_match_length else 0.0
    if hs.solver_match_length and segment_target <= 1.0e-6:
        return False, "Target length is shorter than the finish-end hardware extension", None

    obstacles = _collision_obstacles_for_primary(header, route, collision_routes, world_keepout_obstacles)
    guide_probes = []
    guide_influence = 0.0
    preferred_clocking = (hs.solver_dogleg_clocking if dogleg_clocking_override is None else float(dogleg_clocking_override))
    if hs.solver_use_guide_splines:
        with stage('guide_search'):
            guide_probes = guide_probe_points(route)
            if len(guide_probes) == 3:
                guide_influence = float(hs.solver_guide_influence)
                guide_clock = guide_preferred_clocking(route)
                if guide_clock is not None:
                    # Group-search clocking biases still apply around the user's guide.
                    bias = 0.0 if dogleg_clocking_override is None else float(dogleg_clocking_override) - float(hs.solver_dogleg_clocking)
                    preferred_clocking = float(guide_clock) + bias

    candidate = solve_primary_route(
        target_local, target_tangent_local, segment_target,
        hs.solver_clr, hs.solver_min_straight, hs.solver_max_bend_angle,
        hs.solver_max_dogleg_offset,
        preferred_clocking,
        hs.solver_quality, hs.solver_match_length, hs.solver_shortest_route,
        hs.length_tolerance, obstacles,
        max(0.0, float(route.exhaust_route.outside_diameter) * 0.5),
        hs.solver_primary_clearance, bool(obstacles),
        hs.solver_auto_clocking_search, hs.solver_bend_resolution,
        guide_probes, guide_influence, route_matrix_world=route.matrix_world,
        dense_search=dense_search,
        start_radius=(hs.solver_start_clr if hs.solver_use_tight_start_clr else None),
    )
    if candidate is None:
        return False, ("No collision-free route within the current CLR / angle / dogleg limits" if obstacles else "No feasible route within the current CLR / angle / dogleg limits"), None

    previous_segments = _snapshot_route_segments(route)
    _apply_solver_segments(route, candidate, hs.solver_bend_resolution)

    # Hard safety gate: verify the exact rebuilt Route envelope.  A generated
    # primary is never accepted if the final geometry still violates another
    # primary's requested clearance, even if the search approximation passed.
    final_clearance, final_hit = _postbuild_primary_clearance(header, route, collision_routes)
    if hs.solver_avoid_primary_collisions and final_clearance < -1.0e-8:
        _restore_route_segments(route, previous_segments)
        refresh_header_stats(header)
        name = final_hit.name if final_hit is not None else 'another primary'
        return False, f"Final geometry still intersects {name}; route was not applied", None

    keepout_clearance, keepout_hit = _postbuild_keepout_clearance(header, route)
    if hs.solver_avoid_keepouts and keepout_clearance < -1.0e-8:
        _restore_route_segments(route, previous_segments)
        refresh_header_stats(header)
        name = keepout_hit.name if keepout_hit is not None else 'a keep-out object'
        return False, f"Final geometry still intersects keep-out {name}; route was not applied", None

    refresh_header_stats(header)

    status = None
    for item in header_collector_status(header):
        if item['primary_index'] == primary_index:
            status = item
            break
    actual_total = float(route.exhaust_route.centerline_length)
    length_error = actual_total - reference_total
    route['exhaust_solver_last_success'] = True
    route['exhaust_solver_target_length'] = reference_total
    route['exhaust_solver_length_error'] = length_error
    route['exhaust_solver_dogleg_offset'] = float(candidate['offset'])
    route['exhaust_solver_max_bend_angle'] = float(candidate['max_angle'])
    route['exhaust_solver_dogleg_clocking_used'] = float(candidate.get('dogleg_clocking_used', hs.solver_dogleg_clocking if dogleg_clocking_override is None else float(dogleg_clocking_override)))
    _mc = float(candidate.get('min_primary_clearance', float('inf')))
    route['exhaust_solver_min_primary_clearance'] = _mc if math.isfinite(_mc) else -1.0
    route['exhaust_solver_collision_rejections'] = int(candidate.get('collision_rejections', 0))
    route['exhaust_solver_guide_error'] = float(candidate.get('guide_error', 0.0))
    route['exhaust_solver_objective'] = str(candidate.get('objective', 'EQUAL_LENGTH'))
    if status:
        route['exhaust_solver_position_error'] = float(status['position_error'])
        route['exhaust_solver_angle_error'] = float(status['angle_error'])
    else:
        route['exhaust_solver_position_error'] = -1.0
        route['exhaust_solver_angle_error'] = -1.0
    route['exhaust_solver_generated'] = True
    return True, "Solved", candidate


def _trim_capsule_chain_tail(chain, standoff):
    """Drop capsule segments within `standoff` of a chain's own tail end.

    The manually-placed prefix's own last segment always shares its exact
    endpoint with wherever the solved remainder starts -- that seam is a
    real, intended connection, not a collision, but a plain capsule-clearance
    check can't tell the difference from an actual self-intersection there
    (distance ~0 either way). Excluding the short stretch closest to the seam
    keeps the rest of the prefix as a real self-collision obstacle (e.g. an
    S-bend later looping back over earlier pipe) without rejecting every
    candidate purely for touching its own starting point.
    """
    if not chain or standoff <= 0.0:
        return list(chain)
    kept = []
    cum = 0.0
    # Each tuple runs (earlier point) -> (later, seam-ward point); walking the
    # chain tail-to-head keeps that per-segment orientation intact.
    for p0, p1, r in reversed(chain):
        p0v, p1v = Vector(p0), Vector(p1)
        seg_len = (p1v - p0v).length
        remain = standoff - cum
        if remain >= seg_len:
            # Fully inside the standoff distance from the seam -- drop it.
            cum += seg_len
            continue
        if remain > 0.0 and seg_len > 1.0e-9:
            # Straddles the standoff boundary: clip off only the near
            # (seam-ward) portion, keep the rest of this same segment.
            clip_point = p1v.lerp(p0v, remain / seg_len)
            kept.append((p0, tuple(clip_point), r))
        else:
            kept.append((p0, p1, r))
        cum += seg_len
    kept.reverse()
    return kept


@timed('primary_solve')
def _solve_one_header_primary_completion(header, primary_index, world_keepout_obstacles=None):
    """Auto-complete ONE Header primary to the Collector from wherever the user
    manually left off (v0.14.17).

    Whatever segments already exist on the Route when this runs are treated
    as a fixed, unsearched prefix -- the solver only searches the remaining
    geometry from the end of that prefix to the Collector port, using the
    same 3-corner filleted-polyline search as `_solve_one_header_primary`.
    The manual prefix itself is never touched, re-optimized, or replaced; it
    is instead added to the search as a fixed obstacle (so the solved
    remainder cannot double back into pipe the user already drew) and its
    own length is subtracted from the equal-length target.

    This reuses `solve_primary_route` completely unmodified: since the
    solver's bend/clocking math is expressed purely relative to its own local
    (tangent, up) frame (see routing_solver.py's `_turn_from_to`), re-anchoring
    the search to start at the end of the manual prefix -- instead of at the
    Route's own local origin -- only requires transforming the Collector
    target and the collision obstacles into a frame anchored there; the
    solved segments themselves are plain relative (length, angle, clocking)
    tuples that compose correctly with the existing prefix the moment
    geometry.py's turtle-graphics builder walks them, with no separate
    coordinate conversion needed.

    An empty existing segment list (nothing manually placed yet) degrades
    exactly to "Generate Active Primary" -- the anchor frame is then simply
    the Route's own origin/+X/+Z, unchanged.
    """
    hs = header.exhaust_header
    collector = header_target_collector(header)
    if collector is None:
        return False, "Assign a Collector first", None
    routes = header_primary_objects(header)
    ports = collector_inlet_connectors(collector)
    if not (0 <= primary_index < len(routes)) or not ports:
        return False, "Invalid Header primary / Collector port", None
    route = routes[primary_index]
    pi = header_port_index(header, primary_index, len(ports))
    if not (0 <= pi < len(ports)):
        return False, "No mapped Collector inlet", None

    port = ports[pi]
    cp_world = collector.matrix_world @ Vector(port['point'])
    ct_world = (collector.matrix_world.to_3x3() @ Vector(port['tangent'])).normalized()

    end_spec = route_end_treatment_spec(route.exhaust_route, 'end')
    end_hw = route_connection_hardware_spec(route.exhaust_route, 'end', end_spec)
    end_extension = float(end_spec['extension'] + end_hw['extension'])
    core_target_world = cp_world - ct_world * end_extension

    manual_prefix = _snapshot_route_segments(route)
    manual_length = float(route_centerline_length(route.exhaust_route.segments))

    frames = route_seam_frames(route.exhaust_route, include_end=True)
    p_end, t_end, n_end, _b_end = frames[-1]
    # Re-derive the binormal directly rather than reusing route_seam_frames'
    # own `b` -- that helper only needs SOME perpendicular vector for display
    # framing, but this matrix must be a proper (non-mirrored) rotation so it
    # embeds solver-space coordinates into Route-local space exactly as
    # geometry.py's own right-handed (tangent, up) convention would.
    y_axis = n_end.cross(t_end)
    y_axis = y_axis.normalized() if y_axis.length > 1.0e-9 else Vector((0.0, 1.0, 0.0))
    frame_local = Matrix((
        (t_end.x, y_axis.x, n_end.x, p_end.x),
        (t_end.y, y_axis.y, n_end.y, p_end.y),
        (t_end.z, y_axis.z, n_end.z, p_end.z),
        (0.0, 0.0, 0.0, 1.0),
    ))
    frame_matrix_world = route.matrix_world @ frame_local
    inv = frame_matrix_world.inverted()

    target_local = inv @ core_target_world
    target_tangent_local = (inv.to_3x3() @ ct_world).normalized()

    reference_total = float(_header_solver_reference_length(header))
    if hs.solver_match_length and reference_total - end_extension <= 1.0e-6:
        return False, "Target length is shorter than the finish-end hardware extension", None
    segment_target = max(0.0, reference_total - end_extension - manual_length) if hs.solver_match_length else 0.0
    if hs.solver_match_length and manual_length > 1.0e-6 and segment_target <= 1.0e-6:
        return False, "Manually placed segments already reach or exceed the target length", None

    obstacles = _collision_obstacles_for_primary(
        header, route, None, world_keepout_obstacles, local_matrix_world=frame_matrix_world,
    )
    if hs.solver_avoid_primary_collisions and len(route.exhaust_route.segments) > 0:
        self_chain_world = route_world_capsule_chain(route, include_finish_extension=False)
        standoff = 2.0 * max(float(hs.solver_clr), float(hs.solver_min_straight))
        self_chain_world = _trim_capsule_chain_tail(self_chain_world, standoff)
        if self_chain_world:
            self_chain_local = transform_capsule_chain(self_chain_world, inv)
            obstacles.append({
                'type': 'CAPSULE',
                'name': route.name + ' (manually placed prefix)',
                'chain': self_chain_local,
                'clearance': float(hs.solver_primary_clearance),
                'bounds': capsule_chain_bounds(self_chain_local, extra=0.0),
            })

    candidate = solve_primary_route(
        target_local, target_tangent_local, segment_target,
        hs.solver_clr, hs.solver_min_straight, hs.solver_max_bend_angle,
        hs.solver_max_dogleg_offset,
        hs.solver_dogleg_clocking,
        hs.solver_quality, hs.solver_match_length, hs.solver_shortest_route,
        hs.length_tolerance, obstacles,
        max(0.0, float(route.exhaust_route.outside_diameter) * 0.5),
        hs.solver_primary_clearance, bool(obstacles),
        hs.solver_auto_clocking_search, hs.solver_bend_resolution,
        None, 0.0, route_matrix_world=frame_matrix_world,
        dense_search=False,
        # Tighter Start CLR is intentionally NOT applied here: it exists for the
        # first bend nearest the flange, and by the time a user has a manual
        # prefix to complete from, that tight-clearance zone is normally
        # exactly the part they already hand-built. Applying it again here
        # would tighten the SOLVER's own first bend instead, which is no
        # longer anywhere near the flange.
    )
    if candidate is None:
        return False, ("No collision-free completion within the current CLR / angle / dogleg limits" if obstacles else "No feasible completion within the current CLR / angle / dogleg limits"), None

    _append_solver_segments(route, candidate, hs.solver_bend_resolution)

    final_clearance, final_hit = _postbuild_primary_clearance(header, route)
    if hs.solver_avoid_primary_collisions and final_clearance < -1.0e-8:
        _restore_route_segments(route, manual_prefix)
        refresh_header_stats(header)
        name = final_hit.name if final_hit is not None else 'another primary'
        return False, f"Final geometry still intersects {name}; completion was not applied", None

    keepout_clearance, keepout_hit = _postbuild_keepout_clearance(header, route)
    if hs.solver_avoid_keepouts and keepout_clearance < -1.0e-8:
        _restore_route_segments(route, manual_prefix)
        refresh_header_stats(header)
        name = keepout_hit.name if keepout_hit is not None else 'a keep-out object'
        return False, f"Final geometry still intersects keep-out {name}; completion was not applied", None

    refresh_header_stats(header)

    status = None
    for item in header_collector_status(header):
        if item['primary_index'] == primary_index:
            status = item
            break
    actual_total = float(route.exhaust_route.centerline_length)
    length_error = actual_total - reference_total
    route['exhaust_solver_last_success'] = True
    route['exhaust_solver_target_length'] = reference_total
    route['exhaust_solver_length_error'] = length_error
    route['exhaust_solver_dogleg_offset'] = float(candidate['offset'])
    route['exhaust_solver_max_bend_angle'] = float(candidate['max_angle'])
    route['exhaust_solver_dogleg_clocking_used'] = float(candidate.get('dogleg_clocking_used', hs.solver_dogleg_clocking))
    _mc = float(candidate.get('min_primary_clearance', float('inf')))
    route['exhaust_solver_min_primary_clearance'] = _mc if math.isfinite(_mc) else -1.0
    route['exhaust_solver_collision_rejections'] = int(candidate.get('collision_rejections', 0))
    route['exhaust_solver_guide_error'] = 0.0
    route['exhaust_solver_objective'] = str(candidate.get('objective', 'EQUAL_LENGTH'))
    if status:
        route['exhaust_solver_position_error'] = float(status['position_error'])
        route['exhaust_solver_angle_error'] = float(status['angle_error'])
    else:
        route['exhaust_solver_position_error'] = -1.0
        route['exhaust_solver_angle_error'] = -1.0
    route['exhaust_solver_generated'] = True
    return True, "Solved", candidate


def _run_profiled_solve(header, label, fn, *args):
    """Run a solver entry point, optionally timed, storing/printing the breakdown.

    Returns whatever `fn(*args)` returns. When Profile Solve Performance is on,
    the per-stage report is stored on the Header (visible under Solver
    Diagnostics) and printed to Blender's system console.
    """
    hs = header.exhaust_header
    if not hs.solver_profiling_enabled:
        return fn(*args)
    profile = SolverProfile()
    with profile.active():
        result = fn(*args)
    report = profile.report_text(header=f"Exhaust Fabricator solver profile — {label}")
    print(report)
    try:
        hs.solver_profile_report = profile.report_text(header=label)
    except Exception:
        pass
    return result


_SOLVER_STATE_KEYS = (
    'exhaust_solver_last_success', 'exhaust_solver_target_length',
    'exhaust_solver_length_error', 'exhaust_solver_dogleg_offset',
    'exhaust_solver_max_bend_angle', 'exhaust_solver_dogleg_clocking_used',
    'exhaust_solver_min_primary_clearance', 'exhaust_solver_collision_rejections', 'exhaust_solver_guide_error',
    'exhaust_solver_objective', 'exhaust_solver_position_error',
    'exhaust_solver_angle_error', 'exhaust_solver_generated',
)


def _snapshot_primary_state(route):
    """Snapshot route geometry plus assisted-solver metadata for transactional group search."""
    meta = {}
    for key in _SOLVER_STATE_KEYS:
        if key in route:
            try:
                meta[key] = route[key]
            except Exception:
                pass
    return {'segments': _snapshot_route_segments(route), 'meta': meta}


def _restore_primary_state(route, state):
    _restore_route_segments(route, state['segments'])
    for key in _SOLVER_STATE_KEYS:
        try:
            if key in route:
                del route[key]
        except Exception:
            pass
    for key, value in state.get('meta', {}).items():
        try:
            route[key] = value
        except Exception:
            pass


def _snapshot_header_primary_states(routes):
    return [_snapshot_primary_state(route) for route in routes]


def _restore_header_primary_states(routes, states):
    for route, state in zip(routes, states):
        _restore_primary_state(route, state)


def _blank_primary_state(hs):
    """A fresh, never-generated primary state: one default straight segment,
    matching what a newly-created Header primary starts with (see
    _create_header_primary), and no assisted-solver metadata.

    Used so Generate All Primaries always searches from a clean slate instead
    of whatever happens to already be on each primary (leftover from a prior
    Generate, a hand-edited Complete-to-Collector prefix, etc.) -- that old
    content would otherwise still count as a real collision obstacle for
    whichever primaries get solved earlier in a given order/phase attempt,
    even though it has nothing to do with the result being searched for now.
    """
    segments = [{
        'kind': 'STRAIGHT', 'length': max(0.001, float(hs.initial_primary_length)),
        'bend_style': 'MANDREL', 'radius': 3.0 * INCH,
        'angle': 0.0, 'clocking': 0.0, 'resolution': 16,
        'pie_sections': 6, 'show_pie_weld_seams': True,
    }]
    return {'segments': (segments, 0), 'meta': {}}


def _blank_header_primary_states(header, routes):
    hs = header.exhaust_header
    return [_blank_primary_state(hs) for _ in routes]


def _group_solve_orders(count, quality='NORMAL'):
    """Return deterministic alternative primary orders for global collision search.

    The older solver always used 0..N-1.  That is greedy: an early primary can
    occupy the only clean corridor for a later one.  These order families give
    the same exact per-primary solver several materially different packing
    opportunities without factorial growth.
    """
    n = max(0, int(count))
    if n <= 1:
        return [list(range(n))]
    orders = []

    def add(order):
        order = [int(i) for i in order]
        if len(order) == n and sorted(order) == list(range(n)) and order not in orders:
            orders.append(order)

    forward = list(range(n))
    reverse = list(reversed(forward))
    add(forward); add(reverse)

    # Cyclic rotations allow a different primary to claim the first corridor.
    cycle_limit = {'FAST': min(n, 2), 'NORMAL': min(n, 4), 'HIGH': n}.get(str(quality), min(n, 4))
    for shift in range(1, cycle_limit):
        add(forward[shift:] + forward[:shift])
        r = reverse[shift:] + reverse[:shift]
        add(r)

    # Outside-in and center-out are useful for typical inline cylinder-head rows.
    outside = []
    lo, hi = 0, n - 1
    while lo <= hi:
        outside.append(lo); lo += 1
        if lo <= hi:
            outside.append(hi); hi -= 1
    add(outside)
    add(list(reversed(outside)))

    center = []
    if n % 2:
        center.append(n // 2)
        step = 1
        while len(center) < n:
            if n // 2 - step >= 0:
                center.append(n // 2 - step)
            if n // 2 + step < n:
                center.append(n // 2 + step)
            step += 1
    else:
        left, right = n // 2 - 1, n // 2
        while left >= 0 or right < n:
            if left >= 0:
                center.append(left); left -= 1
            if right < n:
                center.append(right); right += 1
    add(center)
    add(list(reversed(center)))

    max_attempts = {'FAST': 4, 'NORMAL': 10, 'HIGH': 24}.get(str(quality), 10)
    return orders[:max_attempts]


def _group_clocking_bias(attempt_index, quality='NORMAL'):
    """Small phase shifts fill gaps between per-primary auto-clocking samples."""
    if attempt_index <= 0:
        return 0.0
    if str(quality) == 'FAST':
        seq = (math.radians(22.5), math.radians(-22.5))
    elif str(quality) == 'HIGH':
        seq = (math.radians(11.25), math.radians(-11.25), math.radians(22.5),
               math.radians(-22.5), math.radians(33.75), math.radians(-33.75))
    else:
        seq = (math.radians(15.0), math.radians(-15.0), math.radians(30.0), math.radians(-30.0))
    return seq[(attempt_index - 1) % len(seq)]


def _group_solution_score(header, routes, target):
    """Score only an already collision-free complete Header solution."""
    hs = header.exhaust_header
    lengths = [float(r.exhaust_route.centerline_length) for r in routes]
    total = sum(lengths)
    max_offset = max((abs(float(r.get('exhaust_solver_dogleg_offset', 0.0))) for r in routes), default=0.0)
    total_offset = sum(abs(float(r.get('exhaust_solver_dogleg_offset', 0.0))) for r in routes)
    min_clearance = min((float(r.get('exhaust_solver_min_primary_clearance', 1.0e30)) for r in routes), default=1.0e30)
    total_guide_error = sum(float(r.get('exhaust_solver_guide_error', 0.0)) for r in routes) if hs.solver_use_guide_splines else 0.0
    if hs.solver_match_length and target is not None:
        errs = [abs(L - float(target)) for L in lengths]
        outside = [max(0.0, e - float(hs.length_tolerance)) for e in errs]
        # Complete solutions within tolerance are ranked by the shortest total
        # tubing first, matching the user's requested Equal-Length objective.
        return (max(outside, default=0.0), sum(outside), total, total_guide_error, max_offset, total_offset, -min_clearance)
    if hs.solver_shortest_route:
        return (total, total_guide_error, max_offset, total_offset, -min_clearance)
    return (total_guide_error, total_offset, max_offset, total, -min_clearance)


def _solve_header_primary_set_fixed_phase(header, routes, target, deadline=None, world_keepout_obstacles=None,
                                           dense_search=False):
    """Transactional multi-order solve for the collector's CURRENT radial phase.

    Stops trying additional orders once `solver_order_success_cap` complete
    collision-free orders have been found (the best-scoring one is kept) --
    mirrors the success-cap pattern already used for collector-phase search
    below, so a scenario with several easy valid orders doesn't keep paying
    for orders that can only ever change the tie-break, not the outcome.
    `deadline` (a `time.perf_counter()` timestamp from the caller) is an
    optional shared time budget across every order *and* every phase; it is
    checked only between whole order attempts, never mid-attempt.  This is
    deliberate, not just an implementation shortcut: an order attempt that is
    already most of the way through solving every primary is also an order
    attempt that is close to *succeeding*, and killing it partway would trade
    a real (if slow) solution for a guaranteed "no solution found" -- the one
    outcome this budget must never cause on its own, since it exists to bound
    worst-case time on scenarios that were never going to succeed regardless
    of how long they ran, not to punish scenarios that just take one long
    attempt. The practical effect: total time is bounded by roughly the
    budget plus one attempt's own worst-case duration, so pick the budget
    together with Search Quality (which directly controls how long a single
    attempt can take) rather than assuming the budget alone caps latency
    tightly.
    """
    hs = header.exhaust_header
    original = _snapshot_header_primary_states(routes)
    orders = _group_solve_orders(len(routes), hs.solver_quality)
    order_success_cap = max(1, int(hs.solver_order_success_cap))
    best = None
    successful_orders = 0
    attempt_messages = []

    for attempt_index, order in enumerate(orders):
        if deadline is not None and time.perf_counter() >= deadline:
            attempt_messages.append(
                f'Search budget exhausted after {attempt_index}/{len(orders)} order attempt(s) at this phase'
            )
            break
        attempt_start = mark()
        _restore_header_primary_states(routes, original)
        refresh_header_stats(header)
        accepted = []
        failed = None
        bias = _group_clocking_bias(attempt_index, hs.solver_quality)
        preferred_clock = float(hs.solver_dogleg_clocking) + bias

        for i in order:
            collision_set = accepted if hs.solver_avoid_primary_collisions else None
            ok, message, _candidate = _solve_one_header_primary(
                header, i, target, collision_set, preferred_clock,
                world_keepout_obstacles=world_keepout_obstacles, dense_search=dense_search,
            )
            if not ok:
                failed = f'P{i+1}: {message}'
                break
            accepted.append(routes[i])

        if failed is not None:
            attempt_messages.append(failed)
            record_since('order_attempt', attempt_start)
            continue

        # Full-set hard gate: every accepted Header must be clear of both sibling
        # primaries and every assigned arbitrary keep-out mesh.
        with stage('fullset_gate_check'):
            pairs = _header_primary_collision_pairs(header) if hs.solver_avoid_primary_collisions else []
            keep_hits = _header_keepout_hits(header) if hs.solver_avoid_keepouts else []
        if pairs or keep_hits:
            attempt_messages.append(
                f'Attempt {attempt_index+1}: {len(pairs)} primary collision(s), {len(keep_hits)} keep-out collision(s)'
            )
            record_since('order_attempt', attempt_start)
            continue

        refresh_header_stats(header)
        score = _group_solution_score(header, routes, target)
        states = _snapshot_header_primary_states(routes)
        if best is None or score < best[0]:
            best = (score, states, list(order), preferred_clock)
        record_since('order_attempt', attempt_start)
        successful_orders += 1
        if successful_orders >= order_success_cap:
            break

    if best is None:
        _restore_header_primary_states(routes, original)
        refresh_header_stats(header)
        return False, None, attempt_messages

    _restore_header_primary_states(routes, best[1])
    refresh_header_stats(header)
    # Final verification after restoring the selected winning snapshot.
    final_pairs = _header_primary_collision_pairs(header) if hs.solver_avoid_primary_collisions else []
    final_keep_hits = _header_keepout_hits(header) if hs.solver_avoid_keepouts else []
    if final_pairs or final_keep_hits:
        _restore_header_primary_states(routes, original)
        refresh_header_stats(header)
        return False, None, [
            f'Winning snapshot failed final safety gate: {len(final_pairs)} primary collision(s), {len(final_keep_hits)} keep-out collision(s)'
        ]
    return True, best, attempt_messages


def _phase_wrap(angle):
    """Wrap an angle to [-pi, pi) for compact stored/report values."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _collector_phase_offsets(header, collector):
    """Unique radial-phase offsets around the current phase.

    With cyclic port remapping, rotating farther than half one inlet pitch just
    repeats an equivalent assignment.  Search density follows the existing
    routing Search Quality so this feature does not introduce another quality
    knob.
    """
    hs = header.exhaust_header
    ports = collector_inlet_connectors(collector)
    n = max(1, len(ports))
    if not hs.solver_adjust_collector_phase or n <= 1:
        return [0.0]
    limit = min(abs(float(hs.solver_collector_phase_range)), math.pi / n)
    if limit <= 1.0e-10:
        return [0.0]
    samples = {'FAST': 3, 'NORMAL': 5, 'HIGH': 9}.get(str(hs.solver_quality), 5)
    if samples <= 1:
        return [0.0]
    values = [0.0]
    pairs = (samples - 1) // 2
    for i in range(1, pairs + 1):
        d = limit * i / pairs
        values.extend((d, -d))
    return values


@timed('phase_mapping_heuristic')
def _best_mapping_for_current_collector(header, collector):
    """Cheap start-to-port mapping heuristic for a collector phase candidate."""
    hs = header.exhaust_header
    routes = header_primary_objects(header)
    ports = collector_inlet_connectors(collector)
    if not routes or not ports:
        return int(hs.collector_port_offset), bool(hs.collector_reverse_order), float('inf')

    # Use the cylinder-head-side connector, not the previous generated endpoint.
    # This makes phase ranking independent of whatever route happened to exist
    # before the solve.
    starts = []
    for route in routes[:len(ports)]:
        # In Guide mode the drawn spline's downstream end is a better cheap
        # phase/mapping heuristic than the cylinder-head start. The expensive
        # route solver still treats the guide as soft only.
        gpts = sample_guide_local(route) if hs.solver_use_guide_splines else []
        if gpts:
            starts.append(route.matrix_world @ Vector(gpts[-1]))
        elif 'exhaust_connector_start' in route:
            starts.append(route.matrix_world @ Vector(route['exhaust_connector_start']))
        else:
            starts.append(route.matrix_world.translation.copy())
    port_world = [collector.matrix_world @ Vector(p['point']) for p in ports]
    best = None
    for reverse in (False, True):
        direction = -1 if reverse else 1
        for offset in range(len(ports)):
            cost = 0.0
            for i, sp in enumerate(starts):
                pi = (offset + direction * i) % len(ports)
                d = (sp - port_world[pi]).length
                cost += d * d
            key = (cost, int(reverse), offset)
            if best is None or key < best[0]:
                best = (key, offset, reverse)
    if best is None:
        return int(hs.collector_port_offset), bool(hs.collector_reverse_order), float('inf')
    return int(best[1]), bool(best[2]), float(best[0][0])


@timed('phase_rank')
def _rank_collector_phase_candidates(header, collector, original_phase, original_offset, original_reverse):
    """Pre-rank phase candidates with a cheap mapping-distance heuristic."""
    hs = header.exhaust_header
    cs = collector.exhaust_collector
    ranked = []
    for delta in _collector_phase_offsets(header, collector):
        phase = float(original_phase) + float(delta)
        cs.radial_phase = phase  # property update rebuilds connector metadata synchronously
        if hs.solver_phase_auto_remap:
            offset, reverse, map_cost = _best_mapping_for_current_collector(header, collector)
        else:
            offset, reverse, map_cost = int(original_offset), bool(original_reverse), 0.0
        ranked.append((map_cost, abs(float(delta)), float(delta), phase, offset, reverse))
    # Restore before expensive route solving starts.
    cs.radial_phase = float(original_phase)
    hs.collector_port_offset = int(original_offset)
    hs.collector_reverse_order = bool(original_reverse)
    ranked.sort(key=lambda x: (x[0], x[1]))
    return ranked


def _solve_header_primary_set(header, routes, target):
    """Global transactional solve with optional Collector Radial Phase search.

    A winning transaction contains route geometry, collector phase AND port map.
    If no complete solution exists, all three are restored exactly.

    Tries the fast per-primary search first (a coarser clocking grid,
    compensated by locally refining clocking too -- see solve_primary_route's
    docstring), then falls back to the slower, more conservative dense search
    ONLY if the fast search found no complete solution at all across every
    phase and order tried. This matters because the fast search can find a
    genuinely *better* per-primary candidate that nonetheless occupies the
    shared 3D volume differently than the old dense search would have, and in
    an already-tightly-packed Header that shift alone -- no bug, every
    individual candidate fully valid -- can be enough to make a later primary
    in the same order infeasible where the old search's result would not
    have been. The fallback exists specifically to recover that case: pay the
    slower, proven-reliable search's cost only when the faster one already
    failed outright, which is exactly the situation where spending more time
    is worth it.

    The fast and dense attempts each get their OWN full `Max Search Time`
    budget rather than splitting one shared window. Found by testing against
    a realistically complex keep-out mesh (thousands of triangles, not the
    trivial test cube used earlier): a near-zero-slack Header whose fast
    search fails outright can burn through the ENTIRE budget on the fast
    attempt alone before the dense fallback -- the one actually capable of
    finding the solution -- ever gets a turn, silently turning a scenario
    that used to succeed comfortably within the default budget (back when
    there was only ever one, "dense", search) into a reported failure. Giving
    each attempt its own full budget means a genuinely infeasible Header now
    takes up to ~2x as long to report failure, but a Header the dense search
    CAN solve is no longer starved out of the time it needs by how long the
    fast attempt happened to take failing first.
    """
    hs = header.exhaust_header
    collector = header_target_collector(header)
    if collector is None:
        return False, None, ['Assign a Collector first']
    cs = collector.exhaust_collector

    original_states = _snapshot_header_primary_states(routes)
    original_phase = float(cs.radial_phase)
    original_offset = int(hs.collector_port_offset)
    original_reverse = bool(hs.collector_reverse_order)
    # Every attempt below searches from a blank slate (one default straight
    # segment per primary), not from whatever's currently on each primary --
    # see _blank_primary_state's docstring for why leftover content would
    # otherwise silently bias the search as a stale collision obstacle.
    # original_states is kept as-is and still used to restore the user's
    # actual pre-click layout if the search ultimately fails outright.
    blank_states = _blank_header_primary_states(header, routes)

    # Keep-out geometry doesn't move with collector phase or primary order, so
    # build its (potentially expensive) world-space obstacle representation
    # once for this entire search instead of once per primary solved.
    world_keepout_obstacles = _build_world_keepout_obstacles(header)

    phase_candidates = _rank_collector_phase_candidates(
        header, collector, original_phase, original_offset, original_reverse
    ) if hs.solver_adjust_collector_phase else [
        (0.0, 0.0, 0.0, original_phase, original_offset, original_reverse)
    ]

    # Each of the fast and dense attempts below gets its OWN full wall-clock
    # budget (see this function's docstring for why a shared one starved the
    # dense fallback on expensive keep-out meshes), not a per-phase allowance
    # -- otherwise a Header with several phase candidates could still run for
    # (budget x phase_count) before giving up.  Checked between attempts
    # only; see _solve_header_primary_set_fixed_phase's docstring for why
    # that's still enough to bound the actual runaway case (many attempts
    # multiplying together), just not a single slow attempt.
    budget_seconds = max(1.0, float(hs.solver_max_solve_seconds))
    # Fast stops on the first globally valid phase. Normal compares the first
    # two valid phases; High compares every sampled phase. This keeps the new
    # degree of freedom useful without multiplying keep-out solve time blindly.
    success_cap = {'FAST': 1, 'NORMAL': 2, 'HIGH': 10**6}.get(str(hs.solver_quality), 2)

    def _try_all_phases(dense_search, deadline):
        best_global = None
        messages = []
        successful_phases = 0
        tested = 0
        for _heur, _abs_delta, delta, phase, map_offset, map_reverse in phase_candidates:
            if time.perf_counter() >= deadline:
                messages.append(f'Search budget exhausted after {tested}/{len(phase_candidates)} phase candidate(s)')
                break
            phase_start = mark()
            tested += 1
            _restore_header_primary_states(routes, blank_states)
            cs.radial_phase = float(phase)
            hs.collector_port_offset = int(map_offset)
            hs.collector_reverse_order = bool(map_reverse)
            refresh_header_stats(header)

            ok, local_best, local_messages = _solve_header_primary_set_fixed_phase(
                header, routes, target, deadline, world_keepout_obstacles=world_keepout_obstacles,
                dense_search=dense_search,
            )
            messages.extend([f'Phase {math.degrees(delta):+.1f}°: {m}' for m in local_messages[-2:]])
            if not ok or local_best is None:
                record_since('phase_attempt', phase_start)
                continue

            successful_phases += 1
            # local_best[0] is the objective score already enforcing collision /
            # equal-length priorities. Phase movement is only a final tie-breaker.
            global_score = tuple(local_best[0]) + (abs(float(delta)), float(_heur))
            states = _snapshot_header_primary_states(routes)
            candidate = (
                global_score, states, list(local_best[2]), local_best[3],
                float(phase), int(map_offset), bool(map_reverse), float(delta), tested,
            )
            if best_global is None or global_score < best_global[0]:
                best_global = candidate
            record_since('phase_attempt', phase_start)
            if successful_phases >= success_cap:
                break
        return best_global, messages

    fast_deadline = time.perf_counter() + budget_seconds
    best_global, messages = _try_all_phases(dense_search=False, deadline=fast_deadline)
    if best_global is None:
        # A fresh full budget, not whatever's left of fast_deadline: the fast
        # attempt failing outright already means it used its own budget
        # fairly (or ran out early) -- the dense fallback still deserves a
        # full, undiminished chance to find what the fast search couldn't.
        dense_deadline = time.perf_counter() + budget_seconds
        dense_best, dense_messages = _try_all_phases(dense_search=True, deadline=dense_deadline)
        if dense_best is not None:
            messages = [
                'Fast search found no complete solution; retried with the slower, '
                'more conservative dense search, which succeeded.'
            ]
            best_global = dense_best
        else:
            messages = messages + ['Dense fallback search also found no complete solution:'] + dense_messages

    if best_global is None:
        _restore_header_primary_states(routes, original_states)
        cs.radial_phase = original_phase
        hs.collector_port_offset = original_offset
        hs.collector_reverse_order = original_reverse
        refresh_header_stats(header)
        for key in ('exhaust_solver_collector_phase_used','exhaust_solver_collector_phase_delta','exhaust_solver_collector_phase_candidates'):
            try:
                if key in header: del header[key]
            except Exception:
                pass
        return False, None, messages

    cs.radial_phase = float(best_global[4])
    hs.collector_port_offset = int(best_global[5])
    hs.collector_reverse_order = bool(best_global[6])
    _restore_header_primary_states(routes, best_global[1])
    refresh_header_stats(header)

    # One last hard gate with the collector left at the winning phase.
    final_pairs = _header_primary_collision_pairs(header) if hs.solver_avoid_primary_collisions else []
    final_keep_hits = _header_keepout_hits(header) if hs.solver_avoid_keepouts else []
    statuses = header_collector_status(header)
    target_bad = any(
        item['position_error'] > max(1.0e-7, float(hs.collector_position_tolerance)) or
        item['angle_error'] > max(1.0e-7, float(hs.collector_angle_tolerance))
        for item in statuses
    )
    if final_pairs or final_keep_hits or target_bad:
        _restore_header_primary_states(routes, original_states)
        cs.radial_phase = original_phase
        hs.collector_port_offset = original_offset
        hs.collector_reverse_order = original_reverse
        refresh_header_stats(header)
        return False, None, [
            f'Winning phase failed final gate: {len(final_pairs)} primary collision(s), '
            f'{len(final_keep_hits)} keep-out collision(s), target alignment={"bad" if target_bad else "ok"}'
        ]

    header['exhaust_solver_collector_phase_used'] = float(_phase_wrap(best_global[4]))
    header['exhaust_solver_collector_phase_delta'] = float(best_global[7])
    header['exhaust_solver_collector_phase_candidates'] = int(best_global[8])
    return True, best_global, messages


class EXHAUST_OT_HeaderCreateResetGuides(bpy.types.Operator):
    bl_idname = "exhaust.header_create_reset_guides"
    bl_label = "Create / Reset Guides"
    bl_description = "Create an editable guide curve for each primary that you can sketch a rough path onto, or reset existing guides back to their targets"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        header = _header_from_context(context)
        if not header:
            self.report({'ERROR'}, "Select a Header or one of its primaries")
            return {'CANCELLED'}
        guides = create_or_reset_all(header)
        self.report({'INFO'}, f"Created/reset {len(guides)} primary guide spline(s)")
        return {'FINISHED'}


class EXHAUST_OT_HeaderGuidesToCurrent(bpy.types.Operator):
    bl_idname = "exhaust.header_guides_to_current"
    bl_label = "Splines to Current"
    bl_description = "Reshape every guide curve to match how its primary is currently routed"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        header = _header_from_context(context)
        if not header:
            self.report({'ERROR'}, "Select a Header or one of its primaries")
            return {'CANCELLED'}
        guides = guides_to_current(header)
        if not guides:
            self.report({'WARNING'}, "Header has no managed primary Routes")
            return {'CANCELLED'}
        self.report({'INFO'}, f"Updated {len(guides)} guide spline(s) from the current primaries")
        return {'FINISHED'}


class EXHAUST_OT_HeaderSelectActiveGuide(bpy.types.Operator):
    bl_idname = "exhaust.header_select_active_guide"
    bl_label = "Select Active Guide"
    bl_description = "Select the editable guide curve belonging to the Header's active primary"

    def execute(self, context):
        header = _header_from_context(context)
        if not header:
            return {'CANCELLED'}
        routes = header_primary_objects(header)
        if not routes:
            return {'CANCELLED'}
        idx = max(0, min(int(header.exhaust_header.active_primary), len(routes)-1))
        guide = guide_for_route(routes[idx])
        if guide is None:
            self.report({'WARNING'}, "No guide exists for the active primary; use Create / Reset Guides first")
            return {'CANCELLED'}
        if context.object and context.object.mode != 'OBJECT':
            try: bpy.ops.object.mode_set(mode='OBJECT')
            except Exception: pass
        for obj in context.selected_objects:
            obj.select_set(False)
        guide.hide_set(False)
        guide.select_set(True)
        context.view_layer.objects.active = guide
        return {'FINISHED'}


class EXHAUST_OT_HeaderRemoveGuides(bpy.types.Operator):
    bl_idname = "exhaust.header_remove_guides"
    bl_label = "Remove Guides"
    bl_description = "Delete every guide curve owned by this Header"
    bl_options = {'UNDO'}

    def execute(self, context):
        header = _header_from_context(context)
        if not header:
            return {'CANCELLED'}
        count = remove_all_guides(header)
        self.report({'INFO'}, f"Removed {count} primary guide spline(s)")
        return {'FINISHED'}


class EXHAUST_OT_HeaderSolveActivePrimary(bpy.types.Operator):
    bl_idname = "exhaust.header_solve_active_primary"
    bl_label = "Generate Active Primary"
    bl_description = "Automatically build a new pipe path for the active primary, from its start all the way to its assigned Collector inlet"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        header = _header_from_context(context)
        if not header:
            self.report({'ERROR'}, "Select a Header or one of its primaries")
            return {'CANCELLED'}
        hs = header.exhaust_header
        if context.object != header and getattr(context.object, 'exhaust_route', None) and context.object.exhaust_route.is_route:
            idx = int(context.object.get('exhaust_header_primary_index', hs.active_primary))
            hs.active_primary = max(0, idx)
        elif context.object and context.object.type == 'CURVE' and bool(context.object.get('exhaust_primary_guide', False)):
            route_obj = context.object.parent
            if route_obj is not None:
                idx = int(route_obj.get('exhaust_header_primary_index', hs.active_primary))
                hs.active_primary = max(0, idx)
        idx = int(hs.active_primary)
        ok, message, candidate = _run_profiled_solve(
            header, f'Generate Active Primary (P{idx+1})', _solve_one_header_primary, header, idx
        )
        if not ok:
            self.report({'WARNING'}, message)
            return {'CANCELLED'}
        route = header_primary_objects(header)[idx]
        pos = float(route.get('exhaust_solver_position_error', 0.0))
        ang = float(route.get('exhaust_solver_angle_error', 0.0))
        le = float(route.get('exhaust_solver_length_error', 0.0))
        objective = str(route.get('exhaust_solver_objective', 'EQUAL_LENGTH'))
        if objective == 'SHORTEST':
            self.report({'INFO'}, f'P{idx+1}: shortest route {float(route.exhaust_route.centerline_length)/INCH:.3f}"; target error {pos/INCH:.3f}" / {math.degrees(max(0.0,ang)):.2f}°')
        elif objective == 'FEASIBLE':
            self.report({'INFO'}, f'P{idx+1}: feasible route {float(route.exhaust_route.centerline_length)/INCH:.3f}"; target error {pos/INCH:.3f}" / {math.degrees(max(0.0,ang)):.2f}°')
        else:
            self.report({'INFO'}, f'P{idx+1}: ΔL {le/INCH:+.3f}"; target error {pos/INCH:.3f}" / {math.degrees(max(0.0,ang)):.2f}°')
        try:
            from .guides import tag_view3d_redraw
            tag_view3d_redraw()
        except Exception:
            pass
        return {'FINISHED'}


class EXHAUST_OT_HeaderCompleteActivePrimary(bpy.types.Operator):
    bl_idname = "exhaust.header_complete_active_primary"
    bl_label = "Complete to Collector"
    bl_description = "Keep any segments you've already added by hand, and auto-route just the rest of the way to the assigned Collector inlet"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        header = _header_from_context(context)
        if not header:
            self.report({'ERROR'}, "Select a Header or one of its primaries")
            return {'CANCELLED'}
        hs = header.exhaust_header
        if context.object != header and getattr(context.object, 'exhaust_route', None) and context.object.exhaust_route.is_route:
            idx = int(context.object.get('exhaust_header_primary_index', hs.active_primary))
            hs.active_primary = max(0, idx)
        idx = int(hs.active_primary)
        ok, message, candidate = _run_profiled_solve(
            header, f'Complete Active Primary to Collector (P{idx+1})', _solve_one_header_primary_completion, header, idx
        )
        if not ok:
            self.report({'WARNING'}, message)
            return {'CANCELLED'}
        route = header_primary_objects(header)[idx]
        pos = float(route.get('exhaust_solver_position_error', 0.0))
        ang = float(route.get('exhaust_solver_angle_error', 0.0))
        le = float(route.get('exhaust_solver_length_error', 0.0))
        self.report({'INFO'}, f'P{idx+1}: completed to Collector; ΔL {le/INCH:+.3f}"; target error {pos/INCH:.3f}" / {math.degrees(max(0.0,ang)):.2f}°')
        try:
            from .guides import tag_view3d_redraw
            tag_view3d_redraw()
        except Exception:
            pass
        return {'FINISHED'}


class EXHAUST_OT_HeaderSolveAllPrimaries(bpy.types.Operator):
    bl_idname = "exhaust.header_solve_all_primaries"
    bl_label = "Generate All Primaries"
    bl_description = "Automatically build paths for every primary at once, trying different orders so the whole set fits together without any pipes colliding"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        header = _header_from_context(context)
        if not header:
            self.report({'ERROR'}, "Select a Header or one of its primaries")
            return {'CANCELLED'}
        if header_target_collector(header) is None:
            self.report({'ERROR'}, "Assign a Collector first")
            return {'CANCELLED'}
        routes = header_primary_objects(header)
        if not routes:
            return {'CANCELLED'}

        hs = header.exhaust_header
        # Freeze Current Average once for the entire global search.  Every order
        # competes against the exact same objective.
        target = _header_solver_reference_length(header) if hs.solver_match_length else None

        use_group_transaction = (
            (hs.solver_avoid_primary_collisions and len(routes) > 1) or
            (hs.solver_avoid_keepouts and bool(keepout_objects(header))) or
            bool(hs.solver_adjust_collector_phase)
        )
        if use_group_transaction:
            ok, best, messages = _run_profiled_solve(
                header, 'Generate All Primaries (group transaction)',
                _solve_header_primary_set, header, routes, target
            )
            try:
                from .guides import tag_view3d_redraw
                tag_view3d_redraw()
            except Exception:
                pass
            if not ok:
                detail = messages[-1] if messages else 'No complete collision-free primary set found'
                self.report({'WARNING'}, 'No complete constraint-clear Header solution found; previous routes restored. ' + detail)
                return {'CANCELLED'}

            order = best[2] if best else list(range(len(routes)))
            order_text = ' → '.join(f'P{i+1}' for i in order)
            phase_suffix = ''
            if best and len(best) > 7 and hs.solver_adjust_collector_phase:
                phase_suffix = f'; collector phase {math.degrees(float(best[4])):.1f}° ({math.degrees(float(best[7])):+.1f}°)'
            if hs.solver_match_length:
                self.report({'INFO'}, f'Solved all {len(routes)} primaries clear of primaries/keep-outs at {target/INCH:.3f}" reference; selected shortest complete equal-length set (order {order_text}){phase_suffix}')
            elif hs.solver_shortest_route:
                self.report({'INFO'}, f'Solved all {len(routes)} primaries clear of primaries/keep-outs using shortest complete set (order {order_text}){phase_suffix}')
            else:
                self.report({'INFO'}, f'Solved all {len(routes)} primaries clear of primaries/keep-outs using global multi-order search (order {order_text}){phase_suffix}')
            return {'FINISHED'}

        # Collision avoidance disabled: preserve the lightweight independent
        # behavior rather than paying for group-search attempts unnecessarily.
        def _solve_independent():
            solved = 0
            failures = []
            for i in range(len(routes)):
                ok, message, _candidate = _solve_one_header_primary(header, i, target, None)
                if ok:
                    solved += 1
                else:
                    failures.append(f'P{i+1}: {message}')
            return solved, failures

        solved, failures = _run_profiled_solve(
            header, 'Generate All Primaries (independent)', _solve_independent
        )
        refresh_header_stats(header)
        try:
            from .guides import tag_view3d_redraw
            tag_view3d_redraw()
        except Exception:
            pass
        if failures:
            self.report({'WARNING'}, f'Solved {solved}/{len(routes)}. ' + '; '.join(failures[:3]))
        else:
            if hs.solver_match_length:
                self.report({'INFO'}, f'Solved all {solved} primaries to the frozen {target/INCH:.3f}" reference')
            elif hs.solver_shortest_route:
                self.report({'INFO'}, f'Solved all {solved} primaries using the shortest-feasible-route objective')
            else:
                self.report({'INFO'}, f'Solved all {solved} primaries using compact/gentle feasible routing')
        return {'FINISHED'} if solved else {'CANCELLED'}


class EXHAUST_OT_SegmentRow(bpy.types.Operator):
    """Make a route segment active in the segment list and viewport guides."""
    bl_idname = "exhaust.segment_row"
    bl_label = "Route Segment"
    bl_description = "Make this route segment active"
    bl_options = {'INTERNAL'}

    object_name: bpy.props.StringProperty(options={'HIDDEN', 'SKIP_SAVE'})
    index: bpy.props.IntProperty(default=-1, options={'HIDDEN', 'SKIP_SAVE'})

    def execute(self, context):
        obj = bpy.data.objects.get(self.object_name)
        if not obj or not getattr(obj, "exhaust_route", None) or not obj.exhaust_route.is_route:
            return {'CANCELLED'}
        if 0 <= self.index < len(obj.exhaust_route.segments):
            obj.exhaust_route.active_segment = self.index
            context.view_layer.objects.active = obj
            obj.select_set(True)
            try:
                from .guides import tag_view3d_redraw
                tag_view3d_redraw()
            except Exception:
                pass
        return {'FINISHED'}


class EXHAUST_OT_AddRoute(bpy.types.Operator):
    bl_idname = "exhaust.add_route"
    bl_label = "Add Exhaust Route"
    bl_description = "Create a new pipe made of editable straight and bend segments"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = _new_mesh_object(context, "Exhaust_Route")
        _initialize_default_route(obj)
        return {'FINISHED'}


class EXHAUST_OT_AddCollector(bpy.types.Operator):
    bl_idname = "exhaust.add_collector"
    bl_label = "Add Collector"
    bl_description = "Create a collector that merges any number of primary pipes into one outlet, evenly spaced around a center"
    bl_options = {'REGISTER', 'UNDO'}

    primary_count: bpy.props.IntProperty(name="Primary Count", default=4, min=2, max=12)

    def execute(self, context):
        obj = _new_mesh_object(context, f"Collector_{self.primary_count}to1")
        s = obj.exhaust_collector
        s.is_collector = True
        s.primary_count = self.primary_count
        rebuild_collector_object(obj)
        return {'FINISHED'}


class EXHAUST_OT_AddYPipe(bpy.types.Operator):
    bl_idname = "exhaust.add_y_pipe"
    bl_label = "Add Y-Pipe"
    bl_description = "Create a Y-pipe: two pipes merging into one, with a smooth shared merge shape"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = _new_mesh_object(context, "Exhaust_Y_Pipe")
        s = obj.exhaust_y_pipe
        s.is_y_pipe = True
        rebuild_y_pipe_object(obj)
        return {'FINISHED'}


class EXHAUST_OT_AddXPipe(bpy.types.Operator):
    bl_idname = "exhaust.add_x_pipe"
    bl_label = "Add X-Pipe"
    bl_description = "Create an X-pipe: two separate pipes that cross and share exhaust flow through a central crossover"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = _new_mesh_object(context, "Exhaust_X_Pipe")
        s = obj.exhaust_x_pipe
        s.is_x_pipe = True
        rebuild_x_pipe_object(obj)
        return {'FINISHED'}


class EXHAUST_OT_AddHPipe(bpy.types.Operator):
    bl_idname = "exhaust.add_h_pipe"
    bl_label = "Add H-Pipe"
    bl_description = "Create an H-pipe: two parallel pipes joined by a crossover tube that lets them share exhaust flow"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = _new_mesh_object(context, "Exhaust_H_Pipe")
        s = obj.exhaust_h_pipe
        s.is_h_pipe = True
        rebuild_h_pipe_object(obj)
        return {'FINISHED'}


class EXHAUST_OT_AddPieCut(bpy.types.Operator):
    bl_idname = "exhaust.add_pie_cut"
    bl_label = "Add Pie-Cut Bend"
    bl_description = "Create a bend made from straight mitered tube sections welded together, instead of one smooth mandrel bend"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = _new_mesh_object(context, "Exhaust_Pie_Cut_Bend")
        s = obj.exhaust_pie_cut
        s.is_pie_cut = True
        rebuild_pie_cut_object(obj)
        return {'FINISHED'}

class EXHAUST_OT_AddReducer(bpy.types.Operator):
    bl_idname = "exhaust.add_reducer"
    bl_label = "Add Reducer / Expander"
    bl_description = "Create a simple cone-shaped transition between two different pipe diameters"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = _new_mesh_object(context, "Exhaust_Reducer")
        obj.exhaust_reducer.is_reducer = True
        rebuild_reducer_object(obj)
        return {'FINISHED'}


class EXHAUST_OT_AddSegment(bpy.types.Operator):
    bl_idname = "exhaust.add_segment"
    bl_label = "Add Segment"
    bl_description = "Add a new straight or bend piece to the end of this pipe's segment list"
    bl_options = {'UNDO'}

    kind: bpy.props.EnumProperty(items=[('STRAIGHT', "Straight", ""), ('BEND', "Bend", "")], default='STRAIGHT')

    def execute(self, context):
        obj = context.object
        if not obj or not obj.exhaust_route.is_route:
            return {'CANCELLED'}
        seg = obj.exhaust_route.segments.add()
        seg.kind = self.kind
        if self.kind == 'STRAIGHT':
            seg.length = 6.0 * INCH
        else:
            seg.radius = 3.0 * INCH
            seg.angle = 0.78539816339
            seg.resolution = 16
        obj.exhaust_route.active_segment = len(obj.exhaust_route.segments) - 1
        rebuild_route_object(obj)
        return {'FINISHED'}


class EXHAUST_OT_RemoveSegment(bpy.types.Operator):
    bl_idname = "exhaust.remove_segment"
    bl_label = "Remove Segment"
    bl_description = "Delete the currently selected segment from this pipe"
    bl_options = {'UNDO'}

    index: bpy.props.IntProperty(default=-1)

    def execute(self, context):
        obj = context.object
        if not obj or not obj.exhaust_route.is_route:
            return {'CANCELLED'}
        s = obj.exhaust_route
        idx = self.index if self.index >= 0 else s.active_segment
        if 0 <= idx < len(s.segments):
            s.segments.remove(idx)
            s.active_segment = max(0, min(idx, len(s.segments) - 1))
            rebuild_route_object(obj)
        return {'FINISHED'}


class EXHAUST_OT_MoveSegment(bpy.types.Operator):
    bl_idname = "exhaust.move_segment"
    bl_label = "Move Segment"
    bl_description = "Move the currently selected segment earlier or later in this pipe's segment order"
    bl_options = {'UNDO'}

    direction: bpy.props.IntProperty(default=1)

    def execute(self, context):
        obj = context.object
        if not obj or not obj.exhaust_route.is_route:
            return {'CANCELLED'}
        s = obj.exhaust_route
        i = s.active_segment
        j = i + self.direction
        if 0 <= i < len(s.segments) and 0 <= j < len(s.segments):
            s.segments.move(i, j)
            s.active_segment = j
            rebuild_route_object(obj)
        return {'FINISHED'}


class EXHAUST_OT_Rebuild(bpy.types.Operator):
    bl_idname = "exhaust.rebuild"
    bl_label = "Rebuild Exhaust Object"
    bl_description = "Rebuild this object's mesh from its current settings -- use this if the shape ever looks out of date"

    def execute(self, context):
        rebuild_any(context.object)
        return {'FINISHED'}


class EXHAUST_OT_SnapRouteToCollector(bpy.types.Operator):
    bl_idname = "exhaust.snap_route_to_collector"
    bl_label = "Snap Route Start to Collector Outlet"
    bl_description = "Move this pipe so its start lines up exactly with the selected Collector's outlet"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        route = context.view_layer.objects.active
        if not route or not getattr(route, 'exhaust_route', None) or not route.exhaust_route.is_route:
            self.report({'ERROR'}, "Make the Exhaust Route active")
            return {'CANCELLED'}
        if route.parent and getattr(route.parent, 'exhaust_header', None) and route.parent.exhaust_header.is_header:
            self.report({'ERROR'}, "Header primaries are assigned to collector inlet ports, not the collector outlet")
            return {'CANCELLED'}
        collectors = [o for o in context.selected_objects if o != route and getattr(o, 'exhaust_collector', None) and o.exhaust_collector.is_collector]
        if len(collectors) != 1:
            self.report({'ERROR'}, "Select this Route and exactly one Collector; keep the Route active")
            return {'CANCELLED'}
        collector = collectors[0]
        if not _align_object_start_to_target_end(route, collector):
            self.report({'ERROR'}, "Collector outlet connector metadata is unavailable")
            return {'CANCELLED'}
        rod = float(route.get('exhaust_connector_start_od', route.exhaust_route.outside_diameter))
        cod = float(collector.get('exhaust_connector_end_od', collector.exhaust_collector.outlet_od))
        if abs(rod-cod) > 0.010 * INCH:
            self.report({'WARNING'}, f"Snapped; connector OD differs by {abs(rod-cod)/INCH:.3f} in")
        return {'FINISHED'}


class EXHAUST_OT_AddRouteFromCollector(bpy.types.Operator):
    bl_idname = "exhaust.add_route_from_collector"
    bl_label = "Add Route from Collector Outlet"
    bl_description = "Create a new pipe starting exactly at this Collector's outlet, matching its connection hardware"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        collector = context.view_layer.objects.active
        if not collector or not getattr(collector, 'exhaust_collector', None) or not collector.exhaust_collector.is_collector:
            self.report({'ERROR'}, "Make a Collector active")
            return {'CANCELLED'}
        # Save collector because creating the route changes selection/active object.
        end_spec = collector_outlet_end_treatment_spec(collector.exhaust_collector)
        route = _new_mesh_object(context, "Exhaust_Route")
        mating_od = collector.exhaust_collector.outlet_od if end_spec.get('type') == 'SLIP_SOCKET' else end_spec['od']
        _initialize_default_route(route, mating_od)
        _copy_collector_connection_to_route_start(collector, route)
        if not _align_object_start_to_target_end(route, collector):
            self.report({'ERROR'}, "Could not read Collector outlet connector")
            return {'CANCELLED'}
        route["exhaust_attached_collector"] = collector.name
        return {'FINISHED'}


class EXHAUST_OT_SnapStartToEnd(bpy.types.Operator):
    bl_idname = "exhaust.snap_start_to_end"
    bl_label = "Snap Active Start to Other End"
    bl_description = "Move the active object so its start end lines up exactly with the other selected object's finish end"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        mover = context.view_layer.objects.active
        targets = [o for o in context.selected_objects if o != mover]
        if not mover or len(targets) != 1:
            self.report({'ERROR'}, "Select exactly two objects; make the object to move active")
            return {'CANCELLED'}
        target = targets[0]
        if not _align_object_start_to_target_end(mover, target):
            self.report({'ERROR'}, "Both objects need Exhaust Fabricator connector metadata")
            return {'CANCELLED'}
        return {'FINISHED'}


CLASSES = (
    EXHAUST_OT_AddHeader,
    EXHAUST_OT_HeaderApplyCount,
    EXHAUST_OT_HeaderArrangeStarts,
    EXHAUST_OT_HeaderRefresh,
    EXHAUST_OT_HeaderSelectPrimary,
    EXHAUST_OT_SelectHeaderOwner,
    EXHAUST_OT_HeaderSetTargetFromActive,
    EXHAUST_OT_HeaderCopyActiveLayout,
    EXHAUST_OT_HeaderAssignCollector,
    EXHAUST_OT_HeaderClearCollector,
    EXHAUST_OT_HeaderAddKeepouts,
    EXHAUST_OT_HeaderRemoveKeepout,
    EXHAUST_OT_HeaderClearKeepouts,
    EXHAUST_OT_HeaderAutoMapCollector,
    EXHAUST_OT_HeaderAlignCollectorToActive,
    EXHAUST_OT_HeaderSelectCollector,
    EXHAUST_OT_HeaderCreateResetGuides,
    EXHAUST_OT_HeaderGuidesToCurrent,
    EXHAUST_OT_HeaderSelectActiveGuide,
    EXHAUST_OT_HeaderRemoveGuides,
    EXHAUST_OT_HeaderSolveActivePrimary,
    EXHAUST_OT_HeaderCompleteActivePrimary,
    EXHAUST_OT_HeaderSolveAllPrimaries,
    EXHAUST_OT_SegmentRow,
    EXHAUST_OT_AddRoute,
    EXHAUST_OT_AddCollector,
    EXHAUST_OT_AddYPipe,
    EXHAUST_OT_AddXPipe,
    EXHAUST_OT_AddHPipe,
    EXHAUST_OT_AddPieCut,
    EXHAUST_OT_AddReducer,
    EXHAUST_OT_AddSegment,
    EXHAUST_OT_RemoveSegment,
    EXHAUST_OT_MoveSegment,
    EXHAUST_OT_Rebuild,
    EXHAUST_OT_SnapRouteToCollector,
    EXHAUST_OT_AddRouteFromCollector,
    EXHAUST_OT_SnapStartToEnd,
)
