import math
import bpy
from .collision import primary_collision_pairs
from .properties import (
    INCH, header_primary_objects, header_target_collector, collector_inlet_connectors,
    header_collector_status,
)
from .keepouts import keepout_names, keepout_objects
from .guide_splines import guide_objects


def _header_settings(context):
    """Return the active object's Header PropertyGroup, or None if not a Header."""
    obj = context.object
    hs = getattr(obj, "exhaust_header", None)
    return hs if (hs and hs.is_header) else None


def _header_with_target(context):
    """Return (header_settings, assigned_collector) for the active Header, or (hs_or_None, None)."""
    hs = _header_settings(context)
    if hs is None:
        return None, None
    return hs, header_target_collector(context.object)


class EXHAUST_UL_Segments(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        seg = item
        row = layout.row(align=True)
        icon_name = 'IPO_LINEAR' if seg.kind == 'STRAIGHT' else 'CURVE_BEZCURVE'
        if seg.kind == 'BEND' and getattr(seg, 'bend_style', 'MANDREL') == 'PIE_CUT':
            item_name = "Pie Bend"
        else:
            item_name = seg.kind.title()
        op = row.operator(
            "exhaust.segment_row",
            text=f"{index + 1}. {item_name}",
            icon=icon_name,
            emboss=False,
        )
        op.object_name = data.id_data.name
        op.index = index
        if seg.kind == 'STRAIGHT':
            row.prop(seg, "length", text="")
        else:
            row.prop(seg, "angle", text="")


class EXHAUST_PT_Main(bpy.types.Panel):
    bl_label = "Exhaust Fabricator"
    bl_idname = "EXHAUST_PT_main"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'

    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)
        col.operator("exhaust.add_header", text="Add Header", icon='MOD_ARRAY')
        col.operator("exhaust.add_route", icon='CURVE_DATA')
        col.operator("exhaust.add_reducer", icon='MOD_SIMPLEDEFORM')
        col.operator("exhaust.add_collector", text="Add Collector", icon='MESH_CONE')
        col.operator("exhaust.add_y_pipe", text="Add Y-Pipe", icon='MOD_BOOLEAN')
        col.operator("exhaust.add_x_pipe", text="Add X-Pipe", icon='MOD_BOOLEAN')
        col.operator("exhaust.add_h_pipe", text="Add H-Pipe", icon='MOD_BOOLEAN')

        if len(context.selected_objects) == 2:
            layout.operator("exhaust.snap_start_to_end", icon='SNAP_ON')


class EXHAUST_PT_Selected(bpy.types.Panel):
    bl_label = "Selected Exhaust Object"
    bl_idname = "EXHAUST_PT_selected"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_main"

    def draw(self, context):
        layout = self.layout
        obj = context.object
        if not obj:
            layout.label(text="Select an Exhaust Fabricator object")
            return

        header = getattr(obj, "exhaust_header", None)
        route = getattr(obj, "exhaust_route", None)
        collector = getattr(obj, "exhaust_collector", None)
        y_pipe = getattr(obj, "exhaust_y_pipe", None)
        x_pipe = getattr(obj, "exhaust_x_pipe", None)
        h_pipe = getattr(obj, "exhaust_h_pipe", None)
        pie_cut = getattr(obj, "exhaust_pie_cut", None)
        reducer = getattr(obj, "exhaust_reducer", None)

        if header and header.is_header:
            refresh = layout.row(align=True)
            refresh.label(text="Header / Primary Set")
            refresh.operator("exhaust.header_refresh", text="", icon='FILE_REFRESH')
            layout.label(text="See the sections below for setup, routing, and diagnostics", icon='INFO')
        elif route and route.is_route:
            self.draw_route(layout, obj)
        elif collector and collector.is_collector:
            self.draw_collector(layout, obj)
        elif y_pipe and y_pipe.is_y_pipe:
            self.draw_y_pipe(layout, obj)
        elif x_pipe and x_pipe.is_x_pipe:
            self.draw_x_pipe(layout, obj)
        elif h_pipe and h_pipe.is_h_pipe:
            self.draw_h_pipe(layout, obj)
        elif pie_cut and pie_cut.is_pie_cut:
            self.draw_pie_cut(layout, obj)
        elif reducer and reducer.is_reducer:
            self.draw_reducer(layout, obj)
        elif obj.type == 'CURVE' and bool(obj.get('exhaust_primary_guide', False)):
            route_obj = obj.parent
            header_obj = route_obj.parent if route_obj else None
            hs = getattr(header_obj, 'exhaust_header', None) if header_obj else None
            layout.label(text="Header Primary Guide Spline", icon='CURVE_BEZCURVE')
            if hs and hs.is_header and route_obj:
                idx = int(route_obj.get('exhaust_header_primary_index', 0))
                layout.label(text=f"Primary P{idx+1}: {route_obj.name}")
                layout.label(text="Edit Mode: move/subdivide Bezier points to suggest routing")
                layout.label(text="Guide is soft; CLR, collisions and collector alignment stay hard")
                row = layout.row(align=True)
                row.operator('exhaust.header_solve_active_primary', text='Solve This Primary', icon='PLAY')
                row.operator('exhaust.select_header_owner', text='Back to Header', icon='FILE_PARENT')
            else:
                layout.label(text="Guide owner Header is missing", icon='ERROR')
        else:
            layout.label(text="Not an Exhaust Fabricator object")

    def draw_route_end_treatment(self, layout, s, prefix, title):
        box = layout.box()
        box.label(text=title, icon='CON_TRACKTO')
        type_prop = f"{prefix}_end_type"
        box.prop(s, type_prop, text="Type")
        treatment = getattr(s, type_prop)
        if treatment == 'PLAIN':
            box.label(text="Nominal tube OD — square cut")
            return

        base_od = max(0.0, s.outside_diameter)
        wall = max(0.0, min(s.wall_thickness, base_od * 0.499))
        if treatment == 'EXPANDED':
            prop = f"{prefix}_expanded_od"
            box.prop(s, prop, text="Expanded OD")
            if getattr(s, prop) <= base_od:
                row = box.row(); row.alert = True
                row.label(text="Expanded OD should exceed route OD", icon='ERROR')
        elif treatment == 'REDUCED':
            prop = f"{prefix}_reduced_od"
            box.prop(s, prop, text="Reduced OD")
            if getattr(s, prop) >= base_od:
                row = box.row(); row.alert = True
                row.label(text="Reduced OD should be below route OD", icon='ERROR')
            if getattr(s, prop) <= 2.0 * wall:
                row = box.row(); row.alert = True
                row.label(text="Reduced OD is too small for this wall", icon='ERROR')
        elif treatment == 'SLIP_SOCKET':
            clearance_prop = f"{prefix}_slip_clearance"
            box.prop(s, clearance_prop)
            clearance = max(0.0, getattr(s, clearance_prop))
            socket_id = base_od + clearance
            socket_od = socket_id + 2.0 * wall
            box.label(text=f"Socket ID: {socket_id:.4f} m")
            box.label(text=f"Computed Socket OD: {socket_od:.4f} m")

        box.prop(s, f"{prefix}_transition_length")
        collar_text = "Socket Depth" if treatment == 'SLIP_SOCKET' else "Straight End Length"
        box.prop(s, f"{prefix}_collar_length", text=collar_text)

    def draw_route_connection_hardware(self, layout, s, prefix, title):
        box = layout.box()
        box.label(text=title, icon='MOD_SOLIDIFY')
        type_prop = f"{prefix}_connection_type"
        box.prop(s, type_prop, text="Connection")
        kind = getattr(s, type_prop)
        if kind == 'NONE':
            box.label(text="No endpoint hardware")
            return

        base_od = max(0.0, s.outside_diameter)
        end_type = getattr(s, f"{prefix}_end_type", 'PLAIN')
        if end_type == 'EXPANDED':
            base_od = max(base_od, getattr(s, f"{prefix}_expanded_od"))
        elif end_type == 'REDUCED':
            base_od = min(base_od, getattr(s, f"{prefix}_reduced_od"))
        elif end_type == 'SLIP_SOCKET':
            wall = max(0.0, min(s.wall_thickness, base_od * 0.499))
            base_od = base_od + max(0.0, getattr(s, f"{prefix}_slip_clearance")) + 2.0 * wall

        if kind in {'VBAND', 'WELD_FLANGE'}:
            box.prop(s, f"{prefix}_flange_od")
            flange_od = getattr(s, f"{prefix}_flange_od")
            if flange_od <= base_od:
                row = box.row(); row.alert = True
                row.label(text="Flange OD should exceed exposed tube OD", icon='ERROR')

        if kind == 'VBAND':
            box.prop(s, f"{prefix}_vband_neck_length")
            box.prop(s, f"{prefix}_vband_taper_length")
            box.prop(s, f"{prefix}_vband_face_width")
            box.label(text="Integrated weld neck + tapered clamp lip")
        else:
            box.prop(s, f"{prefix}_flange_thickness")
            if kind == 'FLAT_2BOLT':
                box.prop(s, f"{prefix}_2bolt_height")
                box.prop(s, f"{prefix}_2bolt_width")
                box.prop(s, f"{prefix}_2bolt_spacing")
                box.prop(s, f"{prefix}_bolt_hole_diameter")
                box.prop(s, f"{prefix}_bolt_phase")
                box.label(text="A/B/C automotive two-ear outline")
                box.label(text="2 true through-holes in the flange plate")
            elif kind == 'FLAT_3BOLT':
                box.prop(s, f"{prefix}_3bolt_height")
                box.prop(s, f"{prefix}_3bolt_width")
                box.prop(s, f"{prefix}_bolt_hole_diameter")
                box.prop(s, f"{prefix}_bolt_phase")
                box.label(text="A/B rounded-triangle automotive outline")
                box.label(text="3 true through-holes in the flange plate")
            elif kind == 'WELD_FLANGE':
                box.label(text="Circular butt/weld flange")

    def draw_route(self, layout, obj):
        s = obj.exhaust_route
        header = obj.parent if obj.parent and getattr(obj.parent, "exhaust_header", None) and obj.parent.exhaust_header.is_header else None
        if header:
            hs = header.exhaust_header
            idx = int(obj.get("exhaust_header_primary_index", -1))
            target = hs.average_length if hs.target_mode == 'AVERAGE' else hs.target_length
            delta = s.centerline_length - target
            hbox = layout.box()
            hbox.label(text=f"Header Primary {idx + 1 if idx >= 0 else '?'}")
            row = hbox.row(); row.alert = abs(delta) > hs.length_tolerance + 1.0e-9
            row.label(text=f'Length Δ: {delta / INCH:+.3f}"')
            # Keep collector target feedback visible while editing the primary,
            # which is when it is most useful.
            try:
                target_items = header_collector_status(header)
                item = next((v for v in target_items if v["primary_index"] == idx), None)
            except Exception:
                item = None
            if item is not None:
                tr = hbox.row()
                tr.alert = (item["position_error"] > hs.collector_position_tolerance + 1e-9 or
                            item["angle_error"] > hs.collector_angle_tolerance + 1e-9)
                tr.label(text=f'Collector C{item["port_index"]+1}: {item["position_error"]/INCH:.3f}" / {math.degrees(item["angle_error"]):.1f}°')
            if header_target_collector(header) is not None:
                hbox.operator("exhaust.header_solve_active_primary", text="Regenerate This Primary")
            if obj.get('exhaust_solver_generated', False):
                hbox.label(text=f'Assisted ΔL: {float(obj.get("exhaust_solver_length_error",0.0))/INCH:+.3f}"')
            hbox.operator("exhaust.select_header_owner", text="Back to Header")
        layout.label(text="Route", icon='CURVE_DATA')
        layout.prop(s, "outside_diameter")
        layout.prop(s, "wall_thickness")
        layout.prop(s, "profile_segments")

        if header:
            interfaces = layout.box()
            interfaces.label(text="Header Primary Interfaces", icon='LINKED')
            interfaces.label(text="Start: Shared Header Flange", icon='MESH_GRID')
            interfaces.label(text="Finish: Assigned Collector Inlet", icon='MESH_CONE')
            interfaces.label(text="End treatments / hardware are owned by the Header and Collector")
        else:
            ends = layout.box()
            ends.label(text="Route End Treatments", icon='MOD_SIMPLEDEFORM')
            self.draw_route_end_treatment(ends, s, 'start', "Start End")
            self.draw_route_end_treatment(ends, s, 'end', "Finish End")
            if s.start_end_type != 'PLAIN' or s.end_end_type != 'PLAIN':
                ends.prop(s, "end_treatment_segments")
                ends.label(text="Treatments are part of the same route mesh")

            hardware = layout.box()
            hardware.label(text="Endpoint Connection Hardware", icon='MOD_SOLIDIFY')
            self.draw_route_connection_hardware(hardware, s, 'start', "Start Connection")
            self.draw_route_connection_hardware(hardware, s, 'end', "Finish Connection")
            if s.start_connection_type != 'NONE' or s.end_connection_type != 'NONE':
                hardware.prop(s, "hardware_profile_segments")
                hardware.label(text="Hardware is attached to the exposed Route end")

            attach = layout.box()
            attach.label(text="Collector Attachment", icon='SNAP_ON')
            attach.operator("exhaust.snap_route_to_collector", text="Snap Start to Selected Collector", icon='SNAP_ON')
            attach.label(text="Select this Route + one Collector; keep Route active")

        guide_box = layout.box()
        guide_box.label(text="Viewport Guides", icon='OVERLAY')
        guide_box.prop(s, "show_segment_seams")
        any_pie_guides = any(
            seg.kind == 'BEND' and getattr(seg, 'bend_style', 'MANDREL') == 'PIE_CUT' and getattr(seg, 'show_pie_weld_seams', True)
            for seg in s.segments
        )
        if s.show_segment_seams or any_pie_guides:
            guide_box.prop(s, "seam_guide_offset")
            guide_box.label(text="Viewport only — never rendered/exported")
        layout.separator()
        layout.label(text=f"Centerline Length: {s.centerline_length:.4f} m")

        layout.template_list("EXHAUST_UL_Segments", "", s, "segments", s, "active_segment", rows=4)
        row = layout.row(align=True)
        op = row.operator("exhaust.add_segment", text="Straight", icon='ADD'); op.kind = 'STRAIGHT'
        op = row.operator("exhaust.add_segment", text="Bend", icon='ADD'); op.kind = 'BEND'
        row = layout.row(align=True)
        row.operator("exhaust.remove_segment", text="Remove", icon='REMOVE')
        op = row.operator("exhaust.move_segment", text="Up", icon='TRIA_UP'); op.direction = -1
        op = row.operator("exhaust.move_segment", text="Down", icon='TRIA_DOWN'); op.direction = 1

        if 0 <= s.active_segment < len(s.segments):
            seg = s.segments[s.active_segment]
            box = layout.box()
            box.prop(seg, "kind")
            if seg.kind == 'STRAIGHT':
                box.prop(seg, "length")
            else:
                box.prop(seg, "bend_style")
                if seg.bend_style == 'PIE_CUT':
                    box.prop(seg, "radius", text="Equivalent CLR")
                    box.prop(seg, "angle")
                    box.prop(seg, "clocking")
                    box.prop(seg, "pie_sections")
                    box.prop(seg, "show_pie_weld_seams")
                    sections = max(2, int(seg.pie_sections))
                    weld_angle = abs(seg.angle) / max(1, sections - 1)
                    section_length = 2.0 * max(0.0, seg.radius) * math.sin(weld_angle * 0.5)
                    box.label(text=f"Weld seams: {sections - 1}")
                    box.label(text=f"Angle per weld: {math.degrees(weld_angle):.3f}°")
                    if math.degrees(weld_angle) > 75.0:
                        warn = box.row()
                        warn.alert = True
                        warn.label(text="Very steep miter — add more pie sections", icon='ERROR')
                    box.label(text=f"Section C/L length: {section_length:.4f} m")
                    box.label(text=f"Pie bend C/L length: {section_length * sections:.4f} m")
                else:
                    box.prop(seg, "radius", text="CLR")
                    box.prop(seg, "angle")
                    box.prop(seg, "clocking")
                    box.prop(seg, "resolution")
        layout.operator("exhaust.rebuild", icon='FILE_REFRESH')

        if header is not None:
            comp = layout.row()
            comp.enabled = header_target_collector(header) is not None
            comp.operator("exhaust.header_complete_active_primary", text="Complete to Collector", icon='TRACKING_FORWARDS')
            if header_target_collector(header) is None:
                layout.label(text="Assign a Collector on the Header to enable this", icon='INFO')

    def draw_collector(self, layout, obj):
        s = obj.exhaust_collector
        layout.label(text="Radially Symmetric Collector", icon='MESH_CONE')
        layout.prop(s, "primary_count")
        layout.prop(s, "primary_od")
        layout.prop(s, "outlet_od")
        layout.prop(s, "wall_thickness")
        layout.prop(s, "collector_length")
        layout.prop(s, "transition_length")
        layout.prop(s, "outlet_length")
        layout.separator()
        layout.prop(s, "auto_radial_spread")
        if s.auto_radial_spread:
            layout.prop(s, "radial_gap")
            layout.label(text=f"Center Radius: {s.computed_radial_spread:.4f} m")
        else:
            layout.prop(s, "radial_spread")
        layout.prop(s, "radial_phase")
        layout.separator()
        layout.prop(s, "branch_samples")
        layout.prop(s, "transition_segments")
        layout.prop(s, "profile_segments")
        layout.label(text=f"Common body radial samples: {s.primary_count * max(12, s.profile_segments)}")
        layout.label(text="Automatic non-overlapping central merge core")
        layout.label(text="Tangent-trimmed radial merge → round outlet")
        layout.label(text="Radial symmetry: equal angular spacing")

        outlet = layout.box()
        outlet.label(text="Collector Outlet Fabrication", icon='MOD_SIMPLEDEFORM')
        outlet.prop(s, "outlet_end_type", text="Outlet End")
        if s.outlet_end_type == 'EXPANDED':
            outlet.prop(s, "outlet_expanded_od")
        elif s.outlet_end_type == 'REDUCED':
            outlet.prop(s, "outlet_reduced_od")
        elif s.outlet_end_type == 'SLIP_SOCKET':
            outlet.prop(s, "outlet_slip_clearance")
            socket_id = s.outlet_od + max(0.0, s.outlet_slip_clearance)
            socket_od = socket_id + 2.0 * max(0.0, s.wall_thickness)
            outlet.label(text=f"Socket ID: {socket_id:.4f} m")
            outlet.label(text=f"Computed Socket OD: {socket_od:.4f} m")
        if s.outlet_end_type != 'PLAIN':
            outlet.prop(s, "outlet_transition_length")
            outlet.prop(s, "outlet_collar_length", text="Socket Depth" if s.outlet_end_type == 'SLIP_SOCKET' else "Straight End Length")
            outlet.prop(s, "outlet_treatment_segments")

        conn = layout.box()
        conn.label(text="Collector Outlet Connection", icon='MOD_SOLIDIFY')
        conn.prop(s, "outlet_connection_type", text="Connection")
        kind = s.outlet_connection_type
        if kind in {'VBAND', 'WELD_FLANGE'}:
            conn.prop(s, "outlet_flange_od")
        if kind == 'VBAND':
            conn.prop(s, "outlet_vband_neck_length")
            conn.prop(s, "outlet_vband_taper_length")
            conn.prop(s, "outlet_vband_face_width")
        elif kind != 'NONE':
            conn.prop(s, "outlet_flange_thickness")
            if kind == 'FLAT_2BOLT':
                conn.prop(s, "outlet_2bolt_height")
                conn.prop(s, "outlet_2bolt_width")
                conn.prop(s, "outlet_2bolt_spacing")
                conn.prop(s, "outlet_bolt_hole_diameter")
                conn.prop(s, "outlet_bolt_phase")
            elif kind == 'FLAT_3BOLT':
                conn.prop(s, "outlet_3bolt_height")
                conn.prop(s, "outlet_3bolt_width")
                conn.prop(s, "outlet_bolt_hole_diameter")
                conn.prop(s, "outlet_bolt_phase")

        if kind != 'NONE':
            conn.prop(s, "outlet_hardware_profile_segments")

        actions = layout.box()
        actions.label(text="Outlet Routing", icon='CURVE_DATA')
        actions.operator("exhaust.add_route_from_collector", text="Add Route from Outlet", icon='CURVE_DATA')
        actions.label(text="Creates a normal Route aligned to the exposed collector outlet")
        layout.operator("exhaust.rebuild", icon='FILE_REFRESH')


    def draw_y_pipe(self, layout, obj):
        s = obj.exhaust_y_pipe
        layout.label(text="2 → 1 Automotive Y-Pipe", icon='MOD_BOOLEAN')
        layout.prop(s, "topology")

        # Tangent/side-entry is inherently asymmetric in routing even when the
        # tube diameters happen to match, so its controls are presented directly.
        if s.topology != 'TANGENT':
            layout.prop(s, "symmetric")

        dims = layout.box()
        dims.label(text="Tube Dimensions")
        if s.symmetric and s.topology != 'TANGENT':
            dims.prop(s, "inlet_a_od", text="Inlet OD")
        else:
            row = dims.row(align=True)
            row.prop(s, "inlet_a_od", text="Inlet A OD")
            row.prop(s, "inlet_b_od", text="Inlet B OD")
        dims.prop(s, "outlet_od")
        dims.prop(s, "wall_thickness")

        routing = layout.box()
        routing.label(text="Branch Routing")
        if s.topology == 'PARALLEL':
            routing.prop(s, "inlet_spacing")
            routing.label(text="Both inlet tangents remain parallel to the outlet")
        elif s.topology == 'TANGENT':
            routing.prop(s, "main_branch")
            if s.main_branch == 'A':
                routing.prop(s, "inlet_b_angle", text="Side Branch B Angle")
            else:
                routing.prop(s, "inlet_a_angle", text="Side Branch A Angle")
            routing.prop(s, "outlet_bias")
        elif s.topology == 'CUSTOM':
            routing.prop(s, "inlet_spacing")
            row = routing.row(align=True)
            row.prop(s, "inlet_a_angle", text="A Entry Angle")
            row.prop(s, "inlet_b_angle", text="B Entry Angle")
            routing.prop(s, "straight_fraction")
            routing.prop(s, "custom_outlet_bias")
        else:
            if s.symmetric:
                routing.prop(s, "inlet_a_angle", text="Branch Angle")
            else:
                row = routing.row(align=True)
                row.prop(s, "inlet_a_angle", text="A Angle")
                row.prop(s, "inlet_b_angle", text="B Angle")
            if s.topology == 'CLASSIC':
                routing.prop(s, "straight_fraction")

        lengths = layout.box()
        lengths.label(text="Lengths")
        lengths.prop(s, "branch_length")
        lengths.prop(s, "merge_length")
        lengths.prop(s, "outlet_length")

        info = layout.box()
        info.label(text=f"Computed inlet spacing: {s.computed_inlet_spacing:.4f} m")
        info.label(text=f"Outlet center offset: {s.computed_outlet_offset:.4f} m")
        if s.topology == 'SWEPT':
            info.label(text="Smooth curved branches → shared saddle merge")
        elif s.topology == 'CLASSIC':
            info.label(text="Straight-leg approach → compact terminal sweep")
        elif s.topology == 'PARALLEL':
            info.label(text="Parallel inlets → long divider-style merge")
        elif s.topology == 'TANGENT':
            info.label(text="Dominant run + side-entry branch → biased outlet")
        elif s.topology == 'FORMED':
            info.label(text="Low-curvature formed shell / organic Y")
        else:
            info.label(text="Custom spacing, angles, straightness, and outlet bias")
        info.label(text="Single shared-saddle manifold mesh")

        resolution = layout.box()
        resolution.label(text="Resolution")
        resolution.prop(s, "branch_samples")
        resolution.prop(s, "transition_segments")
        resolution.prop(s, "profile_segments")
        layout.operator("exhaust.rebuild", icon='FILE_REFRESH')


    def draw_x_pipe(self, layout, obj):
        s = obj.exhaust_x_pipe
        layout.label(text="2 → 2 Automotive X-Pipe", icon='MOD_BOOLEAN')
        layout.prop(s, "topology")
        layout.prop(s, "symmetric")

        if s.symmetric:
            layout.prop(s, "pipe_a_od", text="Pipe OD")
        else:
            row = layout.row(align=True)
            row.prop(s, "pipe_a_od", text="Pipe A OD")
            row.prop(s, "pipe_b_od", text="Pipe B OD")
        layout.prop(s, "wall_thickness")

        spacing = layout.box()
        spacing.label(text="Port Layout")
        spacing.prop(s, "inlet_spacing")
        spacing.prop(s, "outlet_spacing")
        spacing.label(text=f"Effective inlet spacing: {s.computed_inlet_spacing:.4f} m")
        spacing.label(text=f"Effective outlet spacing: {s.computed_outlet_spacing:.4f} m")

        if s.topology != 'PARALLEL':
            angles = layout.box()
            angles.label(text="Approach / Departure Angles")
            if s.symmetric:
                angles.prop(s, "inlet_a_angle", text="Inlet Angle")
                angles.prop(s, "outlet_a_angle", text="Outlet Angle")
            else:
                row = angles.row(align=True)
                row.prop(s, "inlet_a_angle", text="In A")
                row.prop(s, "inlet_b_angle", text="In B")
                row = angles.row(align=True)
                row.prop(s, "outlet_a_angle", text="Out A")
                row.prop(s, "outlet_b_angle", text="Out B")

        shape = layout.box()
        shape.label(text="Crossover Geometry")
        shape.prop(s, "inlet_branch_length")
        shape.prop(s, "crossover_length")
        shape.prop(s, "outlet_branch_length")
        shape.prop(s, "crossover_opening")
        if s.topology == 'CUSTOM':
            shape.prop(s, "custom_profile")
        shape.prop(s, "plane_rotation")

        res = layout.box()
        res.label(text="Resolution")
        res.prop(s, "branch_samples")
        res.prop(s, "crossover_segments")
        res.prop(s, "profile_segments")

        layout.label(text="Four ports → shared crossover cavity → four ports")
        layout.label(text="Outer/inner tangencies preserve wall thickness")
        layout.label(text="Single welded manifold mesh")
        layout.operator("exhaust.rebuild", icon='FILE_REFRESH')


    def draw_h_pipe(self, layout, obj):
        s = obj.exhaust_h_pipe
        layout.label(text="H-Pipe", icon='MOD_BOOLEAN')

        main = layout.box()
        main.label(text="Main Pipes")
        main.prop(s, "symmetric")
        main.prop(s, "pipe_a_od")
        if not s.symmetric:
            main.prop(s, "pipe_b_od")
        main.prop(s, "wall_thickness")
        main.prop(s, "main_length")
        main.prop(s, "main_spacing")

        cross = layout.box()
        cross.label(text="Balance Crossover")
        cross.prop(s, "crossover_od")
        cross.prop(s, "crossover_position")
        cross.prop(s, "crossover_angle")
        cross.prop(s, "junction_blend_length")
        cross.prop(s, "plane_rotation")
        cross.label(text=f"Tap A: {s.computed_tap_a:.4f} m")
        cross.label(text=f"Tap B: {s.computed_tap_b:.4f} m")
        cross.label(text=f"Clear crossover: {s.computed_crossover_length:.4f} m")

        res = layout.box()
        res.label(text="Resolution")
        res.prop(s, "main_segments")
        res.prop(s, "junction_segments")
        res.prop(s, "profile_segments")

        layout.label(text="Saddle-cut main pipes + hollow crossover")
        layout.label(text="Single connected manifold shell")
        layout.operator("exhaust.rebuild", icon='FILE_REFRESH')


    def draw_pie_cut(self, layout, obj):
        s = obj.exhaust_pie_cut
        layout.label(text="Fabrication Pie-Cut Bend", icon='MOD_SIMPLEDEFORM')

        dims = layout.box()
        dims.label(text="Tube")
        dims.prop(s, "outside_diameter")
        dims.prop(s, "wall_thickness")

        bend = layout.box()
        bend.label(text="Pie Geometry")
        bend.prop(s, "bend_angle")
        bend.prop(s, "pie_sections")
        bend.prop(s, "equivalent_clr")
        bend.prop(s, "clocking")
        bend.label(text=f"Weld seams: {max(1, s.pie_sections - 1)}")
        weld_deg = math.degrees(s.computed_weld_angle)
        bend.label(text=f"Angle per weld: {weld_deg:.3f}°")
        if weld_deg > 75.0:
            warn = bend.row()
            warn.alert = True
            warn.label(text="Very steep miter — add more pie sections", icon='ERROR')
        bend.label(text=f"Section C/L length: {s.computed_section_length:.4f} m")
        bend.label(text=f"Total C/L length: {s.computed_centerline_length:.4f} m")

        guides = layout.box()
        guides.label(text="Viewport Guides", icon='OVERLAY')
        guides.prop(s, "show_weld_seams")
        if s.show_weld_seams:
            guides.prop(s, "seam_guide_offset")
            guides.label(text="Miter weld ellipses — viewport only")

        res = layout.box()
        res.label(text="Resolution")
        res.prop(s, "profile_segments")

        layout.label(text="Straight cylindrical sections + shared miter seams")
        layout.label(text="One hollow manifold mesh with square inlet/outlet ports")
        layout.operator("exhaust.rebuild", icon='FILE_REFRESH')


    def draw_reducer(self, layout, obj):
        s = obj.exhaust_reducer
        layout.label(text="Reducer / Expander", icon='MOD_SIMPLEDEFORM')
        layout.prop(s, "inlet_od")
        layout.prop(s, "outlet_od")
        layout.prop(s, "length")
        layout.prop(s, "wall_thickness")
        layout.prop(s, "profile_segments")
        layout.operator("exhaust.rebuild", icon='FILE_REFRESH')


class EXHAUST_PT_HeaderSetup(bpy.types.Panel):
    bl_label = "Setup"
    bl_idname = "EXHAUST_PT_header_setup"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_selected"

    @classmethod
    def poll(cls, context):
        return _header_settings(context) is not None

    def draw(self, context):
        s = _header_settings(context)
        layout = self.layout
        layout.prop(s, "desired_primary_count")
        layout.operator("exhaust.header_apply_count", text="Apply Count")
        layout.prop(s, "initial_primary_length")
        layout.prop(s, "port_spacing")
        layout.operator("exhaust.header_arrange_starts", text="Arrange Primary Starts")


class EXHAUST_PT_HeaderTube(bpy.types.Panel):
    bl_label = "Primary Tube"
    bl_idname = "EXHAUST_PT_header_tube"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_selected"

    @classmethod
    def poll(cls, context):
        return _header_settings(context) is not None

    def draw(self, context):
        s = _header_settings(context)
        layout = self.layout
        layout.prop(s, "primary_od")
        layout.prop(s, "wall_thickness")


class EXHAUST_PT_HeaderFlange(bpy.types.Panel):
    bl_label = "Shared Head Flange"
    bl_idname = "EXHAUST_PT_header_flange"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_selected"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return _header_settings(context) is not None

    def draw_header(self, context):
        self.layout.label(text="", icon='MESH_GRID')

    def draw(self, context):
        s = _header_settings(context)
        layout = self.layout
        layout.label(text="One generic flange spans every primary start")
        layout.prop(s, "flange_thickness")
        layout.prop(s, "flange_edge_margin")
        layout.prop(s, "flange_corner_radius")
        layout.prop(s, "flange_bore_clearance")
        layout.prop(s, "flange_corner_segments")
        layout.label(text=f'Overall Width: {s.flange_width / INCH:.3f}"')
        layout.label(text=f'Overall Height: {s.flange_height / INCH:.3f}"')
        if s.flange_plane_deviation > 0.001 * INCH:
            warn = layout.row(); warn.alert = True
            warn.label(text=f'Starts are not coplanar: {s.flange_plane_deviation / INCH:.3f}"', icon='ERROR')
            layout.label(text="Use Arrange Primary Starts or align the Route origins")


class EXHAUST_PT_HeaderLength(bpy.types.Panel):
    bl_label = "Equal-Length Analysis"
    bl_idname = "EXHAUST_PT_header_length"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_selected"

    @classmethod
    def poll(cls, context):
        return _header_settings(context) is not None

    def draw(self, context):
        s = _header_settings(context)
        layout = self.layout
        layout.prop(s, "target_mode")
        if s.target_mode == 'TARGET':
            layout.prop(s, "target_length")
        layout.prop(s, "length_tolerance")
        layout.label(text=f'Average: {s.average_length / INCH:.3f}"')
        layout.label(text=f'Shortest: {s.minimum_length / INCH:.3f}"')
        layout.label(text=f'Longest: {s.maximum_length / INCH:.3f}"')
        layout.label(text=f'Spread: {s.length_spread / INCH:.3f}"')
        status = layout.row()
        status.alert = not s.all_within_tolerance
        status.label(text="All within tolerance" if s.all_within_tolerance else "Outside tolerance")


class EXHAUST_PT_HeaderCollectorTarget(bpy.types.Panel):
    bl_label = "Collector Target"
    bl_idname = "EXHAUST_PT_header_collector_target"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_selected"

    @classmethod
    def poll(cls, context):
        return _header_settings(context) is not None

    def draw(self, context):
        obj = context.object
        s = _header_settings(context)
        layout = self.layout
        target = header_target_collector(obj)
        if target is None:
            if s.collector_target_name:
                warn = layout.row(); warn.alert = True
                warn.label(text=f"Missing target: {s.collector_target_name}", icon='ERROR')
            else:
                layout.label(text="No collector assigned")
            layout.label(text="Select Header + one Collector; keep Header active")
            layout.operator("exhaust.header_assign_collector", text="Assign Selected Collector")
            return

        row = layout.row(align=True)
        row.label(text=target.name, icon='MESH_DATA')
        row.operator("exhaust.header_select_collector", text="", icon='RESTRICT_SELECT_OFF')
        row.operator("exhaust.header_clear_collector", text="", icon='X')
        ports = collector_inlet_connectors(target)
        routes_for_target = header_primary_objects(obj)
        if len(ports) != len(routes_for_target):
            warn = layout.row(); warn.alert = True
            warn.label(text=f"Count mismatch: {len(routes_for_target)} primaries / {len(ports)} collector inlets", icon='ERROR')
        maprow = layout.row(align=True)
        maprow.prop(s, "collector_port_offset")
        maprow.prop(s, "collector_reverse_order", text="Reverse")
        layout.operator("exhaust.header_auto_map_collector", text="Auto Map Ports")
        layout.prop(s, "show_collector_targets")
        if s.show_collector_targets:
            layout.prop(s, "collector_target_guide_offset")
        tol = layout.column(align=True)
        tol.prop(s, "collector_position_tolerance")
        tol.prop(s, "collector_angle_tolerance")
        layout.operator("exhaust.header_align_collector_to_active", text="Align Collector to Active Primary")

        statuses = header_collector_status(obj)
        if statuses:
            layout.separator()
            layout.label(text="Primary Endpoint Alignment")
            for item in statuses:
                pos = item["position_error"]
                ang = item["angle_error"]
                row = layout.row(align=True)
                row.alert = (pos > s.collector_position_tolerance + 1e-9 or ang > s.collector_angle_tolerance + 1e-9)
                row.label(text=f'P{item["primary_index"]+1} → C{item["port_index"]+1}')
                row.label(text=f'{pos / INCH:.3f}"')
                row.label(text=f'{math.degrees(ang):.1f}°')
            max_pos = max(i["position_error"] for i in statuses)
            max_ang = max(i["angle_error"] for i in statuses)
            all_ok = (len(statuses) == len(routes_for_target) == len(ports) and
                      max_pos <= s.collector_position_tolerance + 1e-9 and
                      max_ang <= s.collector_angle_tolerance + 1e-9)
            row = layout.row(); row.alert = not all_ok
            row.label(text="All primary ends aligned" if all_ok else
                           f'Max error: {max_pos/INCH:.3f}" / {math.degrees(max_ang):.1f}°')


class EXHAUST_PT_HeaderAssistedRouting(bpy.types.Panel):
    bl_label = "Assisted Primary Routing"
    bl_idname = "EXHAUST_PT_header_assisted_routing"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_selected"

    @classmethod
    def poll(cls, context):
        return _header_with_target(context)[1] is not None

    def draw(self, context):
        obj = context.object
        s = _header_settings(context)
        layout = self.layout
        layout.label(text="Generates Straight + Mandrel Bend segments")
        layout.prop(s, "solver_clr")
        layout.prop(s, "solver_min_straight")
        layout.prop(s, "solver_max_bend_angle")
        layout.prop(s, "solver_max_dogleg_offset")
        layout.prop(s, "solver_dogleg_clocking")
        layout.prop(s, "solver_bend_resolution")
        layout.prop(s, "solver_quality")
        layout.prop(s, "solver_match_length")
        if s.solver_match_length:
            ref = s.average_length if s.target_mode == 'AVERAGE' else s.target_length
            layout.label(text=f'Routing length reference: {ref/INCH:.3f}"')
            layout.label(text=f'Shortest route inside ±{s.length_tolerance/INCH:.3f}" tolerance')
        else:
            layout.prop(s, "solver_shortest_route")
            if s.solver_shortest_route:
                layout.label(text="Objective: minimum feasible centerline length", icon='DRIVER_DISTANCE')
            else:
                layout.label(text="Objective: compact / gentle feasible route")

        layout.separator()
        use_group_search = (
            s.solver_avoid_primary_collisions or (s.solver_avoid_keepouts and keepout_names(obj))
            or s.solver_adjust_collector_phase
        )
        if use_group_search:
            layout.label(text="Generate All: global / transactional / all-or-nothing solve", icon='MOD_ARRAY')
        row = layout.row(align=True)
        row.operator("exhaust.header_solve_active_primary", text="Generate Active Primary")
        row.operator("exhaust.header_solve_all_primaries", text="Generate All Primaries")
        warn = layout.row(); warn.alert = True
        warn.label(text="Replaces generated primary segment layouts — Undo supported")

        active_routes = header_primary_objects(obj)
        if 0 <= s.active_primary < len(active_routes):
            ar = active_routes[s.active_primary]
            if ar.get('exhaust_solver_generated', False):
                result = layout.box()
                result.label(text=f"Last solution — P{s.active_primary+1}")
                objective = str(ar.get('exhaust_solver_objective', 'EQUAL_LENGTH'))
                if objective == 'SHORTEST':
                    result.label(text=f'Route length: {float(ar.exhaust_route.centerline_length)/INCH:.3f}"')
                    result.label(text="Objective: Shortest Possible Route")
                elif objective == 'FEASIBLE':
                    result.label(text=f'Route length: {float(ar.exhaust_route.centerline_length)/INCH:.3f}"')
                    result.label(text="Objective: Compact / Gentle")
                else:
                    result.label(text=f'Length Δ: {float(ar.get("exhaust_solver_length_error",0.0))/INCH:+.3f}"')
                pe = float(ar.get('exhaust_solver_position_error', -1.0))
                ae = float(ar.get('exhaust_solver_angle_error', -1.0))
                if pe >= 0.0 and ae >= 0.0:
                    result.label(text=f'Target error: {pe/INCH:.4f}" / {math.degrees(ae):.2f}°')
                result.label(text=f'Dogleg offset: {float(ar.get("exhaust_solver_dogleg_offset",0.0))/INCH:+.3f}"')
                used_clock = float(ar.get("exhaust_solver_dogleg_clocking_used", s.solver_dogleg_clocking))
                if abs(used_clock - s.solver_dogleg_clocking) > math.radians(0.05):
                    result.label(text=f'Auto clocking used: {math.degrees(used_clock):+.1f}°')
                min_clr = float(ar.get("exhaust_solver_min_primary_clearance", -1.0))
                if min_clr >= 0.0:
                    result.label(text=f'Min clearance margin: {min_clr/INCH:.3f}"')
                rejected = int(ar.get("exhaust_solver_collision_rejections", 0))
                if rejected > 0:
                    result.label(text=f'Collision candidates rejected: {rejected}')


class EXHAUST_PT_HeaderGuides(bpy.types.Panel):
    bl_label = "Primary Guide Splines"
    bl_idname = "EXHAUST_PT_header_guides"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_header_assisted_routing"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return _header_with_target(context)[1] is not None

    def draw(self, context):
        obj = context.object
        s = _header_settings(context)
        layout = self.layout
        layout.prop(s, "solver_use_guide_splines")
        if not s.solver_use_guide_splines:
            return
        layout.prop(s, "solver_guide_influence")
        guides = guide_objects(obj)
        routes_for_guides = header_primary_objects(obj)
        grow = layout.row(); grow.alert = (len(guides) != len(routes_for_guides))
        grow.label(text=f"Guides: {len(guides)} / {len(routes_for_guides)} primaries", icon=('CHECKMARK' if len(guides)==len(routes_for_guides) else 'ERROR'))
        rowg = layout.row(align=True)
        rowg.operator('exhaust.header_create_reset_guides', text='Create / Reset Guides', icon='CURVE_BEZCURVE')
        rowg.operator('exhaust.header_select_active_guide', text='Select Active', icon='RESTRICT_SELECT_OFF')
        layout.operator('exhaust.header_guides_to_current', text='Splines to Current', icon='CURVE_BEZCURVE')
        solve_row = layout.row(align=True)
        solve_row.operator('exhaust.header_solve_active_primary', text='Generate Active from Spline', icon='PLAY')
        if guides:
            layout.operator('exhaust.header_remove_guides', text='Remove Guides', icon='TRASH')
        layout.label(text="Solves only the active primary; all other primaries stay fixed")
        layout.label(text="Guide curves are editable, non-rendering soft routing hints")
        layout.label(text="Higher influence narrows the blind search around your sketch")


class EXHAUST_PT_HeaderPrimaryCollision(bpy.types.Panel):
    bl_label = "Primary Collision Avoidance"
    bl_idname = "EXHAUST_PT_header_primary_collision"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_header_assisted_routing"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return _header_with_target(context)[1] is not None

    def draw(self, context):
        obj = context.object
        s = _header_settings(context)
        layout = self.layout
        layout.prop(s, "solver_avoid_primary_collisions")
        if not s.solver_avoid_primary_collisions:
            return
        layout.prop(s, "solver_primary_clearance")
        layout.prop(s, "solver_auto_clocking_search")
        layout.label(text="Rejects tube-envelope overlap with other primaries", icon='CHECKMARK')
        current_pairs = primary_collision_pairs(header_primary_objects(obj), s.solver_primary_clearance)
        status_row = layout.row()
        status_row.alert = bool(current_pairs)
        if current_pairs:
            names = ", ".join(f"P{a+1}/P{b+1}" for a, b, _c in current_pairs[:3])
            suffix = " …" if len(current_pairs) > 3 else ""
            status_row.label(text=f"Current primary clearance violations: {len(current_pairs)} ({names}{suffix})", icon='ERROR')
        else:
            status_row.label(text="Current primary layout: clear", icon='CHECKMARK')


class EXHAUST_PT_HeaderPhase(bpy.types.Panel):
    bl_label = "Collector Phase Flexibility"
    bl_idname = "EXHAUST_PT_header_phase"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_header_assisted_routing"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return _header_with_target(context)[1] is not None

    def draw(self, context):
        obj = context.object
        s = _header_settings(context)
        layout = self.layout
        layout.prop(s, "solver_adjust_collector_phase")
        if not s.solver_adjust_collector_phase:
            return
        layout.prop(s, "solver_collector_phase_range")
        layout.prop(s, "solver_phase_auto_remap")
        target_phase_obj = header_target_collector(obj)
        phase_ports = collector_inlet_connectors(target_phase_obj) if target_phase_obj else []
        if phase_ports:
            half_pitch = math.pi / max(1, len(phase_ports))
            effective = min(float(s.solver_collector_phase_range), half_pitch)
            layout.label(text=f'Generate All searches ±{math.degrees(effective):.1f}° around current phase')
            layout.label(text=f'Current collector phase: {math.degrees(float(target_phase_obj.exhaust_collector.radial_phase)):.1f}°')
        layout.label(text="Generate Active Primary keeps collector phase fixed")
        if 'exhaust_solver_collector_phase_used' in obj:
            used = float(obj.get('exhaust_solver_collector_phase_used', 0.0))
            delta = float(obj.get('exhaust_solver_collector_phase_delta', 0.0))
            tried = int(obj.get('exhaust_solver_collector_phase_candidates', 0))
            layout.label(text=f'Last full solve: {math.degrees(used):.1f}° ({math.degrees(delta):+.1f}°), {tried} phase candidate(s)')


class EXHAUST_PT_HeaderKeepouts(bpy.types.Panel):
    bl_label = "Keep-Out Geometry — Experimental"
    bl_idname = "EXHAUST_PT_header_keepouts"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_header_assisted_routing"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return _header_with_target(context)[1] is not None

    def draw(self, context):
        obj = context.object
        s = _header_settings(context)
        layout = self.layout
        layout.prop(s, "solver_avoid_keepouts")
        if not s.solver_avoid_keepouts:
            return
        layout.prop(s, "solver_keepout_clearance")
        names = keepout_names(obj)
        objects = {o.name: o for o in keepout_objects(obj)}
        if not names:
            layout.label(text="No keep-out meshes assigned")
        for name in names:
            rowk = layout.row(align=True)
            missing = name not in objects
            rowk.alert = missing
            rowk.label(text=(f"Missing: {name}" if missing else name), icon=('ERROR' if missing else 'MESH_DATA'))
            op = rowk.operator("exhaust.header_remove_keepout", text="", icon='X')
            op.object_name = name
        addrow = layout.row(align=True)
        addrow.operator("exhaust.header_add_keepouts", text="Add Selected Meshes", icon='ADD')
        if names:
            addrow.operator("exhaust.header_clear_keepouts", text="Clear", icon='TRASH')
        layout.label(text="Select Header + mesh obstacles; keep Header active")
        layout.label(text="Evaluated mesh/modifier geometry is used for routing")


class EXHAUST_PT_HeaderSearchBudget(bpy.types.Panel):
    bl_label = "Search Budget"
    bl_idname = "EXHAUST_PT_header_search_budget"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_header_assisted_routing"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        hs, target = _header_with_target(context)
        if target is None:
            return False
        obj = context.object
        return bool(
            hs.solver_avoid_primary_collisions
            or (hs.solver_avoid_keepouts and keepout_names(obj))
            or hs.solver_adjust_collector_phase
        )

    def draw(self, context):
        s = _header_settings(context)
        layout = self.layout
        layout.prop(s, "solver_order_success_cap")
        layout.prop(s, "solver_max_solve_seconds")
        layout.label(text="Stops Generate All early once enough valid orders/phases are compared,")
        layout.label(text="or between attempts once the time budget is spent -- never mid-attempt,")
        layout.label(text="so a single search can exceed the budget by about one attempt's duration.")
        layout.label(text="Applied twice (fast search, then a slower fallback if needed), so a")
        layout.label(text="Header with no valid solution can take up to roughly 2x this long to fail")


class EXHAUST_PT_HeaderDiagnostics(bpy.types.Panel):
    bl_label = "Solver Diagnostics — Experimental"
    bl_idname = "EXHAUST_PT_header_diagnostics"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_header_assisted_routing"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return _header_with_target(context)[1] is not None

    def draw_header(self, context):
        s = _header_settings(context)
        self.layout.prop(s, "solver_profiling_enabled", text="")

    def draw(self, context):
        s = _header_settings(context)
        layout = self.layout
        if s.solver_profiling_enabled:
            if s.solver_profile_report:
                for line in s.solver_profile_report.splitlines():
                    layout.label(text=line)
                layout.label(text="Nested stages overlap their parent call; percentages need not sum to 100%")
            else:
                layout.label(text="Run Generate Active/All Primaries to capture a timing breakdown")
        else:
            layout.label(text="Timing capture is off; enable to profile the next solve")


class EXHAUST_PT_HeaderPrimaries(bpy.types.Panel):
    bl_label = "Primaries"
    bl_idname = "EXHAUST_PT_header_primaries"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Exhaust'
    bl_parent_id = "EXHAUST_PT_selected"

    @classmethod
    def poll(cls, context):
        return _header_settings(context) is not None

    def draw(self, context):
        obj = context.object
        s = _header_settings(context)
        layout = self.layout
        routes = header_primary_objects(obj)
        layout.label(text=f"{len(routes)} primaries")
        target = s.average_length if s.target_mode == 'AVERAGE' else s.target_length
        for i, route in enumerate(routes):
            length = float(route.exhaust_route.centerline_length)
            delta = length - target
            row = layout.row(align=True)
            row.alert = abs(delta) > s.length_tolerance + 1.0e-9
            op = row.operator("exhaust.header_select_primary", text=f"P{i+1}", emboss=True)
            op.index = i
            row.label(text=f'{length / INCH:.3f}"')
            row.label(text=f'Δ {delta / INCH:+.3f}"')
        controls = layout.row(align=True)
        controls.operator("exhaust.header_select_primary", text="Edit Active Primary")
        controls.operator("exhaust.header_target_from_active", text="Target = Active")
        layout.operator("exhaust.header_copy_active_layout", text="Copy Active Segment Layout to All")


CLASSES = (
    EXHAUST_UL_Segments, EXHAUST_PT_Main, EXHAUST_PT_Selected,
    EXHAUST_PT_HeaderSetup, EXHAUST_PT_HeaderTube, EXHAUST_PT_HeaderFlange,
    EXHAUST_PT_HeaderLength, EXHAUST_PT_HeaderCollectorTarget,
    EXHAUST_PT_HeaderAssistedRouting, EXHAUST_PT_HeaderGuides,
    EXHAUST_PT_HeaderPrimaryCollision, EXHAUST_PT_HeaderPhase,
    EXHAUST_PT_HeaderKeepouts, EXHAUST_PT_HeaderSearchBudget,
    EXHAUST_PT_HeaderDiagnostics, EXHAUST_PT_HeaderPrimaries,
)
