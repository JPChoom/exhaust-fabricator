"""Viewport-only graphical guides for Exhaust Fabricator.

All guides are drawn with Blender's GPU viewport API. They are not scene
objects, so they cannot render or be exported accidentally.
"""

import math
import bpy
import gpu
from gpu_extras.batch import batch_for_shader
from mathutils import Vector, Matrix

from .geometry import route_seam_frames, route_pie_cut_seam_loops, pie_cut_seam_loops
from .properties import header_collector_status

_DRAW_HANDLE = None
_RING_SEGMENTS = 64
# Inactive route boundaries are intentionally subdued. The active segment's
# START seam uses the established Exhaust Fabricator cyan/blue guide color.
_DEFAULT_SEAM_COLOR = (0.28, 0.28, 0.30, 0.90)
_ACTIVE_SEAM_COLOR = (0.10, 0.82, 1.00, 1.00)


def tag_view3d_redraw():
    """Redraw all 3D views after list selection or guide-setting changes."""
    wm = getattr(bpy.context, "window_manager", None)
    if wm is None:
        return
    for window in wm.windows:
        screen = getattr(window, "screen", None)
        if screen is None:
            continue
        for area in screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()


def _ring_line_vertices(center, normal, binormal, radius, matrix_world):
    """Return line-pair vertices for one circular seam ring in world space."""
    pts = []
    for i in range(_RING_SEGMENTS):
        a = 2.0 * math.pi * i / _RING_SEGMENTS
        local = center + (normal * math.cos(a) + binormal * math.sin(a)) * radius
        pts.append(matrix_world @ local)

    lines = []
    for i in range(_RING_SEGMENTS):
        lines.append(tuple(pts[i]))
        lines.append(tuple(pts[(i + 1) % _RING_SEGMENTS]))
    return lines


def _loop_line_vertices(loop, matrix_world):
    if not loop:
        return []
    pts = [matrix_world @ p for p in loop]
    lines = []
    for i in range(len(pts)):
        lines.append(tuple(pts[i]))
        lines.append(tuple(pts[(i + 1) % len(pts)]))
    return lines


def _draw_lines(vertices, color, width):
    if not vertices:
        return
    shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    batch = batch_for_shader(shader, 'LINES', {"pos": vertices})
    gpu.state.line_width_set(width)
    shader.bind()
    shader.uniform_float("color", color)
    batch.draw(shader)


def _target_ring_vertices(center, tangent, radius):
    t = Vector(tangent).normalized()
    ref = Vector((0.0, 0.0, 1.0))
    if abs(t.dot(ref)) > 0.92:
        ref = Vector((0.0, 1.0, 0.0))
    normal = t.cross(ref).normalized()
    binormal = t.cross(normal).normalized()
    return _ring_line_vertices(Vector(center), normal, binormal, radius, Matrix.Identity(4))


def _header_collector_target_vertices(scene):
    inactive = []
    active = []
    active_obj = getattr(bpy.context.view_layer.objects, "active", None)
    for header in scene.objects:
        hs = getattr(header, "exhaust_header", None)
        if not hs or not hs.is_header or not hs.show_collector_targets:
            continue
        try:
            statuses = header_collector_status(header)
        except Exception:
            statuses = []
        if not statuses:
            continue
        radius = max(1.0e-6, hs.primary_od * 0.5 + hs.collector_target_guide_offset)
        for item in statuses:
            verts = _target_ring_vertices(item["port_point"], item["port_tangent"], radius)
            # Add one direct endpoint-to-target line as a simple visual error vector.
            verts.extend([tuple(item["route_point"]), tuple(item["port_point"])])
            active_primary_obj = item.get("route")
            is_active = (
                item["primary_index"] == int(hs.active_primary) and
                (header == active_obj or active_primary_obj == active_obj)
            )
            (active if is_active else inactive).extend(verts)
    return inactive, active

def _draw_segment_seams():
    context = bpy.context
    scene = getattr(context, "scene", None)
    if scene is None:
        return

    inactive_vertices = []
    active_vertices = []
    target_inactive_vertices, target_active_vertices = _header_collector_target_vertices(scene)
    active_obj = getattr(context.view_layer.objects, "active", None)

    for obj in scene.objects:
        if obj.type != 'MESH' or not obj.visible_get():
            continue
        settings = getattr(obj, "exhaust_route", None)
        if not settings or not settings.is_route:
            continue

        mw = obj.matrix_world
        if settings.show_segment_seams:
            try:
                frames = route_seam_frames(settings, include_end=True)
            except Exception:
                frames = []
            radius = max(1.0e-6, settings.outside_diameter * 0.5 + settings.seam_guide_offset)
            active_index = int(settings.active_segment)
            for seam_index, (center, _tangent, normal, binormal) in enumerate(frames):
                verts = _ring_line_vertices(center, normal, binormal, radius, mw)
                if obj == active_obj and obj.select_get() and seam_index == active_index and active_index < len(settings.segments):
                    active_vertices.extend(verts)
                else:
                    inactive_vertices.extend(verts)

        # Pie-cut bends embedded in a Route keep the same true miter-ellipse
        # viewport weld guides that the former standalone object used.
        try:
            pie_loops = route_pie_cut_seam_loops(settings, settings.seam_guide_offset)
        except Exception:
            pie_loops = []
        for loop in pie_loops:
            inactive_vertices.extend(_loop_line_vertices(loop, mw))

    # Dedicated fabrication pie-cut weld seams.  These are true miter ellipses,
    # not circular helper rings, and are still viewport-only GPU geometry.
    for obj in scene.objects:
        if obj.type != 'MESH' or not obj.visible_get():
            continue
        settings = getattr(obj, "exhaust_pie_cut", None)
        if not settings or not settings.is_pie_cut or not settings.show_weld_seams:
            continue
        try:
            loops = pie_cut_seam_loops(settings, settings.seam_guide_offset)
        except Exception:
            continue
        for loop in loops:
            inactive_vertices.extend(_loop_line_vertices(loop, obj.matrix_world))

    try:
        gpu.state.blend_set('ALPHA')
        gpu.state.depth_test_set('LESS_EQUAL')
        _draw_lines(inactive_vertices, _DEFAULT_SEAM_COLOR, 1.5)
        _draw_lines(target_inactive_vertices, _DEFAULT_SEAM_COLOR, 1.5)
        _draw_lines(active_vertices, _ACTIVE_SEAM_COLOR, 3.0)
        _draw_lines(target_active_vertices, _ACTIVE_SEAM_COLOR, 3.0)
    finally:
        gpu.state.line_width_set(1.0)
        gpu.state.depth_test_set('NONE')
        gpu.state.blend_set('NONE')


def register_guides():
    global _DRAW_HANDLE
    if _DRAW_HANDLE is None:
        _DRAW_HANDLE = bpy.types.SpaceView3D.draw_handler_add(
            _draw_segment_seams, (), 'WINDOW', 'POST_VIEW'
        )


def unregister_guides():
    global _DRAW_HANDLE
    if _DRAW_HANDLE is not None:
        try:
            bpy.types.SpaceView3D.draw_handler_remove(_DRAW_HANDLE, 'WINDOW')
        except Exception:
            pass
        _DRAW_HANDLE = None
