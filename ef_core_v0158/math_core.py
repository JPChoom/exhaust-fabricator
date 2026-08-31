import math


def packed_radial_radius(count: int, outside_diameter: float, gap: float = 0.0) -> float:
    """Center-to-axis radius for N equal circles arranged radially with adjacent gap."""
    count = max(2, int(count))
    pitch = max(0.0, outside_diameter + gap)
    s = math.sin(math.pi / count)
    return 0.0 if s == 0.0 else pitch / (2.0 * s)


def bend_arc_length(radius: float, angle_radians: float) -> float:
    return abs(radius * angle_radians)


def route_centerline_length(segments) -> float:
    total = 0.0
    for seg in segments:
        if seg.kind == 'STRAIGHT':
            total += max(0.0, seg.length)
        elif seg.kind == 'BEND':
            if getattr(seg, 'bend_style', 'MANDREL') == 'PIE_CUT':
                sections = max(2, int(getattr(seg, 'pie_sections', 6)))
                total_angle = abs(float(seg.angle))
                if total_angle > 0.0:
                    delta = total_angle / (sections - 1)
                    chord = 2.0 * max(0.0, seg.radius) * math.sin(delta * 0.5)
                    total += sections * chord
            else:
                total += bend_arc_length(max(0.0, seg.radius), seg.angle)
    return total
