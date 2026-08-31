"""Header-owned keep-out object references for Exhaust Fabricator v0.14.0.

Keep-outs are stored as lightweight object-name metadata on the Header rather
than as an RNA CollectionProperty.  This keeps the proven Header registration
path simple and makes missing/deleted obstacle objects harmless.
"""
import json
import bpy

_KEY = "exhaust_header_keepout_names"


def keepout_names(header):
    if header is None:
        return []
    raw = header.get(_KEY, "[]")
    try:
        data = json.loads(str(raw))
    except Exception:
        data = []
    out = []
    seen = set()
    for item in data if isinstance(data, list) else []:
        name = str(item)
        if name and name not in seen:
            seen.add(name)
            out.append(name)
    return out


def set_keepout_names(header, names):
    if header is None:
        return
    clean = []
    seen = set()
    for item in names or []:
        name = str(item)
        if name and name not in seen:
            seen.add(name)
            clean.append(name)
    header[_KEY] = json.dumps(clean, separators=(",", ":"))


def keepout_objects(header):
    return [obj for name in keepout_names(header) if (obj := bpy.data.objects.get(name)) is not None]


def add_keepouts(header, objects):
    names = keepout_names(header)
    existing = set(names)
    added = 0
    for obj in objects or []:
        if obj is None or getattr(obj, 'type', None) != 'MESH':
            continue
        if obj.name in existing:
            continue
        names.append(obj.name)
        existing.add(obj.name)
        added += 1
    set_keepout_names(header, names)
    return added


def remove_keepout(header, object_name):
    before = keepout_names(header)
    after = [n for n in before if n != str(object_name)]
    set_keepout_names(header, after)
    return len(after) != len(before)


def clear_keepouts(header):
    set_keepout_names(header, [])
