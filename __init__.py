bl_info = {
    "name": "Exhaust Fabricator",
    "author": "JP / OpenAI",
    "version": (0, 14, 21),
    "blender": (4, 2, 0),
    "location": "View3D > Sidebar > Exhaust",
    "description": "Parametric automotive exhaust fabrication and equal-length header design",
    "category": "Object",
}

# Blender can retain submodules from a previously uninstalled extension in the
# current Python session.  Purge prior Exhaust Fabricator internal packages
# before loading this release so registration always uses the files in this ZIP.
import sys

for _name in list(sys.modules):
    if _name == __name__:
        continue
    # Purge any previous internal Exhaust Fabricator core namespace retained by
    # Blender in the current application session.
    if _name.startswith(__name__ + ".ef_core_") or _name.startswith(__name__ + ".exhaust_fabricator"):
        sys.modules.pop(_name, None)

from .ef_core_v0161 import register, unregister

__all__ = ("register", "unregister")
