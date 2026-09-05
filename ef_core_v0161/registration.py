import bpy
from .properties import register_properties, unregister_properties
from .operators import CLASSES as OPERATOR_CLASSES
from .ui import CLASSES as UI_CLASSES
from .guides import register_guides, unregister_guides


def register():
    registered_ops = []
    registered_ui = []
    props_done = False
    guides_done = False
    try:
        register_properties()
        props_done = True
        for cls in OPERATOR_CLASSES:
            bpy.utils.register_class(cls)
            registered_ops.append(cls)
        for cls in UI_CLASSES:
            bpy.utils.register_class(cls)
            registered_ui.append(cls)
        register_guides()
        guides_done = True
    except Exception:
        if guides_done:
            try: unregister_guides()
            except Exception: pass
        for cls in reversed(registered_ui):
            try: bpy.utils.unregister_class(cls)
            except Exception: pass
        for cls in reversed(registered_ops):
            try: bpy.utils.unregister_class(cls)
            except Exception: pass
        if props_done:
            try: unregister_properties()
            except Exception: pass
        raise


def unregister():
    try:
        unregister_guides()
    except Exception:
        pass
    for cls in reversed(UI_CLASSES):
        try: bpy.utils.unregister_class(cls)
        except Exception: pass
    for cls in reversed(OPERATOR_CLASSES):
        try: bpy.utils.unregister_class(cls)
        except Exception: pass
    try:
        unregister_properties()
    except Exception:
        pass
