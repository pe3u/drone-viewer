"""Entry point: run inside Blender.  exec(open(path).read())"""
import sys, importlib, bpy

SCRIPTS = r"D:\My files\G\proiecte\261006_Blender and CCode\survey_drone\scripts"
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import dl
importlib.reload(dl)
dl.DETAIL = globals().get("DETAIL", 1.0)

MODULES = ["fuselage"]
for extra in ("wings", "tail", "fillets", "propulsion", "payload", "interior", "avionics", "wiring", "infill"):
    try:
        importlib.import_module(extra)
        MODULES.append(extra)
    except ModuleNotFoundError:
        pass

ONLY = globals().get("ONLY")  # optional list of module names to rebuild


def run():
    mods = {}
    for name in MODULES:
        m = importlib.import_module(name)
        importlib.reload(m)
        mods[name] = m
    if ONLY:
        root = bpy.data.objects.get("SurveyDrone")
        for name in ONLY:
            for o in list(dl.collection().all_objects):
                if o.get("module") == name:
                    bpy.data.objects.remove(o, do_unlink=True)
    else:
        dl.clear_all()
        root = dl.empty("SurveyDrone", size=0.3)
    for name, m in mods.items():
        if ONLY and name not in ONLY:
            continue
        before = set(dl.collection().all_objects)
        m.build(root)
        for o in set(dl.collection().all_objects) - before:
            o["module"] = name
    for me in list(bpy.data.meshes):
        if me.users == 0:
            bpy.data.meshes.remove(me)
    tris = sum(sum(len(p.vertices) - 2 for p in o.data.polygons)
               for o in dl.collection().all_objects if o.type == "MESH")
    print("objects", len(dl.collection().all_objects), "tris", tris)


run()
