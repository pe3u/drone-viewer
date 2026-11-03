"""Rebuild the drone in the open Blender session and export both LODs (quiet).

web/drone_lite.glb  DETAIL 0.5  (phones / first paint)
web/drone.glb       DETAIL 1.0  (full detail, left in the Blender scene and saved to survey_drone.blend)
"""
import io, os, contextlib, bpy

D = r"D:\My files\G\proiecte\261006_Blender and CCode\survey_drone\scripts"
WEB = r"D:\My files\G\proiecte\261006_Blender and CCode\survey_drone\web"
LODS = globals().get("LODS", [("drone_lite.glb", 0.5), ("drone.glb", 1.0)])
WEB = globals().get("OUTDIR", WEB)
COMP = globals().get("COMP", "draco")

for fname, detail in LODS:
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        exec(compile(open(D + r"\build_all.py").read(), "build_all.py", "exec"), {"__name__": "__main__", "DETAIL": detail})
        fc = {"__name__": "fc", "STEP": 0.005}
        exec(compile(open(D + r"\fill_check.py").read(), "fill_check.py", "exec"), fc)
        bpy.data.objects["SurveyDrone"]["fill"] = round(fc["FRACTION"], 3)
        g = {"__name__": "export_mod", "COMPRESS": COMP}
        exec(compile(open(D + r"\export.py").read().replace("\nexport()\n", "\n"), "export.py", "exec"), g)
        g["export"](out=os.path.join(WEB, fname), compress=COMP)
    for line in buf.getvalue().splitlines():
        if line.startswith(("objects", "interior", "exported", "Traceback", "Error")):
            print(f"[{detail}]", line)

for n in ("Access_Panel_R",):
    o = bpy.data.objects.get(n)
    if o:
        o.hide_set(True)
for o in bpy.context.view_layer.objects:
    o.select_set(False)
blend = r"D:\My files\G\proiecte\261006_Blender and CCode\survey_drone\survey_drone.blend"
bpy.ops.wm.save_as_mainfile(filepath=blend, copy=False)
print("saved", blend)
