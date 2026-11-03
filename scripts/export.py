"""Export the drone collection to web/drone.glb (meshopt-compressed, no UVs, extras kept)."""
import os, bpy

ROOT = r"D:\My files\G\proiecte\261006_Blender and CCode\survey_drone"
OUT = os.path.join(ROOT, "web", "drone.glb")
COMPRESS = globals().get("COMPRESS", "meshopt")


SKINS = ("Fuselage_Shell", "Access_Panel_L", "Access_Panel_R", "Top_Hatch", "Wing_L", "Wing_R", "Flap_L", "Flap_R",
         "Aileron_L", "Aileron_R", "H_Stabilizer_L", "H_Stabilizer_R", "Elevator_L", "Elevator_R", "Vertical_Fin",
         "Rudder", "Ventral_Fin")


def annotate(coll):
    """Per-part metadata for the viewer: triangle count, closed (section-cappable), skin flag."""
    import bmesh
    total = 0
    for o in coll.all_objects:
        if o.type != "MESH":
            continue
        bm = bmesh.new()
        bm.from_mesh(o.data)
        open_edges = sum(1 for e in bm.edges if not e.is_manifold)
        tris = sum(len(f.verts) - 2 for f in bm.faces)
        bm.free()
        o["tris"] = tris
        o["closed"] = open_edges == 0
        o["skin"] = o.name in SKINS
        o["node"] = o.name
        total += tris
    return total


def export(out=OUT, compress=COMPRESS):
    os.makedirs(os.path.dirname(out), exist_ok=True)
    coll = bpy.data.collections["SurveyDrone"]
    print("triangles", annotate(coll))
    hidden = [o for o in coll.all_objects if o.hide_get()]
    for o in hidden:
        o.hide_set(False)
    for o in bpy.context.view_layer.objects:
        o.select_set(o.name in coll.all_objects)
    kw = dict(filepath=out, export_format="GLB", use_selection=True, export_extras=True, export_yup=True,
              export_apply=True, export_texcoords=False, export_normals=True, export_tangents=False,
              export_materials="EXPORT", export_vertex_color="NONE")
    if compress == "meshopt":
        kw.update(export_meshopt_compression_enable=True)
    elif compress == "draco":
        kw.update(export_draco_mesh_compression_enable=True, export_draco_mesh_compression_level=7,
                  export_draco_position_quantization=14, export_draco_normal_quantization=10)
    try:
        bpy.ops.export_scene.gltf(**kw)
    except TypeError as e:
        kw.pop("export_vertex_color", None)
        bpy.ops.export_scene.gltf(**kw)
    for o in hidden:
        o.hide_set(True)
    print("exported", out, round(os.path.getsize(out) / 1024), "KB")


export()
