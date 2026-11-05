# Drone Viewer

Procedural Blender model of a fixed-wing LiDAR / photogrammetry survey drone, including its internal construction, plus a web viewer.

**Live viewer:** https://pe3u.github.io/drone-viewer/ (the `web/` folder, deployed by `.github/workflows/pages.yml` on every push to `main` that touches it).

## Model (Blender 5.2, metres, nose toward -X)

Built entirely by the Python scripts in `scripts/`. Edit them, not the `.blend`.

| Script | Builds |
|---|---|
| `dl.py` | Library: flat-colour palette, part builder (boxes, cylinders, lathes, lofts, tubes), fuselage profile, NACA airfoils, `DETAIL` tessellation factor |
| `fuselage.py` | Shell, removable side panels and top hatch (with fasteners), bulkheads, floor, aft tray, longerons, wing joiner tubes, belly skids |
| `wings.py` | Wing skins with leading edges blended into the fuselage, winglets, flaps, ailerons, ribs, spars, servos, pushrods, nav lights, pitot probe |
| `tail.py` | Horizontal stabilisers, elevators, vertical fin, rudder, ventral fin |
| `fillets.py` | Aft root fairings on the wing and tailplane (the leading-edge blends on the wing, tailplane, fin and ventral fin are part of those surfaces, see `dl.blend_ext`) |
| `propulsion.py` | Motor mount, outrunner motor, spinner, 2-blade pusher prop |
| `payload.py` | LiDAR, gimballed mapping camera, GNSS antenna, datalink antenna |
| `avionics.py` | IMU, flight controller, data processing unit, battery, PDB, ESC, tail servos, radio, GNSS receiver, auxiliary electronics |
| `interior.py` | Packed interior: hull-conforming modules per bay (payload computer, storage, satcom, fan, BMS, parachute, payload and belly batteries, Remote ID / ADS-B, tail equipment). `conform()` builds a block = rect ∩ hull − keep-outs |
| `wiring.py` | Harness: power, motor phases, data, servo looms, RF coax, cable ties, routed along the floor, the G1 tray and the risers |
| `fill_check.py` | Voxel measure of the interior fill (and `overlaps()` for part collisions) |

Interior fill: **61.4 % (HD) / 61.3 % (Lite)** of the 53 L fuselage interior, measured by `fill_check.py` and stored on the root node (`fill` extra), shown in the viewer stats.

Every part is a closed mesh (so section cuts can be capped) and carries metadata (label, group, explode vector, description) exported as glTF extras.

## Pipeline

1. In Blender (MCP or the Text editor): run `scripts/rebuild_export.py`. It rebuilds the model and writes `web/drone_lite.glb` (DETAIL 0.5, ~46k tris) and `web/drone.glb` (DETAIL 1.0, ~112k tris), Draco-compressed, then saves `survey_drone.blend`.
2. Local viewer: `node web/serve.js`, then open http://127.0.0.1:8370/.
3. Single-file page for the claude.ai Artifact host: run `rebuild_export.py` with `OUTDIR=artifact/raw` and `COMP="none"`, then `node scripts/build_artifact.js` → `artifact/survey-drone.html`.

## Viewer

three.js 0.170: orbit / auto-orbit, autoplay guided tour (17 steps), staggered exploded view, Solid / Cutaway / X-ray / Off skins, section planes (side, station, plan) with stencil caps and sweep, labels fixed on a ring around the drone with faint leader lines (orbiting moves only the lines), part list with hide / isolate / focus, control-surface check, motor run, wireframe, HD / Lite LOD switch, light and dark themes. The ground lies 100 m below the drone and fades linearly (by viewing angle) to its 6 km rim; the dark theme adds stars and an aurora borealis in the upper sky. Auto-orbit takes ten minutes per turn. Menus rest as Parts / View / Stats buttons and roll out on hover (on touch screens, tap the button). Keys: Space play/pause, E explode, Esc deselect. `?lod=lite` forces the light model.
