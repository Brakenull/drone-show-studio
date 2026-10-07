"""Several holding areas, run inside Blender (pytest does not collect it):

    blender -b --factory-startup --python tests/blender_check_holding_areas.py

A 300-drone fleet over a 4 m cube: the scene's one holding area becomes the
first list item when a second is added; each area gets its own scene objects;
moving a box moves its area; the export carries `holding_areas` whose slot
counts fill the areas in list order, with the first keyframe's spare drones on
the first slots; areas too close lock Export.
"""
import json, os, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import bpy
import numpy as np
import stage1_designer as dss
from stage1_designer.core import holding_area as core
from stage1_designer.ui import holding_area_scene, panel
dss.register()

scene = bpy.context.scene
vl = bpy.context.view_layer
s = scene.drone_show_settings
for o in list(bpy.data.objects):
    bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_cube_add(size=4, location=(0, 20, 20))
cube = bpy.context.active_object
cube.keyframe_insert("location", frame=1)
cube.location.x = 10
cube.keyframe_insert("location", frame=240)
s.target_object = cube
s.fleet_size = 300
s.export_path = os.path.join(tempfile.gettempdir(), "dss_holding_areas_test.json")

# One area, as before the list: the scene's own holding_area.
s.holding_area.center = (0, -30, 0)
s.holding_area.size = (20, 10)  # 11 x 6 / 10 x 5 per layer, 4 layers: 232 drones
assert len(s.holding_areas) == 0 and holding_area_scene.slot_counts(s) == [300]
assert holding_area_scene.layouts_for(s)[0].widened

class FakeLayout:
    """Swallows every UI call, so the panel's draw() runs headless."""
    alert = enabled = False
    def __getattr__(self, name):
        return lambda *a, **k: FakeLayout()

def draw():
    panel.DSS_PT_MainPanel.draw(type("P", (), {"layout": FakeLayout()})(), bpy.context)

draw()

# Adding a second area: the first item is the old one, the new one next to it.
assert bpy.ops.dss.holding_area_add() == {"FINISHED"}
assert len(s.holding_areas) == 2 and tuple(s.holding_areas[0].center) == (0, -30, 0)
assert holding_area_scene.slot_counts(s) == [232, 68]
assert not any(l.widened for l in holding_area_scene.layouts_for(s))
assert panel.holding_apart_messages(s) == [], panel.holding_apart_messages(s)
draw()

# Scene objects per area; moving area 2's box moves its center.
s.show_holding_area_object = True
for n in (1, 2):
    for part in ("Volume", "Slots", "Clearance"):
        assert bpy.data.objects.get(f"DSS_HoldingArea_{n}_{part}") is not None, (n, part)
box = bpy.data.objects["DSS_HoldingArea_2_Volume"]
box.location.x += 5.0
vl.update()
assert abs(s.holding_areas[1].center[0] - (box.location.x)) < 1e-4

# Too close: Export locks.
saved = tuple(s.holding_areas[1].center)
s.holding_areas[1].center = (22, -30, 0)
assert panel.holding_apart_messages(s), "areas 2 m apart should lock"
s.holding_areas[1].center = saved

# Export: two areas with their slot counts, first-keyframe padding on the first slots.
bpy.ops.dss.check_kinematics()
assert bpy.ops.dss.export_intermediate() == {"FINISHED"}
data = json.load(open(s.export_path))
meta = data["project_metadata"]
assert meta["version"] == "1.8.0" and "holding_area" not in meta
assert [a["slot_count"] for a in meta["holding_areas"]] == [232, 68]
areas, counts = core.areas_from_metadata(meta)
slots = core.compute_all_holding_positions(areas, counts)
first = np.array([p["pos"] for p in data["keyframes"][0]["points"]])
spare = first[core.in_holding_region(first, areas, counts)]
assert len(spare) and np.allclose(spare, slots[: len(spare)]), len(spare)

# Remove: back to one list item, its objects rebuilt.
s.holding_area_index = 1
assert bpy.ops.dss.holding_area_remove() == {"FINISHED"}
assert len(s.holding_areas) == 1 and bpy.data.objects.get("DSS_HoldingArea_2_Volume") is None
assert bpy.data.objects.get("DSS_HoldingArea_1_Volume") is not None
draw()
print("blender_check_holding_areas: OK")
