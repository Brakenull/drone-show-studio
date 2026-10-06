"""Check Kinematics lock, run inside Blender (pytest does not collect it):

    blender -b --factory-startup --python tests/blender_check_kinematics_lock.py

A 4 m cube moving 10 m in 10 s with a 300-drone fleet: every seed gives the
true 1 m/s; a pass disables Check Kinematics until the show changes; Export
writes the passed keyframes without sampling again.
"""
import json, os, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import bpy
import stage1_designer as dss
from stage1_designer.ui import panel
from bpy_extras import anim_utils
dss.register()

scene = bpy.context.scene
vl = bpy.context.view_layer
s = scene.drone_show_settings
for o in list(bpy.data.objects):
    bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_cube_add(size=4, location=(0, 0, 20))
cube = bpy.context.active_object
cube.keyframe_insert("location", frame=1)
cube.location.x = 10
cube.keyframe_insert("location", frame=240)
s.target_object = cube
s.fleet_size = 300
s.holding_area.center = (0, -30, 0)
s.export_path = os.path.join(tempfile.gettempdir(), "dss_lock_test.json")
vl.update()

calls = [0]
orig = dss._sample_all_keyframes_for_validation
def counted(*a, **k):
    calls[0] += 1
    return orig(*a, **k)
dss._sample_all_keyframes_for_validation = counted

def locked():
    vl.update()
    return panel.passed_check(scene) is not None

def check():
    bpy.ops.dss.check_kinematics()
    return panel.get_kinematic_cache()

# 1. Same seed -> same result; pass rate over seeds (was random before the seed).
results = []
for seed in range(20):
    s.sample_seed = seed
    t = check()
    results.append(t[0].v_req)
assert all(abs(v - results[0]) < 1e-6 for v in results) and results[0] < 6.0, results

s.sample_seed = 7
check()
assert locked(), "pass should lock"
try:
    bpy.ops.dss.check_kinematics()
    raise AssertionError("poll should block a second check")
except RuntimeError:
    pass

# 2. Export reuses the passed keyframes (no new sampling pass).
entries, _ = panel.passed_check(scene)
n = calls[0]
assert bpy.ops.dss.export_intermediate() == {"FINISHED"}
assert calls[0] == n, "export sampled again"
data = json.load(open(s.export_path))
assert [p["pos"] for p in data["keyframes"][1]["points"]] == [list(p["pos"]) for p in entries[1]["points"]]

# 3. What keeps the lock and what clears it.
def keeps(name, fn):
    fn(); assert locked(), f"{name} should keep the lock"
def clears(name, fn):
    global n
    s.sample_seed = 7
    if not locked():
        check()
    assert locked(), f"relock before {name}"
    fn(); assert not locked(), f"{name} should clear the lock"

keeps("select", lambda: cube.select_set(False))
keeps("frame_set", lambda: scene.frame_set(100))
keeps("export path", lambda: setattr(s, "export_path", s.export_path + ".x"))
keeps("overlay toggle", lambda: setattr(s, "live_update_on_frame_change", True))
clears("move object", lambda: setattr(cube.location, "z", 21))
def edit_mesh():
    cube.data.vertices[0].co.z += 0.5; cube.data.update()
clears("edit mesh", edit_mesh)
def move_key():
    ad = cube.animation_data
    fc = anim_utils.action_get_channelbag_for_slot(ad.action, ad.action_slot).fcurves[0]
    fc.keyframe_points[1].co.y += 1; fc.update()
clears("move key", move_key)
clears("min distance", lambda: setattr(s, "min_distance_m", s.min_distance_m + 0.1))
clears("holding center", lambda: setattr(s.holding_area, "center", (0, -31, 0)))
clears("marker", lambda: scene.timeline_markers.new("A", frame=1))
clears("fps", lambda: setattr(scene.render, "fps", 30))
clears("preview", lambda: bpy.ops.dss.preview_sample())
clears("v_max override", lambda: setattr(s.kinematic_constraints, "enabled", True))

# 4. Auto-Fix: a failing short timeline is stretched, then locked.
s.kinematic_constraints.enabled = True
s.kinematic_constraints.v_max_mps = 0.5
s.sample_seed = 7
try:
    check()
except RuntimeError:
    pass
assert not locked()
bpy.ops.dss.auto_fix_timeline()
assert locked(), "auto-fix pass should lock"

# 5. The panel still draws (fake layout).
class Fake:
    def __getattr__(self, name):
        return lambda *a, **k: Fake()
    def __setattr__(self, name, value):
        pass
class P: pass
p = P(); p.layout = Fake()
class Ctx: scene = scene; screen = None
panel.DSS_PT_MainPanel.draw(p, Ctx())

# 6. A growing shape changes its point count: the drones that join fly
# from their slots, not a slot-to-shape pairing by index.
s.kinematic_constraints.enabled = False
for o in list(bpy.data.objects):
    bpy.data.objects.remove(o)
bpy.ops.mesh.primitive_cube_add(size=4, location=(0, 0, 20))
grow = bpy.context.active_object
grow.keyframe_insert("location", frame=1)
grow.keyframe_insert("scale", frame=1)
grow.location.x = 10
grow.scale = (1.5, 1.5, 1.5)
grow.keyframe_insert("location", frame=240)
grow.keyframe_insert("scale", frame=240)
s.target_object = grow
counts, v = set(), []
for seed in range(20):
    s.sample_seed = 100 + seed
    t = check()
    counts.add(tuple(len(f[1]) for f in panel._formation_cache["formations"]))
    v.append(t[0].v_req)
print("growing cube counts:", sorted(counts), "v_req:", round(min(v), 2), "-", round(max(v), 2))
assert any(a != b for a, b in counts), "the test needs a count change"
assert max(v) < 6.0, v

print("ALL OK")
