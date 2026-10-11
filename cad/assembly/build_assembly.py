"""Blender script: animated assembly of mic_array -> front_end -> AC7200.

Run via make_assembly.sh (which exports/converts the GLB inputs first), or:
    blender -b --python build_assembly.py -- <glb_dir> <out.blend> [render.mp4]

World frame (mm, shown in Blender as metres*1e-3):
  origin = centre of the 100 mm square of HENC1-4 M4 holes, which sits at
  (268.863, 205.363) on mic_array.kicad_pcb and (100, 100) on front_end.kicad_pcb.
  The boards are exported with that point as the GLB origin, so the M4 holes
  coincide in X/Y by construction.  World Y = -KiCad Y.

Stack heights (PCB surface to PCB surface = connector mated height):
  mic_array      bottom z = 0.0,  top z = 1.6
  front_end      bottom z = 8.6   (AXK6S80347YG/AXK5S80347YG, 7.0 mm mated)
  AC7200 module  bottom z = 13.2  (AXK680337YG/AXK580137YG, 3.0 mm mated)
"""
import math
import sys

import bpy
from mathutils import Matrix, Vector

argv = sys.argv[sys.argv.index("--") + 1:]
GLB_DIR, OUT_BLEND = argv[0], argv[1]
OUT_VIDEO = argv[2] if len(argv) > 2 else None

MM = 0.001
PCB_T = 1.6
FE_Z = PCB_T + 7.0             # front_end bottom surface
AC_Z = FE_Z + PCB_T + 3.0      # AC7200 bottom (connector-side) surface

# KiCad positions relative to the HENC-square centre, converted to world (Y flipped)
def kicad_rel(x, y, ox, oy):
    return Vector(((x - ox) * MM, -(y - oy) * MM, 0))

U2_XY = kicad_rel(100.092, 90.095, 100, 100)        # AC7200 footprint origin
FE_J2_XY = kicad_rel(100.381, 86.124, 100, 100)     # socket, front_end B side
MA_J1_XY = kicad_rel(269.250, 191.495, 268.863, 205.363)  # header, mic_array F side
HENC_XY = [Vector((sx * 50 * MM, sy * 50 * MM, 0)) for sx in (-1, 1) for sy in (-1, 1)]
AC_HOLE_XY = [U2_XY + Vector((sx * 24.8 * MM, sy * 19.8 * MM, 0)) for sx in (-1, 1) for sy in (-1, 1)]

FPS = 24
F_END = 528
# key frames
F_FE_IN, F_FE_HOVER = 60, 110       # front_end flies in and hovers
F_AC_IN, F_AC_HOVER = 150, 200      # AC7200 flies in and hovers
F_GUIDES_ON = 225
F_FE_DOWN0, F_FE_DOWN1 = 240, 320
F_AC_DOWN0, F_AC_DOWN1 = 330, 400
F_GUIDES_OFF = 420


def import_group(name, path):
    """Import a GLB and parent all its top-level objects to a new empty."""
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=path)
    new = [o for o in bpy.data.objects if o not in before]
    # cascadio emits a 2 m bounding "Cube"; drop anything absurdly large
    for o in list(new):
        if o.type == 'MESH' and max(o.dimensions) > 1.0:
            new.remove(o)
            bpy.data.objects.remove(o, do_unlink=True)
    root = bpy.data.objects.new(name, None)
    bpy.context.scene.collection.objects.link(root)
    for o in new:
        if o.parent is None:
            o.parent = root
    return root


def key_loc(obj, frame, loc):
    obj.location = loc
    obj.keyframe_insert("location", frame=frame)


def key_hidden(obj, frame, hidden):
    obj.hide_render = hidden
    obj.hide_viewport = hidden
    obj.keyframe_insert("hide_render", frame=frame)
    obj.keyframe_insert("hide_viewport", frame=frame)


def emissive_mat(name, rgb, alpha=1.0, strength=2.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*rgb, 1)
    bsdf.inputs["Emission Color"].default_value = (*rgb, 1)
    bsdf.inputs["Emission Strength"].default_value = strength
    bsdf.inputs["Alpha"].default_value = alpha
    m.surface_render_method = 'BLENDED'
    return m


def plain_mat(name, rgb, rough=0.5, metal=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*rgb, 1)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    return m


# ---------------------------------------------------------------- scene setup
bpy.ops.wm.read_factory_settings(use_empty=True)
scn = bpy.context.scene
scn.unit_settings.length_unit = 'MILLIMETERS'
scn.render.fps = FPS
scn.frame_start, scn.frame_end = 1, F_END

mic = import_group("mic_array", f"{GLB_DIR}/mic_array.glb")
fe = import_group("front_end", f"{GLB_DIR}/front_end.glb")
ac = import_group("AC7200", f"{GLB_DIR}/ac7200.glb")
sock = import_group("J2_AXK5S80347YG", f"{GLB_DIR}/axk5s.glb")

# KiCad's GLB export uses the stackup colours: dielectric "PTFE natural" (near
# white) and an 83%-alpha mask, which renders washed out.  Make the mask an
# opaque green and the board body FR4-coloured.
for m in bpy.data.materials:
    n = m.node_tree.nodes.get("Principled BSDF") if m.node_tree else None
    if not n:
        continue
    c = tuple(round(v, 2) for v in n.inputs["Base Color"].default_value[:3])
    if c == (0.08, 0.2, 0.14):
        n.inputs["Base Color"].default_value = (0.01, 0.08, 0.035, 1)
        n.inputs["Alpha"].default_value = 1.0
        n.inputs["Roughness"].default_value = 0.65
        n.inputs["Specular IOR Level"].default_value = 0.3
    elif c == (0.99, 0.99, 0.98):
        n.inputs["Base Color"].default_value = (0.36, 0.38, 0.22, 1)

# The AC7200 STEP's board body is light grey; the real module is black.
for o in ac.children_recursive:
    if o.name.startswith("BOARD") and o.type == 'MESH':
        o.data.materials.clear()  # imported without a material slot
        o.data.materials.append(plain_mat("ac7200_mask", (0.012, 0.012, 0.014), 0.5))

# AC7200 STEP (as imported): board in local XZ, 0..55 x 0..45, thickness y 0..1.6,
# B2B connectors on +y, FPGA on -y, vendor top view = local X right / Z up.
# Mount connectors-down: local X->X, Y->-Z, Z->Y (Rx -90 deg), then put the
# module centre (27.5, 22.5) on the U2 footprint origin and its connector face on AC_Z.
ac_inner = bpy.data.objects.new("AC7200_model", None)
scn.collection.objects.link(ac_inner)
for o in list(ac.children):
    o.parent = ac_inner
ac_inner.parent = ac
ac_inner.matrix_parent_inverse = Matrix.Identity(4)
ac_inner.rotation_euler = (math.radians(-90), 0, 0)
ac_inner.location = Vector((-27.5 * MM, -22.5 * MM, PCB_T * MM))  # after rotation: y=1.6 -> z=-1.6
AC_FINAL = Vector((U2_XY.x, U2_XY.y, AC_Z * MM))

# Front_end J2 socket (no 3D model on the footprint): STEP model has its
# solder plane at local y=0 with the body toward -y, long axis along X.
# Hang it from the front_end's bottom face, body pointing down.
sock.parent = fe
sock.rotation_euler = (math.radians(90), 0, 0)
sock.location = FE_J2_XY

# Mic_array J1 header: no vendor model available -> simple proxy block
bpy.ops.mesh.primitive_cube_add(size=1)
hdr = bpy.context.object
hdr.name = "J1_AXK6S80347YG_proxy"
hdr.scale = (21.6 * MM, 3.6 * MM, 1.6 * MM)
hdr.location = MA_J1_XY + Vector((0, 0, (PCB_T + 0.8) * MM))
hdr.data.materials.append(plain_mat("connector_black", (0.02, 0.02, 0.02), 0.6))
hdr.parent = mic

# ----------------------------------------------------------- alignment guides
guide_mat = emissive_mat("guide", (1.0, 0.45, 0.05), alpha=0.6, strength=3.0)
guides = []
for xy, r, z0, z1 in ([(p, 1.6, -4, 150) for p in HENC_XY] +
                      [(p, 1.1, FE_Z, 230) for p in AC_HOLE_XY]):
    bpy.ops.mesh.primitive_cylinder_add(radius=r * MM, depth=(z1 - z0) * MM,
                                        location=(xy.x, xy.y, (z0 + z1) / 2 * MM), vertices=24)
    g = bpy.context.object
    g.name = "guide"
    g.data.materials.append(guide_mat)
    guides.append(g)
for g in guides:
    key_hidden(g, 1, True)
    key_hidden(g, F_GUIDES_ON, False)
    key_hidden(g, F_GUIDES_OFF, True)

# ------------------------------------------------------------------ animation
FE_FINAL = Vector((0, 0, FE_Z * MM))
FE_HOVER = Vector((0, 0, (FE_Z + 70) * MM))
FE_AWAY = Vector((0, 0, (FE_Z + 700) * MM))
AC_HOVER = AC_FINAL + Vector((0, 0, 150 * MM))
AC_AWAY = AC_FINAL + Vector((0, 0, 900 * MM))

mic.location = (0, 0, 0)
for f, p in ((1, FE_AWAY), (F_FE_IN, FE_AWAY), (F_FE_HOVER, FE_HOVER),
             (F_FE_DOWN0, FE_HOVER), (F_FE_DOWN1, FE_FINAL)):
    key_loc(fe, f, p)
for f, p in ((1, AC_AWAY), (F_AC_IN, AC_AWAY), (F_AC_HOVER, AC_HOVER),
             (F_FE_DOWN0, AC_HOVER), (F_FE_DOWN1, AC_HOVER + Vector((0, 0, (FE_FINAL.z - FE_HOVER.z)))),
             (F_AC_DOWN0, AC_HOVER + Vector((0, 0, (FE_FINAL.z - FE_HOVER.z)))), (F_AC_DOWN1, AC_FINAL)):
    key_loc(ac, f, p)

# --------------------------------------------------------------- camera rig
rig = bpy.data.objects.new("camera_rig", None)
scn.collection.objects.link(rig)
cam_data = bpy.data.cameras.new("Camera")
cam_data.lens = 50
cam_data.clip_start = 0.005
cam = bpy.data.objects.new("Camera", cam_data)
scn.collection.objects.link(cam)
cam.parent = rig
scn.camera = cam
target = bpy.data.objects.new("cam_target", None)
scn.collection.objects.link(target)
trk = cam.constraints.new('TRACK_TO')
trk.target = target
trk.track_axis = 'TRACK_NEGATIVE_Z'
trk.up_axis = 'UP_Y'

def key_cam(frame, dist, elev_deg, tz):
    e = math.radians(elev_deg)
    cam.location = (0, -dist * math.cos(e), dist * math.sin(e))
    cam.keyframe_insert("location", frame=frame)
    target.location = (0, 0, tz)
    target.keyframe_insert("location", frame=frame)

key_cam(1, 0.62, 38, 0.0)
key_cam(F_FE_IN, 0.62, 38, 0.0)
key_cam(F_AC_HOVER, 0.70, 28, 0.08)
key_cam(F_FE_DOWN0, 0.70, 28, 0.08)
key_cam(F_AC_DOWN1, 0.42, 30, 0.012)
key_cam(F_GUIDES_OFF + 20, 0.36, 40, 0.008)
key_cam(F_END, 0.60, 60, 0.0)

prefs = bpy.context.preferences.edit
prefs.keyframe_new_interpolation_type = 'LINEAR'
rig.rotation_euler = (0, 0, math.radians(-30))
rig.keyframe_insert("rotation_euler", frame=1)
rig.rotation_euler = (0, 0, math.radians(-30 + 300))
rig.keyframe_insert("rotation_euler", frame=F_END)
prefs.keyframe_new_interpolation_type = 'BEZIER'

# ------------------------------------------------------------ captions (HUD)
def caption(text, f_on, f_off):
    cu = bpy.data.curves.new("cap", 'FONT')
    cu.body = text
    cu.size = 0.0011
    cu.align_x = 'CENTER'
    ob = bpy.data.objects.new("caption", cu)
    scn.collection.objects.link(ob)
    ob.parent = cam
    ob.location = (0, -0.0088, -0.05)
    ob.data.materials.append(emissive_mat("caption_white", (1, 1, 1), strength=1.0))
    bpy.ops.mesh.primitive_plane_add(size=1)
    plate = bpy.context.object
    plate.name = "caption_plate"
    plate.scale = (0.036, 0.0024, 1)
    plate.parent = cam
    plate.location = (0, -0.0084, -0.0501)
    plate.data.materials.append(plate_mat)
    for o in (ob, plate):
        key_hidden(o, 1, True)
        key_hidden(o, f_on, False)
        key_hidden(o, f_off, True)

plate_mat = emissive_mat("caption_plate", (0, 0, 0), alpha=0.7, strength=0.0)

caption("mic_array  -  96-mic Underbrink spiral, 320 mm", 8, F_FE_IN)
caption("front_end  -  GbE PHY, buck, AC7200 carrier", F_FE_IN + 10, F_AC_IN)
caption("ALINX AC7200  -  XC7A200T FPGA module", F_AC_IN + 10, F_GUIDES_ON)
caption("4x M4 holes aligned  -  7.0 mm AXK6S/5S80347YG stack", F_GUIDES_ON + 5, F_FE_DOWN1 + 5)
caption("4x module holes aligned  -  3.0 mm AXK680337YG stack", F_FE_DOWN1 + 10, F_GUIDES_OFF)
caption("Assembled stack", F_GUIDES_OFF + 10, F_END + 1)

# ------------------------------------------------------------------ lighting
world = bpy.data.worlds.new("World")
scn.world = world
world.use_nodes = True
bg = world.node_tree.nodes["Background"]
bg.inputs["Color"].default_value = (0.035, 0.04, 0.05, 1)
bg.inputs["Strength"].default_value = 1.0

def area(name, loc, energy, size):
    ld = bpy.data.lights.new(name, 'AREA')
    ld.energy = energy
    ld.size = size
    lo = bpy.data.objects.new(name, ld)
    scn.collection.objects.link(lo)
    lo.location = loc
    c = lo.constraints.new('TRACK_TO')
    c.target = target
    c.track_axis = 'TRACK_NEGATIVE_Z'
    c.up_axis = 'UP_Y'

area("key", (0.5, -0.5, 0.8), 25, 0.6)
area("fill", (-0.7, -0.2, 0.5), 9, 0.8)
area("rim", (0.0, 0.8, 0.4), 12, 0.5)

# ------------------------------------------------------------------ render
r = scn.render
# Cycles on CPU: this machine only has software OpenGL, where EEVEE is far slower.
r.engine = 'CYCLES'
scn.cycles.device = 'CPU'
scn.cycles.samples = 24
scn.cycles.use_denoising = True
scn.cycles.max_bounces = 4
r.use_persistent_data = True
r.resolution_x, r.resolution_y = 1280, 720
r.resolution_percentage = 100
scn.view_settings.view_transform = 'AgX'
if OUT_VIDEO:
    if hasattr(r.image_settings, 'media_type'):
        r.image_settings.media_type = 'VIDEO'
    r.image_settings.file_format = 'FFMPEG'
    r.ffmpeg.format = 'MPEG4'
    r.ffmpeg.codec = 'H264'
    r.ffmpeg.constant_rate_factor = 'HIGH'
    r.filepath = OUT_VIDEO

bpy.ops.wm.save_as_mainfile(filepath=OUT_BLEND)
print("saved", OUT_BLEND)
