#!/usr/bin/env python3
"""Generate meshes/torso_frame.stl: the Magni torso structural frame.

One binary STL modelling a frame built from 30 mm T-slot aluminum extrusion
(2020/3030-style), containing BOTH the torso column and the serving-tray
support, as one rigid weldment:

  - 4 vertical column rails with two levels of cross members
  - shoulder crossbar with an adapter plate on each end
    (right plate carries the arm; left is free for a second arm)
  - neck post + head top plate
  - tray support: riser off the left shoulder, Y-arm, X-arm under the tray

Each rail is an extruded profile approximated by 6 overlapping boxes
(4 corner strips + 2 center webs), which leaves the characteristic T-slot
groove down the middle of each face.

Local frame: origin at the column base center (bolted to the Magni deck),
+X robot-forward, +Y robot-left, +Z up. All dimensions in meters.

The URDF mount frames in torso.urdf.xacro depend on the dimensions below --
keep them in sync if you change anything.

Run from anywhere:  python3 generate_torso_frame_mesh.py
"""
import os
import struct

P = 0.030   # profile size (30 mm)
G = 0.008   # slot groove width
D = 0.008   # slot groove depth

COL_X = 0.045   # column rail centers at (+-COL_X, +-COL_Y)
COL_Y = 0.12
COL_TOP = 0.495          # verticals run 0 .. COL_TOP
CROSSBAR_Z = 0.51        # shoulder crossbar center (rests on the verticals)
CROSSBAR_HALF = 0.17     # crossbar spans +-CROSSBAR_HALF in y
PLATE = (0.10, 0.012, 0.10)   # shoulder adapter plate (x, y-thickness, z)
NECK_TOP = 0.64
TRAY_RISER_Y = 0.155
TRAY_ARM_Z = 0.5775      # tray support arms center height
TRAY_X = 0.15            # tray disc center (TRAY_X, TRAY_Y)
TRAY_Y = 0.30

triangles = []  # (normal, v0, v1, v2)


def add_box(cx, cy, cz, sx, sy, sz):
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2
    z0, z1 = cz - sz / 2, cz + sz / 2
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [
        ((0, 0, -1), (0, 2, 1), (0, 3, 2)),   # bottom
        ((0, 0, 1), (4, 5, 6), (4, 6, 7)),    # top
        ((0, -1, 0), (0, 1, 5), (0, 5, 4)),   # front (-y)
        ((0, 1, 0), (2, 3, 7), (2, 7, 6)),    # back (+y)
        ((-1, 0, 0), (0, 4, 7), (0, 7, 3)),   # left (-x)
        ((1, 0, 0), (1, 2, 6), (1, 6, 5)),    # right (+x)
    ]
    for n, t1, t2 in faces:
        triangles.append((n, v[t1[0]], v[t1[1]], v[t1[2]]))
        triangles.append((n, v[t2[0]], v[t2[1]], v[t2[2]]))


def add_rail(axis, u, v, a0, a1):
    """Extruded T-slot profile along `axis`; (u, v) = cross-section center.

    axis 'x': (u, v) = (y, z);  'y': (x, z);  'z': (x, y).
    """
    length = a1 - a0
    mid = (a0 + a1) / 2
    corner = (P - G) / 2          # corner strip size
    off = (P - corner) / 2        # corner strip center offset
    web = P - 2 * D               # center web thickness
    # (du, dv, su, sv) in cross-section coordinates
    parts = [
        (0, 0, P, web),           # u-web (full width, leaves groove depth D)
        (0, 0, web, P),           # v-web
        (off, off, corner, corner),
        (-off, off, corner, corner),
        (off, -off, corner, corner),
        (-off, -off, corner, corner),
    ]
    for du, dv, su, sv in parts:
        if axis == 'x':
            add_box(mid, u + du, v + dv, length, su, sv)
        elif axis == 'y':
            add_box(u + du, mid, v + dv, su, length, sv)
        else:
            add_box(u + du, v + dv, mid, su, sv, length)


# --- column: 4 verticals + 2 levels of Y cross members ---
for sx in (COL_X, -COL_X):
    for sy in (COL_Y, -COL_Y):
        add_rail('z', sx, sy, 0.0, COL_TOP)
for level_z in (0.065, 0.315):
    for sx in (COL_X, -COL_X):
        add_rail('y', sx, level_z, -(COL_Y - P / 2), COL_Y - P / 2)

# --- shoulder crossbar + adapter plates ---
add_rail('y', 0.0, CROSSBAR_Z, -CROSSBAR_HALF, CROSSBAR_HALF)
for side in (1, -1):
    add_box(0.0, side * (CROSSBAR_HALF + PLATE[1] / 2), CROSSBAR_Z,
            PLATE[0], PLATE[1], PLATE[2])

# --- neck post + head top plate ---
add_rail('z', 0.0, 0.0, CROSSBAR_Z + P / 2, NECK_TOP)
add_box(0.0, 0.0, NECK_TOP + 0.004, 0.08, 0.08, 0.008)

# --- tray support: riser on the left shoulder, then Y-arm, then X-arm ---
add_rail('z', 0.0, TRAY_RISER_Y, CROSSBAR_Z + P / 2, TRAY_ARM_Z + P / 2)
add_rail('y', 0.0, TRAY_ARM_Z, TRAY_RISER_Y - P / 2, TRAY_Y + P / 2)
add_rail('x', TRAY_Y, TRAY_ARM_Z, -P / 2, TRAY_X + P / 2)

out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'meshes', 'torso_frame.stl')
out = os.path.normpath(out)
with open(out, 'wb') as f:
    f.write(b'magni torso aluminum extrusion frame'.ljust(80, b'\0'))
    f.write(struct.pack('<I', len(triangles)))
    for n, v0, v1, v2 in triangles:
        f.write(struct.pack('<12fH', *n, *v0, *v1, *v2, 0))

print(f'wrote {out}: {len(triangles)} triangles')
