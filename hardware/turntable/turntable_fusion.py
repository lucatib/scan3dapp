"""Scan3D motorised turntable: base, platter with dot inserts, gears, and an iPhone 13 Pro stand.

Run it in Fusion on an empty design (Utilities > Scripts and Add-Ins, or through the Fusion MCP). It switches the
design to direct modelling and builds every part as its own component; export the meshes from Fusion. All dimensions
are millimetres; change the parameters and run it again on a new design.

Layout (z up, table at z = 0, platter centre on the origin): motor on +X, electronics on -X, phone stand on -Y.
"""
import math
import os

import adsk.core
import adsk.fusion

# --- Platter and mat ----------------------------------------------------------------------------------------------
PLATTER_D = 240.0
PLATTER_T = 8.0             # top at PLATTER_Z0 + PLATTER_T
RING_R = 88.0               # TurntableMat.RingRadius
DOT_D = 9.0                 # TurntableMat.DotRadius * 2
HOLE_D = 4.5                # TurntableMat.HoleRadius * 2
FREE_R = 76.0               # TurntableMat.FreeRadius: dashed groove, the piece stands inside it
MAT_CODE = "100100001100101100111101000111011010"   # TurntableMat.Code, 1 = ring dot (with a hole)
INSERT_DEPTH = 2.4
INSERT_CLEAR = 0.15         # radial, between an insert and its pocket

# --- Bearings, axle, gears ----------------------------------------------------------------------------------------
BEARING_OD = 22.15          # 608, printed bore
AXLE_HEAD_AF = 13.2         # hex pocket for the M8 bolt's head, pressed in
SHAFT_BORE = (5.1, 3.1)     # pinion's double-D bore on the 28BYJ-48 shaft (5 mm, flats 3 mm apart)
MOTOR_BODY_HOLE = 28.8      # the 28BYJ-48 body (28 mm) hangs through the plate
BEARING_STACK = 14.0        # two 608 on top of each other
GEAR_MODULE = 1.0
GEAR_TEETH = 64
PINION_TEETH = 16
GEAR_BACKLASH = 0.15        # tooth thickness taken off each gear, at the pitch circle
CENTRE_EXTRA = 0.1          # added to the theoretical centre distance

# --- Base ---------------------------------------------------------------------------------------------------------
BASE_D = 230.0
BASE_WALL = 2.4
BASE_FLOOR = 2.4

# --- 28BYJ-48 -----------------------------------------------------------------------------------------------------
MOTOR_BODY_BELOW = 18.0     # the body's bottom below the ears' underside (motor model)
MOTOR_PLATE_TOP = BASE_FLOOR + 0.5 + MOTOR_BODY_BELOW   # the ears rest here; the body hangs 0.5 mm above the floor
MOTOR_FLATS = (4.6, 10.6)   # the shaft's flats above the plate (motor model)
MOTOR_EAR_T = 0.8           # the ears are flush with the shaft face
MOTOR_SHAFT_OFFSET = 8.0    # shaft to body centre
MOTOR_EAR_HOLES = 35.0
MOTOR_HOUSING = (31.0, 9.5)  # blue cover and wire exit, from the body's centre (reach, half width): the plate's notch

# --- Heights, all from the motor: pinion on the shaft's flats, platter just above it -----------------------------
PINION_Z = (MOTOR_PLATE_TOP + MOTOR_FLATS[0] + 0.1, MOTOR_PLATE_TOP + MOTOR_FLATS[1] + 0.4)
PLATTER_Z0 = PINION_Z[1] + 1.0      # platter underside; its top is PLATTER_Z0 + PLATTER_T
GEAR_Z0 = PINION_Z[0] - 0.5         # platter gear: GEAR_Z0 .. PLATTER_Z0
# Above the bearings, inside the hub: a printed thin nut with a collar underneath that bears on the upper inner race
# only (clear of the ZZ shields). Screwed down, it clamps both inner races onto the boss's ring and pulls the bolt's
# head up against its hex. The bolt cuts its own thread in the printed bore.
AXLE_NUT = (13.0, 4.0, 7.0)         # across flats, height, bore
AXLE_NUT_COLLAR = (11.5, 1.5)       # diameter, height
HUB_Z0 = PLATTER_Z0 - 2 - BEARING_STACK - 1.5   # 1.5 mm lower than flush with the lip, so the bolt goes through the nut
BASE_TOP = PLATTER_Z0 - 2           # the wall stops under the platter's rim

# Reference models (escornabot/3dmodel, MIT, in ./models), placed in the design by the script.
MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
MOTOR_MODEL = ("StepMotor_28BYJ-48.step", 7.875, 8.57)  # file, shaft offset and ears' underside in the model
ULN_MODEL = "DriverBoard_ULN2003.step"                  # PCB centred on the origin, underside at z = 0

# --- ULN2003 board --------------------------------------------------------------------------------------------------
ULN_POS = (35.0, 68.0)      # board centre in the base, beside the motor mount
ULN_HOLES = (30.2, 26.4)    # mounting holes (Ø3), measured on the model
ULN_STANDOFF = 5.0          # above the floor, clear of the solder joints
ULN_PILOT = 2.5             # M3 self-tapping screw into the standoff

# --- Start/stop: TTP223 capacitive touch module behind the wall ------------------------------------------------------
TOUCH_ANGLE, TOUCH_Z = 150.0, 18.0     # where (from +X, counter-clockwise) and the module's centre height
TOUCH_WALL = 2.0            # the wall is thinned to this behind the module, with a flat face for its pad
TTP223 = (11.0, 15.0, 1.6)  # module width, height (stood up, pins at the bottom), thickness

# --- ESP32-S3 devkit (2 x USB-C) ------------------------------------------------------------------------------
# Measured on the real model, ESP32-S3-WROOM-1_devkit_2xUSBC in the Scan3DTable Fusion project: PCB 57 x 28.3 x 1.5,
# pin rows 25.4 apart along almost all of its length, the module overhanging the far end by 6.4 mm, two USB-C at the
# near end. Pins down. It rests on two ribs between the pin rows, stops against a block under the module's overhang,
# and is held down by four snap lips over its long edges (outside the pins); the wall beside the notch stops it when
# the cable is pulled out.
ESP_MODEL_ID = "urn:adsk.wipprod:dm.lineage:Oi68pfRvQISsIDWMEH-G4Q"
ESP_L, ESP_W, ESP_PCB = 57.0, 28.3, 1.5
ESP_PIN_X = 12.7            # pin rows, from the board's middle; header plastic 2.54 wide around them
ESP_UNDER = 21.0            # clear height under the board for the pins and the Dupont plugs
ESP_USB_X = -111.2          # the board's USB end, just inside the wall
ESP_SNAPS = (10.0, 46.0)    # snap lips along the board, from its USB end
USB_NOTCH_W = 25.0          # the opening for the plugs; narrower than the board

# --- Phone stand --------------------------------------------------------------------------------------------------
# Adjustable cradle: a back plate with two slots, and two jaws with a V groove that take the phone's edges, whatever
# its thickness, so the phone stays centred on the plate. Each jaw is clamped by an M5 bolt (head in the jaw) and a
# knob behind the plate. Reference phones, shown or checked: width, thickness, height, and the rear camera block seen
# from the back (left edge to its sides, top edge to its ends, how far it stands out).
PHONES = [
    ("iPhone 13 Pro", 71.5, 7.65, 146.7, (1, 38, 1, 38, 3.5)),
    ("Galaxy S24 Ultra", 79.0, 8.6, 162.3, (7, 24, 9, 62, 2.0)),
    ("Galaxy S24 Ultra, rugged case", 87.0, 15.0, 170.0, (11, 28, 13, 66, 0.0)),
]
PHONE_T_MAX = 18.0          # thickest phone (with its case) the jaws take
GROOVE_OPEN = 20.0          # V groove: width at the jaw's face
GROOVE_DEPTH = 4.0
JAW_T = 10.0                # jaw, across
JAW_H = 60.0
JAW_FRONT = 12.0            # jaw reach in front of the phone's middle plane
JAW_SLOT = (28.0, 56.0)     # bolt travel along each slot, from the plate centre: jaw faces 46..102 mm apart
JAW_FLANGE = 2.8            # the jaw's back is a C that rides on the plate's top and bottom edges: slides, cannot turn
PLATE_CLEAR = 0.2           # between the C and the plate's edges
SLIDER_EXT = 14.0           # the C runs on past the jaw's outer side: a longer slider turns less in its clearance
PLATE_HALF_W = 64.0
PLATE_T = 6.0
PLATE_H = 30.0
FOOT_H = 4.0                # the phone stands on the jaws' feet
HINGE_Z = 70.0              # tilt axis height above the table
HINGE_BACK = 13.0           # tilt axis behind the cradle's back face (the M5 knob, Ø24, has to clear it)
CAMERA_DISTANCE = 250.0     # default phone position: platter centre to the phone's face
ARM_LEN = 240.0
# Tripod adapter: the cradle moves from the tower onto it with the same M5 bolt and knob, and keeps its tilt.
TRIPOD_NUT_AF = 11.4        # 1/4"-20 hex nut (7/16" across flats), slid in from the side
TRIPOD_NUT_T = 5.9
TRIPOD_AXIS_UP = 35.0       # tilt axis above the block: the cradle and the phone swing down to ~31 mm below it
TRIPOD_SHOW = (-220.0, 0.0) # where it is shown, on the table beside the stand (x, offset from the tower in y)
ARM_SLOT = (-355.0, -200.0)

app = adsk.core.Application.get()
tbm = adsk.fusion.TemporaryBRepManager.get()
UNION = adsk.fusion.BooleanTypes.UnionBooleanType
DIFF = adsk.fusion.BooleanTypes.DifferenceBooleanType
INTER = adsk.fusion.BooleanTypes.IntersectionBooleanType


def P(x, y, z=0.0):
    return adsk.core.Point3D.create(x / 10, y / 10, z / 10)


def V(x, y, z):
    return adsk.core.Vector3D.create(x, y, z)


def cyl(x, y, z0, z1, d):
    return tbm.createCylinderOrCone(P(x, y, z0), d / 20, P(x, y, z1), d / 20)


def cyl_pts(p0, p1, d):
    return tbm.createCylinderOrCone(P(*p0), d / 20, P(*p1), d / 20)


def box(x0, x1, y0, y1, z0, z1):
    obb = adsk.core.OrientedBoundingBox3D.create(
        P((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), V(1, 0, 0), V(0, 1, 0),
        (x1 - x0) / 10, (y1 - y0) / 10, (z1 - z0) / 10)
    return tbm.createBox(obb)


def union(a, *bs):
    for b in bs:
        tbm.booleanOperation(a, b, UNION)
    return a


def cut(a, *bs):
    for b in bs:
        tbm.booleanOperation(a, b, DIFF)
    return a


def move(t, dx, dy, dz):
    m = adsk.core.Matrix3D.create()
    m.translation = V(dx / 10, dy / 10, dz / 10)
    tbm.transform(t, m)
    return t


def frame(t, origin, xa, ya, za):
    """Maps the body's local axes onto xa, ya, za at origin (mm)."""
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(*origin), V(*xa), V(*ya), V(*za))
    tbm.transform(t, m)
    return t


def prisms(root, loops, z0, h):
    """Extrudes closed polygons (lists of (x, y) mm) from z0 up by h; one temporary body per loop."""
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.isComputeDeferred = True
    lines = sk.sketchCurves.sketchLines
    for loop in loops:
        pts = [adsk.core.Point3D.create(x / 10, y / 10, 0) for x, y in loop]
        first = prev = None
        for i in range(len(pts)):
            a = prev.endSketchPoint if prev else pts[0]
            b = first.startSketchPoint if i == len(pts) - 1 else pts[i + 1]
            prev = lines.addByTwoPoints(a, b)
            first = first or prev
    sk.isComputeDeferred = False
    profiles = adsk.core.ObjectCollection.create()
    for pr in sk.profiles:
        profiles.add(pr)
    ext = root.features.extrudeFeatures.addSimple(
        profiles, adsk.core.ValueInput.createByReal(h / 10),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    bodies = [ext.bodies.item(i) for i in range(ext.bodies.count)]
    out = [move(tbm.copy(b), 0, 0, z0) for b in bodies]
    for b in bodies:
        b.deleteMe()
    sk.deleteMe()
    return out


def pol(r, a):
    return (r * math.cos(a), r * math.sin(a))


def hexagon(af):
    r = af / math.sqrt(3)
    return [pol(r, math.pi / 6 + i * math.pi / 3) for i in range(6)]


def gear_outline(z, m, backlash, rot=0.0, flank_pts=8, root_pts=4):
    """Involute spur gear, 20 degrees; tooth 0 points along +X (turned by rot). Radial flank below the base circle."""
    a0 = math.radians(20)
    r = m * z / 2
    ra, rf, rb = r + m, r - 1.25 * m, r * math.cos(a0)

    def inv(a):
        return math.tan(a) - a

    half_pitch = math.pi / (2 * z) - (backlash / 2) / r

    def half(radius):
        return half_pitch + inv(a0) - inv(math.acos(rb / max(radius, rb)))

    r0 = max(rf, rb)
    radii = [r0 + (ra - r0) * i / flank_pts for i in range(flank_pts + 1)]
    pts = []
    for k in range(z):
        th = rot + 2 * math.pi * k / z
        if rf < rb:
            pts.append(pol(rf, th - half(rb)))
        pts += [pol(R, th - half(R)) for R in radii]
        pts += [pol(R, th + half(R)) for R in reversed(radii)]
        if rf < rb:
            pts.append(pol(rf, th + half(rb)))
        a1, a2 = th + half(r0), th + 2 * math.pi / z - half(r0)
        pts += [pol(rf, a1 + (a2 - a1) * j / root_pts) for j in range(1, root_pts)]
    return pts


def dots():
    """(x, y, is_ring) of every mat dot, laid out so that seen from above the platter matches the printed mat."""
    out = []
    for i, c in enumerate(MAT_CODE):
        a = 2 * math.pi * i / len(MAT_CODE)
        # The SVG draws mat +Z downwards on paper; Fusion's top view has +Y up, so y = -sin.
        out.append((RING_R * math.cos(a), -RING_R * math.sin(a), c == "1"))
    return out


# Display colours, by component name (the first prefix that matches); matte plastic.
COLORS = [
    ("Piatto", (240, 240, 235)), ("Inserti", (20, 20, 20)), ("Base", (90, 90, 95)),
    ("Pignone", (230, 120, 30)), ("Pomell", (230, 120, 30)), ("Prova", (230, 120, 30)),
    ("Supporto", (200, 60, 60)), ("RIF iPhone", (40, 40, 50)), ("RIF cuscinetti", (190, 190, 200)),
    ("RIF vite", (150, 150, 160)), ("Dado", (230, 120, 30)), ("RIF touch", (40, 120, 200)),
    ("Braccio", (70, 110, 170)), ("Torre", (70, 110, 170)), ("Ganascia", (70, 110, 170)), ("Attacco", (70, 110, 170)),
]


def appearance(name):
    """A matte plastic appearance in the design for the component name, created on first use."""
    rgb = next((c for prefix, c in COLORS if name.startswith(prefix)), None)
    if rgb is None:
        return None
    design = adsk.fusion.Design.cast(app.activeProduct)
    key = "Scan3D %d,%d,%d" % rgb
    ap = design.appearances.itemByName(key)
    if ap is None:
        src = app.materialLibraries.itemByName("Fusion Appearance Library").appearances.itemByName("Plastic - Matte (White)")
        ap = design.appearances.addByCopy(src, key)
        for i in range(ap.appearanceProperties.count):
            prop = ap.appearanceProperties.item(i)
            if isinstance(prop, adsk.core.ColorProperty) and prop.name == "Color":
                prop.value = adsk.core.Color.create(rgb[0], rgb[1], rgb[2], 255)
    return ap


def add_part(root, name, bodies):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    for i, b in enumerate(bodies):
        body = comp.bRepBodies.add(b)
        body.name = name if len(bodies) == 1 else f"{name} {i + 1}"
    ap = appearance(name)
    if ap:
        occ.appearance = ap
    return occ


# --- Parts --------------------------------------------------------------------------------------------------------

def platter(root):
    top = PLATTER_Z0 + PLATTER_T
    body = cyl(0, 0, PLATTER_Z0, top, PLATTER_D)
    gear = prisms(root, [gear_outline(GEAR_TEETH, GEAR_MODULE, GEAR_BACKLASH)], GEAR_Z0, PLATTER_Z0 - GEAR_Z0 + 0.1)[0]
    union(body, gear, cyl(0, 0, HUB_Z0, GEAR_Z0 + 0.1, BEARING_OD + 8))
    # Bearings from below; the upper outer race carries the platter. Clearance above for the axle's end.
    # The upper outer race carries the platter on a ring between the bearing bore and the 16.5 mm bore above it, where
    # the spacer and the nut sit.
    cut(body, cyl(0, 0, HUB_Z0 - 1, HUB_Z0 + BEARING_STACK, BEARING_OD), cyl(0, 0, HUB_Z0 - 1, top - 3, 16.5))
    for x, y, ring in dots():
        cut(body, cyl(x, y, top - INSERT_DEPTH, top + 1, DOT_D))
        if ring:
            union(body, cyl(x, y, top - INSERT_DEPTH - 0.1, top, HOLE_D))
    dashes = 60
    loops = []
    for k in range(dashes):
        a0 = 2 * math.pi * k / dashes
        arc = [a0 + (math.pi / dashes) * j / 4 for j in range(5)]
        loops.append([pol(FREE_R + 0.4, a) for a in arc] + [pol(FREE_R - 0.4, a) for a in reversed(arc)])
    cut(body, *prisms(root, loops, top - 0.4, 1.0))
    return body


def inserts():
    top = PLATTER_Z0 + PLATTER_T
    filled, rings = [], []
    for x, y, ring in dots():
        d = cyl(x, y, top - INSERT_DEPTH, top, DOT_D - 2 * INSERT_CLEAR)
        if ring:
            rings.append(cut(d, cyl(x, y, top - INSERT_DEPTH - 1, top + 1, HOLE_D + 2 * INSERT_CLEAR)))
        else:
            filled.append(d)
    return filled, rings


def shaft_x():
    return GEAR_MODULE * (GEAR_TEETH + PINION_TEETH) / 2 + CENTRE_EXTRA


def base(root):
    r_in = BASE_D / 2 - BASE_WALL
    body = cut(cyl(0, 0, 0, BASE_TOP, BASE_D), cyl(0, 0, BASE_FLOOR, BASE_TOP + 1, 2 * r_in))
    # Axle boss: M8x30 hex bolt from below, its head pressed into the hex; the 608 inner races sit on the ring.
    union(body, cyl(0, 0, 0, HUB_Z0 - 1, 20), cyl(0, 0, HUB_Z0 - 1, HUB_Z0, 11.5))
    cut(body, cyl(0, 0, -1, HUB_Z0 + 1, 8.4), prisms(root, [hexagon(AXLE_HEAD_AF)], -1, 6.5)[0])

    # Motor mount: plate with the body hanging through it, side walls down to the floor, merged into the wall.
    mx = shaft_x() + MOTOR_SHAFT_OFFSET
    pz0 = MOTOR_PLATE_TOP - 4
    union(body, box(28, 111, -27, 27, pz0, MOTOR_PLATE_TOP),
          box(28, 111, -27, -24, 0, pz0), box(28, 111, 24, 27, 0, pz0))
    cut(body, cyl(mx, 0, pz0 - 1, MOTOR_PLATE_TOP + 1, MOTOR_BODY_HOLE),
        box(mx, mx + MOTOR_HOUSING[0], -MOTOR_HOUSING[1], MOTOR_HOUSING[1], pz0 - 1, MOTOR_PLATE_TOP + 1),
        cyl(mx, -MOTOR_EAR_HOLES / 2, pz0 - 1, MOTOR_PLATE_TOP + 1, 3.3),
        cyl(mx, MOTOR_EAR_HOLES / 2, pz0 - 1, MOTOR_PLATE_TOP + 1, 3.3))

    adds, cuts = esp_holder()
    union(body, *adds)
    cut(body, *cuts)
    # ULN2003 board on four standoffs.
    ux, uy = ULN_POS
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = ux + sx * ULN_HOLES[0] / 2, uy + sy * ULN_HOLES[1] / 2
            union(body, cyl(x, y, 0, BASE_FLOOR + ULN_STANDOFF, 6.5))
            cut(body, cyl(x, y, BASE_FLOOR + 0.8, BASE_FLOOR + ULN_STANDOFF + 1, ULN_PILOT))
    # Start/stop: the touch module stands in a holder behind the thinned wall; a ring marks where to touch outside.
    adds, cuts = touch_mount()
    union(body, *[radial(b, TOUCH_ANGLE) for b in adds])
    cut(body, *[radial(b, TOUCH_ANGLE) for b in cuts])

    # Flat pad for the stand's arm on -Y: two M4 screws, nuts from inside.
    union(body, box(-25, 25, -119, -108, 0, 30))
    for x in (-15, 15):
        cut(body, cyl_pts((x, -125, 15), (x, -100, 15), 4.3))
        nut = prisms(root, [hexagon(7.1)], 0, 3.7)[0]
        cut(body, frame(nut, (x, -111.6, 15), (1, 0, 0), (0, 0, -1), (0, 1, 0)))
    return body


def radial(t, angle):
    """Places a body built with +X as the radial direction at angle (degrees) round the platter axis."""
    a = math.radians(angle)
    return frame(t, (0, 0, 0), (math.cos(a), math.sin(a), 0), (-math.sin(a), math.cos(a), 0), (0, 0, 1))


def touch_mount(wall_out=None):
    """(solids to add, solids to cut) of the touch module's holder, with +X radial and the wall's outside at wall_out:
    a flat face TOUCH_WALL inside it for the module's pad, two rails with lips behind the module's edges, a ledge
    under it (it slides in from the top), and a ring engraved outside over the pad."""
    r_out = BASE_D / 2 if wall_out is None else wall_out
    w, h, t = TTP223
    face = r_out - TOUCH_WALL
    z0 = TOUCH_Z - h / 2
    hw = w / 2 + 0.3
    back = face - t - 0.1
    adds = [box(back - 1.2, face + 0.6, -hw - 2.4, hw + 2.4, z0 - 1.5, z0)]                       # ledge
    for sy in (-1, 1):
        rail = sorted((sy * hw, sy * (hw + 2.4)))
        lip = sorted((sy * (hw - 0.9), sy * hw))
        adds += [box(back - 1.2, face + 0.6, rail[0], rail[1], z0 - 1.5, z0 + h),
                 box(back - 1.2, back, lip[0], lip[1], z0 - 1.5, z0 + h)]
    ring = cut(cyl_pts((r_out - 0.4, 0, TOUCH_Z), (r_out + 2, 0, TOUCH_Z), 12),
               cyl_pts((r_out - 1, 0, TOUCH_Z), (r_out + 3, 0, TOUCH_Z), 10))
    # Flat face: shaves the curved inside of the wall up to its top, so the module slides down onto the ledge.
    cuts = [box(face - 1.0, face, -hw, hw, z0, z0 + 50), ring]
    return adds, cuts


def touch_dummy():
    """The TTP223 module: PCB with its pad against the flat face, the chip on the back."""
    w, h, t = TTP223
    face = BASE_D / 2 - TOUCH_WALL
    z0 = TOUCH_Z - h / 2
    body = union(box(face - t, face, -w / 2, w / 2, z0, z0 + h),
                 box(face - t - 1.1, face - t, -1.5, 1.5, z0 + 3, z0 + 6))
    return radial(body, TOUCH_ANGLE)


def esp_holder():
    """(solids to add, solids to cut) of the ESP32 holder, in base coordinates."""
    x0, x1, hw = ESP_USB_X, ESP_USB_X + ESP_L, ESP_W / 2
    h = BASE_FLOOR + ESP_UNDER                  # underside of the board
    top = h + ESP_PCB
    rib = ESP_PIN_X - 1.27 - 0.6                # clear of the header plastic and the Dupont plugs
    adds = [box(x0 + 2, x0 + 8, -rib, rib, 0, h), box(x1 - 9, x1 - 3, -rib, rib, 0, h),
            box(x1 + 0.3, x1 + 3.3, -hw, hw, 0, top - 0.3)]      # stop: below the module's overhang
    for sy in (-1, 1):
        y_post = sorted((sy * (hw + 0.2), sy * (hw + 2.2)))
        y_lip = sorted((sy * (hw - 0.6), sy * (hw + 0.2)))       # outside the pins' solder joints
        for ly in ESP_SNAPS:
            adds += [box(x0 + ly - 3, x0 + ly + 3, y_post[0], y_post[1], 0, top + 0.9),
                     box(x0 + ly - 3, x0 + ly + 3, y_lip[0], y_lip[1], top + 0.3, top + 0.9)]
    cuts = [box(-BASE_D, x0, -USB_NOTCH_W / 2, USB_NOTCH_W / 2, h - 3, BASE_TOP + 1)]
    return adds, cuts


def esp_model(root):
    """The real board, from the Fusion project, placed in its holder; a plain box when the file cannot be found."""
    x0, top = ESP_USB_X, BASE_FLOOR + ESP_UNDER + ESP_PCB
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(x0, 0, top), V(0, -1, 0), V(1, 0, 0), V(0, 0, 1))   # model: USB end at y = 0, top at z = 0
    f = app.data.findFileById(ESP_MODEL_ID)
    if f is None:
        hw = ESP_W / 2
        return add_part(root, "RIF ESP32-S3 devkit", [box(x0, x0 + ESP_L, -hw, hw, top - ESP_PCB, top)])
    occ = root.occurrences.addByInsert(f, m, True)
    occ.transform2 = m
    return occ


def pinion(root, bore=SHAFT_BORE):
    sx = shaft_x()
    z0, z1 = PINION_Z
    body = prisms(root, [gear_outline(PINION_TEETH, GEAR_MODULE, GEAR_BACKLASH, rot=math.pi / PINION_TEETH)], z0, z1 - z0)[0]
    d, flats = bore
    hole = cyl(0, 0, z0 - 1, z1 + 1, d)
    # Double-D, flats facing +-X as on the motor model (its flats face the body's centre).
    tbm.booleanOperation(hole, box(-flats / 2, flats / 2, -3, 3, z0 - 1, z1 + 1), INTER)
    cut(body, hole)
    return move(body, sx, 0, 0)


def post_y():
    """The tilt axis, so that the phone's face towards the platter is CAMERA_DISTANCE from its centre."""
    return -CAMERA_DISTANCE - PHONES[0][2] / 2 - plate_back() - HINGE_BACK


def arm(root):
    y_end = -125 - ARM_LEN
    body = union(box(-25, 25, -125, -119, 0, 30), box(-15, 15, y_end, -119, 0, 10))
    gusset = prisms(root, [[(-125, 10), (-125, 30), (-150, 10)]], 0, 30)[0]
    union(body, frame(gusset, (-15, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0)))
    for x in (-15, 15):
        cut(body, cyl_pts((x, -130, 15), (x, -110, 15), 4.3))
    s0, s1 = ARM_SLOT
    cut(body, box(-3.25, 3.25, s0, s1, -1, 11), cyl(0, s0, -1, 11, 6.5), cyl(0, s1, -1, 11, 6.5),
        box(-5.2, 5.2, y_end - 1, s1 + 3, -1, 4.5))      # channel for the M6 head, open at the far end
    return body


def tower(root):
    py = post_y()
    # Tilted forward, the cradle and the phone's bottom swing back towards the post: the post stops well below the
    # axis and the 8 mm tab rising to it keeps behind the axis (good to 45 degrees).
    body = union(box(-22, 22, py - 45, py + 15, 10, 18), box(-10, 10, py - 12, py + 3, 18, HINGE_Z - 30),
                 box(-4, 4, py - 10, py + 3, HINGE_Z - 32, HINGE_Z), cyl_pts((-4, py, HINGE_Z), (4, py, HINGE_Z), 20))
    # Skirts down both sides of the arm: the foot slides along it and cannot turn round the bolt.
    for sx in (-1, 1):
        union(body, box(min(sx * 15.3, sx * 22), max(sx * 15.3, sx * 22), py - 45, py + 15, 1, 10.1))
    cut(body, cyl(0, py - 35, 0, 30, 6.5), cyl_pts((-6, py, HINGE_Z), (6, py, HINGE_Z), 5.3))
    return body


def plate_back():
    """Distance from the phone's middle plane to the back of the cradle plate."""
    return PHONE_T_MAX / 2 + 1 + PLATE_T


def tripod_adapter(root, axis):
    """Block with a 1/4"-20 nut for the tripod screw under it, and the tower's tab and M5 hole at axis (x, y, z)."""
    zt = -TRIPOD_AXIS_UP
    zb = zt - 14
    body = union(box(-20, 20, -14, 8, zb, zt), box(-4, 4, -10, 3, zt - 1, 0), cyl_pts((-4, 0, 0), (4, 0, 0), 20))
    nz = zb + 2
    cut(body, cyl_pts((-6, 0, 0), (6, 0, 0), 5.3),
        cyl(0, -3, zb - 1, nz + TRIPOD_NUT_T + 6, 6.6),                       # the tripod screw
        move(prisms(root, [hexagon(TRIPOD_NUT_AF)], 0, TRIPOD_NUT_T)[0], 0, -3, nz),
        box(-TRIPOD_NUT_AF / 2, TRIPOD_NUT_AF / 2, -3, 9, nz, nz + TRIPOD_NUT_T))  # slot to slide the nut in
    return move(body, *axis)


def tripod_axis_shown():
    return (TRIPOD_SHOW[0], post_y() + TRIPOD_SHOW[1], TRIPOD_AXIS_UP + 14)


def cradle_origin():
    """World position of the cradle's local origin on the hinge: phone's middle plane, bottom of the jaws."""
    return (0, post_y() + plate_back() + HINGE_BACK, HINGE_Z - 12)


def jaw_gap(w, t):
    """Distance between the jaw faces for a phone w wide and t thick (flat sides): its edges sit in the V grooves."""
    return w - 2 * GROOVE_DEPTH * (1 - t / GROOVE_OPEN)


def cradle_plate():
    front, back = -(PHONE_T_MAX / 2 + 1), -plate_back()
    zc = PLATE_H / 2
    body = box(-PLATE_HALF_W, PLATE_HALF_W, back, front, 0, PLATE_H)
    a, b = JAW_SLOT
    for sx in (-1, 1):
        cut(body, box(min(sx * a, sx * b), max(sx * a, sx * b), back - 1, front + 1, zc - 2.7, zc + 2.7),
            cyl_pts((sx * a, back - 1, zc), (sx * a, front + 1, zc), 5.4),
            cyl_pts((sx * b, back - 1, zc), (sx * b, front + 1, zc), 5.4))
    hy = back - HINGE_BACK
    for sx in (-1, 1):
        x0, x1 = sorted((sx * 4.2, sx * 9.2))
        union(body, box(x0, x1, hy, back + 0.5, 2, 22), cyl_pts((x0, hy, 12), (x1, hy, 12), 20))
    # Tilted forward, the plate's bottom swings back over the tower's tab: a notch lets it through.
    cut(body, cyl_pts((-10, hy, 12), (10, hy, 12), 5.3), box(-4.2, 4.2, back - 1, front + 1, -1, 8))
    return move(body, *cradle_origin())


def jaw(root, sx, gap):
    """The jaw on side sx (+1 / -1), its face gap / 2 from the middle."""
    x_in, x_out = sx * gap / 2, sx * (gap / 2 + JAW_T)
    xc, zc = sx * (gap / 2 + JAW_T / 2), PLATE_H / 2
    front = -(PHONE_T_MAX / 2 + 1)
    body = box(min(x_in, x_out), max(x_in, x_out), front, JAW_FRONT, 0, JAW_H)
    x_foot = x_in - sx * 10
    union(body, box(min(x_in, x_foot), max(x_in, x_foot), -GROOVE_OPEN / 2, GROOVE_OPEN / 2, 0, FOOT_H))
    v = [(x_in, -GROOVE_OPEN / 2), (x_in + sx * GROOVE_DEPTH, 0), (x_in, GROOVE_OPEN / 2)]
    groove = prisms(root, [v + [(x_in - sx * 1, GROOVE_OPEN / 2), (x_in - sx * 1, -GROOVE_OPEN / 2)]],
                    FOOT_H, JAW_H)[0]
    # M5 bolt: hex head in a slot open to the groove face, shank out of the back through the plate's slot.
    head = box(min(x_in - sx, xc + sx * 4.1), max(x_in - sx, xc + sx * 4.1), front + 3, front + 6.8, zc - 4.8, zc + 4.8)
    x0, x1 = sorted((x_in, x_out + sx * SLIDER_EXT))
    xe0, xe1 = sorted((x_out - sx * 0.1, x_out + sx * SLIDER_EXT))
    lo, hi = -PLATE_CLEAR, PLATE_H + PLATE_CLEAR
    union(body, box(x0, x1, -plate_back(), front + 0.1, hi, hi + JAW_FLANGE),          # over the plate's top edge
          box(x0, x1, -plate_back(), front + 0.1, lo - JAW_FLANGE, lo),                # under its bottom edge
          box(x0, x1, front, front + 3, lo - JAW_FLANGE, 0.1),                         # web under the jaw
          box(xe0, xe1, front, front + 3, lo - JAW_FLANGE, hi + JAW_FLANGE))           # web of the extension
    cut(body, groove, head, cyl_pts((xc, front - 1, zc), (xc, front + 5, zc), 5.3))
    return move(body, *cradle_origin())


def knob(root, d, h, nut_af, nut_depth, hole_d):
    body = cyl(0, 0, 0, h, d)
    for k in range(8):
        x, y = pol(d / 2 + 2, 2 * math.pi * k / 8)
        cut(body, cyl(x, y, -1, h + 1, 8))
    cut(body, cyl(0, 0, -1, h + 1, hole_d), prisms(root, [hexagon(nut_af)], h - nut_depth, nut_depth + 1)[0])
    return body


def phone_dummy(w, t, h, bump):
    """Screen towards the platter (+Y), so seen from the back its left edge is -X."""
    st, top = t / 2, FOOT_H + h
    x0, x1, z0, z1, out = bump
    body = box(-w / 2, w / 2, -st, st, FOOT_H, top)
    if out > 0:
        union(body, box(-w / 2 + x0, -w / 2 + x1, -st - out, -st + 0.1, top - z1, top - z0))
    return move(body, *cradle_origin())


def jaw_knob(root, sx, gap):
    knob5 = knob(root, 24, 10, 8.2, 4.5, 5.3)
    xc = sx * (gap / 2 + JAW_T / 2)
    o = cradle_origin()
    return frame(knob5, (o[0] + xc, o[1] - plate_back(), o[2] + PLATE_H / 2), (1, 0, 0), (0, 0, 1), (0, -1, 0))


def import_model(root, name, file, m):
    """Imports a STEP from ./models as a component placed by m; None when the file is missing."""
    path = os.path.join(MODELS_DIR, file)
    if not os.path.exists(path):
        return None
    im = app.importManager
    before = root.occurrences.count
    im.importToTarget(im.createSTEPImportOptions(path), root)
    occ = root.occurrences.item(before)
    occ.component.name = name
    occ.transform2 = m
    return occ


def motor_model(root):
    """The real 28BYJ-48: body centre opposite the pinion, shaft towards the platter, ears on the plate."""
    file, shaft, ears = MOTOR_MODEL
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(shaft_x() + shaft, 0, MOTOR_PLATE_TOP - ears), V(-1, 0, 0), V(0, -1, 0), V(0, 0, 1))
    if import_model(root, "RIF motore 28BYJ-48", file, m) is None:
        add_part(root, "RIF motore 28BYJ-48", [motor_dummy()])


def uln_model(root):
    m = adsk.core.Matrix3D.create()
    m.translation = V(ULN_POS[0] / 10, ULN_POS[1] / 10, (BASE_FLOOR + ULN_STANDOFF) / 10)
    import_model(root, "RIF ULN2003", ULN_MODEL, m)


def bearing_dummies():
    """Two 608ZZ (8 x 22 x 7) stacked in the platter's hub, inner races on the boss's ring."""
    out = []
    for k in range(2):
        z = HUB_Z0 + 7 * k
        ring = cut(cyl(0, 0, z, z + 7, 22), cyl(0, 0, z - 1, z + 8, 8))
        out.append(cut(ring, cut(cyl(0, 0, z + 6.6, z + 7.1, 19), cyl(0, 0, z + 6, z + 8, 12)),     # shields' recess,
                       cut(cyl(0, 0, z - 0.1, z + 0.4, 19), cyl(0, 0, z - 1, z + 1, 12))))      # both faces
    return out


def axle_nut(root):
    """The printed thin nut on the upper bearing: collar on the inner race, hex above it."""
    af, nh, bore = AXLE_NUT
    cd, ch = AXLE_NUT_COLLAR
    z = HUB_Z0 + BEARING_STACK
    nut = union(cyl(0, 0, z, z + ch + 0.1, cd), prisms(root, [hexagon(af)], z + ch, nh)[0])
    return cut(nut, cyl(0, 0, z - 1, z + ch + nh + 1, bore))


def axle_dummy(root):
    """The M8 x 30 hex bolt: head in the hex under the base, shank up through the boss and the bearings."""
    head = prisms(root, [hexagon(13.0)], 0, 5.3)[0]
    return union(head, cyl(0, 0, 5.3, 35.3, 8))


def motor_dummy():
    sx = shaft_x()
    mx = sx + MOTOR_SHAFT_OFFSET
    t = MOTOR_PLATE_TOP + MOTOR_EAR_T      # shaft face
    return union(cyl(mx, 0, t - 19, t, 28), box(mx - 3.5, mx + 3.5, -21, 21, t - MOTOR_EAR_T, t),
                 box(mx + 12, mx + 18, -8.5, 8.5, t - 17, t - 1),
                 cyl(sx, 0, t, t + 1.5, 9), cyl(sx, 0, t + 1.5, t + 10, 5))


# --- Test prints --------------------------------------------------------------------------------------------------
# Small pieces to set the fits on your printer before the long prints. Each ladder has three steps, marked with 1, 2
# and 3 dimples; put the step that fits into the matching parameter above.
INSERT_TESTS = (0.10, 0.15, 0.20)                   # INSERT_CLEAR
BEARING_TESTS = (22.0, 22.15, 22.3)                 # BEARING_OD
AXLE_HEAD_TESTS = (13.0, 13.2, 13.4)                # AXLE_HEAD_AF
SHAFT_TESTS = ((5.0, 3.0), (5.1, 3.1), (5.2, 3.2))  # SHAFT_BORE
TESTS_X = 420.0                                     # laid out beside the assembly


def dimples(body, x, y, z, n, top=True, pitch=1.6, d=1.0, depth=0.5):
    """n marks in a row along X centred on (x, y), in the face at height z (top face, or bottom when top is False)."""
    for i in range(n):
        xi = x + (i - (n - 1) / 2) * pitch
        cut(body, cyl(xi, y, z - depth, z + 1, d) if top else cyl(xi, y, z - 1, z + depth, d))
    return body


def test_platter():
    t = PLATTER_T
    body = box(0, 66, 0, 32, 0, t)
    for k in range(3):
        x = 11 + 22 * k
        cut(body, cyl(x, 10, t - INSERT_DEPTH, t + 1, DOT_D), cyl(x, 23, t - INSERT_DEPTH, t + 1, DOT_D))
        union(body, cyl(x, 23, t - INSERT_DEPTH - 0.1, t, HOLE_D))
        dimples(body, x, 29.8, t, k + 1)
    return body


def test_inserts():
    out = []
    for k, c in enumerate(INSERT_TESTS):
        x = 8 + 14 * k
        filled = dimples(cyl(x, 0, 0, INSERT_DEPTH, DOT_D - 2 * c), x, 0, 0, k + 1, top=False, pitch=1.4, depth=0.4)
        ring = cut(cyl(x, 14, 0, INSERT_DEPTH, DOT_D - 2 * c), cyl(x, 14, -1, INSERT_DEPTH + 1, HOLE_D + 2 * c))
        out += [filled, dimples(ring, x, 14 - 3.4, 0, k + 1, top=False, pitch=1.4, depth=0.4)]
    return out


def test_bearing(root):
    body = box(0, 90, 0, 56, 0, 8)
    for k in range(3):
        x = 15 + 30 * k
        cut(body, cyl(x, 15, 0.5, 9, BEARING_TESTS[k]), cyl(x, 15, -1, 1, 18),      # 608 from the top, push it out below
            cyl(x, 40, -1, 9, 8.4), prisms(root, [hexagon(AXLE_HEAD_TESTS[k])], -1, 6.5)[0])  # M8 head from below
        dimples(body, x, 52, 8, k + 1)
    return body


def test_pinions(root):
    out = []
    for k, (d, flats) in enumerate(SHAFT_TESTS):
        x = 10 + 24 * k
        body = prisms(root, [gear_outline(PINION_TEETH, GEAR_MODULE, GEAR_BACKLASH)], 0, PINION_Z[1] - PINION_Z[0])[0]
        hole = cyl(0, 0, -1, 10, d)
        tbm.booleanOperation(hole, box(-flats / 2, flats / 2, -3, 3, -1, 10), INTER)
        cut(body, hole)
        for i in range(k + 1):
            a = math.pi / 2 + (i - k / 2) * 0.45
            cut(body, cyl(4.6 * math.cos(a), 4.6 * math.sin(a), PINION_Z[1] - PINION_Z[0] - 0.5, 10, 0.9))
        out.append(move(body, x, 0, 0))
    return out


def test_motor_plate():
    body = box(-24, 32, -27, 27, 0, 4)
    return cut(body, cyl(0, 0, -1, 5, MOTOR_BODY_HOLE), box(0, 24, -9.5, 9.5, -1, 5),
               cyl(0, -MOTOR_EAR_HOLES / 2, -1, 5, 3.3), cyl(0, MOTOR_EAR_HOLES / 2, -1, 5, 3.3))


def test_slots():
    """The cradle plate's slot, a jaw's bolt-head slot (printed on its outer face, as the jaw) and the arm's channel."""
    zc = PLATE_H / 2
    plate = cut(box(0, 40, 0, PLATE_H, 0, PLATE_T), box(10, 30, zc - 2.7, zc + 2.7, -1, PLATE_T + 1),
                cyl(10, zc, -1, PLATE_T + 1, 5.4), cyl(30, zc, -1, PLATE_T + 1, 5.4))
    # A 10 mm slice of a jaw, jaw coordinates (face at x = 0, outer side at x = JAW_T, plate between y = -16 and -10).
    front, back = -(PHONE_T_MAX / 2 + 1), -plate_back()
    lo, hi = -PLATE_CLEAR, PLATE_H + PLATE_CLEAR
    jaw_cut = union(box(0, JAW_T, front, 10, lo - JAW_FLANGE, hi + JAW_FLANGE),
                    box(0, JAW_T, back, front + 0.1, hi, hi + JAW_FLANGE), box(0, JAW_T, back, front + 0.1, lo - JAW_FLANGE, lo))
    cut(jaw_cut, box(-1, JAW_T / 2 + 4.1, front + 3, front + 6.8, zc - 4.8, zc + 4.8),
        cyl_pts((JAW_T / 2, front - 1, zc), (JAW_T / 2, front + 5, zc), 5.3))
    jaw_piece = frame(jaw_cut, (50, 20, JAW_T), (0, 0, -1), (0, 1, 0), (1, 0, 0))
    arm_piece = move(cut(box(80, 110, 0, 30, 0, 10), box(91.75, 98.25, 5, 25, -1, 11), box(89.8, 100.2, -1, 31, -1, 4.5)),
                     15, 0, 0)
    return [plate, jaw_piece, arm_piece]


def test_esp():
    """The ESP32 holder on a piece of floor, with a straight piece of wall and its notch."""
    adds, cuts = esp_holder()
    x0, x1 = ESP_USB_X, ESP_USB_X + ESP_L
    wall_in = x0 - 0.5
    body = union(box(wall_in - BASE_WALL, x1 + 6, -18, 18, 0, BASE_FLOOR),
                 box(wall_in - BASE_WALL, wall_in, -18, 18, 0, BASE_TOP), *adds)
    cut(body, *cuts)
    return move(body, -(wall_in - BASE_WALL), 0, 0)


def test_uln():
    """The ULN2003 standoffs on a thin plate, to check the hole spacing against your board."""
    hx, hy = ULN_HOLES[0] / 2, ULN_HOLES[1] / 2
    body = box(-hx - 5, hx + 5, -hy - 5, hy + 5, 0, 2)
    for sx in (-1, 1):
        for sy in (-1, 1):
            union(body, cyl(sx * hx, sy * hy, 0, 2 + ULN_STANDOFF, 6.5))
            cut(body, cyl(sx * hx, sy * hy, 0.8, 2 + ULN_STANDOFF + 1, ULN_PILOT))
    return move(body, hx + 5, hy + 5, 0)


def test_touch():
    """A straight piece of the thinned wall with the touch module's holder, to try the touch through it."""
    adds, cuts = touch_mount(wall_out=0.0)
    z0 = TOUCH_Z - TTP223[1] / 2
    body = union(box(-BASE_WALL, 0, -12, 12, z0 - 5, z0 + TTP223[1] + 3), *adds)
    cut(body, *cuts)
    return move(body, BASE_WALL + 10, 12, 0)


def test_parts(root):
    """(name, bodies), laid out in a column beside the assembly."""
    parts = [
        ("Prova piatto", [test_platter()]),
        ("Prova inserti", test_inserts()),
        ("Prova cuscinetti", [test_bearing(root)]),
        ("Prova pignoni", test_pinions(root)),
        ("Prova motore", [move(test_motor_plate(), 24, 27, 0)]),
        ("Prova asole", test_slots()),
        ("Prova ESP32", [test_esp()]),
        ("Prova ULN2003", [test_uln()]),
        ("Prova touch", [test_touch()]),
    ]
    y = -150.0
    for _, bodies in parts:
        lo, hi = min(b.boundingBox.minPoint.y for b in bodies), max(b.boundingBox.maxPoint.y for b in bodies)
        for b in bodies:
            move(b, TESTS_X, y - lo * 10, 0)
        y += (hi - lo) * 10 + 15
    return parts


def run(_context):
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    root = design.rootComponent

    filled, rings = inserts()
    py = post_y()
    knob6 = move(knob(root, 30, 12, 10.2, 5.5, 6.5), 0, py - 35, 18)
    knob5 = frame(knob(root, 24, 10, 8.2, 4.5, 5.3), (9.7, py, HINGE_Z), (0, 1, 0), (0, 0, 1), (1, 0, 0))
    printed = [
        ("Base", [base(root)]),
        ("Piatto", [platter(root)]),
        ("Inserti pieni", filled),
        ("Inserti anello", rings),
        ("Pignone", [pinion(root)]),
        ("Braccio", [arm(root)]),
        ("Torre", [tower(root)]),
        ("Pomello M6", [knob6]),
        ("Pomello M5", [knob5]),
    ]
    printed.append(("Dado asse", [axle_nut(root)]))
    name, w, t, h, bump = PHONES[0]
    gap = jaw_gap(w, t)
    printed += [
        ("Supporto piastra", [cradle_plate()]),
        ("Ganascia sinistra", [jaw(root, -1, gap)]),
        ("Ganascia destra", [jaw(root, 1, gap)]),
        ("Pomelli ganasce", [jaw_knob(root, -1, gap), jaw_knob(root, 1, gap)]),
        ("Attacco treppiede", [tripod_adapter(root, tripod_axis_shown())]),
    ]
    reference = [(f"RIF {name}", [phone_dummy(w, t, h, bump)]), ("RIF touch TTP223", [touch_dummy()]), ("RIF cuscinetti 608ZZ", bearing_dummies()),
                 ("RIF vite M8x30", [axle_dummy(root)])]

    for name, bodies in printed + reference + test_parts(root):
        add_part(root, name, bodies)
    esp_model(root)
    motor_model(root)
    uln_model(root)
    print(f"{len(printed)} printed parts, {len(filled)} filled + {len(rings)} ring inserts; "
          f"stand post at y = {py:.1f} mm")
