import FreeCAD, Import, Part, MeshPart, math, json, re
V = FreeCAD.Vector
doc = FreeCAD.newDocument("h"); Import.insert("/home/maxou/Hexapode2026/CAD/Hexapod Bas1ma Haz1r/Hexapod.STEP", "h")
root = [o for o in doc.Objects if not o.InList][0]
M = FreeCAD.Matrix(0,0,1,0, 1,0,0,0, 0,1,0,-18.5, 0,0,0,1)   # STEP (y haut) -> robot (z haut), hanches a z=0
HW = re.compile(r"(nut|screw|washer)", re.I)
OUT = "MESHES_DIR"

def gshape(f):
    s = f.Shape.copy(); s.Placement = f.getGlobalPlacement(); return s.transformGeometry(M)
def feats(p):
    for c in p.Group:
        if c.TypeId == "App::Part": yield from feats(c)
        elif c.TypeId.startswith("Part::") and not HW.search(c.Label) and not c.Shape.isNull() and c.Shape.Volume > 1e-6: yield c
def servo_axis(p):
    pl = p.getGlobalPlacement(); o = M.multVec(pl.Base); d = M.multVec(pl.Base + pl.Rotation.multVec(V(0,0,1))) - o
    return o, d
def dist_line(pt, o, d):
    v = pt - o; return (v - d * v.dot(d)).Length
def frame(o, x, z):
    y = z.cross(x); m = FreeCAD.Matrix(x.x, y.x, z.x, o.x, x.y, y.y, z.y, o.y, x.z, y.z, z.z, o.z, 0,0,0,1); return m
def write(shapes, name, defl=0.3):
    comp = Part.makeCompound(shapes)
    mesh = MeshPart.meshFromShape(Shape=comp, LinearDeflection=defl, AngularDeflection=0.6, Relative=False)
    mesh.write(f"{OUT}/{name}.stl"); return mesh.CountFacets

legs = {c.Label: c for c in root.Group if c.TypeId == "App::Part"}
geo = {}
body_extra = []
for side, label in (("right", "Leg001"), ("left", [k for k in legs if k.endswith("Leg001") and k != "Leg001"][0])):
    leg = legs[label]
    subs = {re.sub(r"^Sym.{1,4}ie", "", c.Label).rstrip("0123456789"): c for c in leg.Group if c.TypeId == "App::Part"}
    servos = {}
    for key in ("Coxa", "Femur", "Tibia"):
        sv = [s for s in subs[key].Group if s.TypeId == "App::Part" and "AX-18F" in s.Label][0]
        servos[key] = (sv, servo_axis(sv))
    Oc, _ = servos["Coxa"][1]; Of, df = servos["Femur"][1]; Ok, dk = servos["Tibia"][1]
    Oc = V(Oc.x, Oc.y, 0)
    u = V(Of.x - Oc.x, Of.y - Oc.y, 0); u.normalize(); z = V(0,0,1)
    rf = lambda p: ((p - Oc).dot(u), p.z)                      # (r, z) dans le plan de la patte
    rub = [c for c in leg.Group if "Rubber" in c.Label][0]; bb = gshape(rub).BoundBox
    F = V(bb.Center.x, bb.Center.y, bb.ZMin)
    (rF, zF), (rK, zK), (rFe, zFe) = rf(F), rf(Ok), rf(Of)
    coxa = rFe; femur = math.hypot(rK - rFe, zK - zFe); thf = math.atan2(zK - zFe, rK - rFe)
    tib = math.hypot(rF - rK, zF - zK); thfoot = math.atan2(zF - zK, rF - rK)
    geo[side] = dict(coxa=coxa, femur=femur, femur_deg=math.degrees(thf), knee_foot=tib, foot_abs_deg=math.degrees(thfoot), hip_r=math.hypot(Oc.x, Oc.y))
    print(side, {k: round(v, 2) for k, v in geo[side].items()})
    json.dump(geo, open(OUT + "/../config/cad_geometry.json", "w"), indent=1)

# ---------------- export des maillages par segment ----------------
BEND, PERP = math.radians(-45.0), -20.0            # calibration conservee (tibia cintre, decalage du pied)
def tibia_len(target):
    a, b = 1.0, 2 * (-PERP) * (-math.sin(BEND)) * 1.0, 
    # |(lt cos B, lt sin B + PERP)| = target
    c = PERP * PERP - target * target; b = 2 * PERP * math.sin(BEND)
    return (-b + math.sqrt(b * b - 4 * c)) / 2
counts = {}
for side, label in (("right", "Leg001"), ("left", [k for k in legs if k.endswith("Leg001") and k != "Leg001"][0])):
    g = geo[side]; leg = legs[label]
    subs = {re.sub(r"^Sym.{1,4}ie", "", c.Label).rstrip("0123456789"): c for c in leg.Group if c.TypeId == "App::Part"}
    ax = {}
    for key in ("Coxa", "Femur", "Tibia"):
        sv = [s for s in subs[key].Group if s.TypeId == "App::Part" and "AX-18F" in s.Label][0]
        ax[key] = (sv,) + servo_axis(sv)
    Oc = V(ax["Coxa"][1].x, ax["Coxa"][1].y, 0); Of = ax["Femur"][1]; Ok = ax["Tibia"][1]
    u = V(Of.x - Oc.x, Of.y - Oc.y, 0); u.normalize(); zup = V(0, 0, 1)
    lt = tibia_len(g["knee_foot"]); phi = math.atan2(lt * math.sin(BEND) + PERP, lt * math.cos(BEND))
    g["tibia_len"] = lt; g["phi_deg"] = math.degrees(phi)
    def leg_frame(origin_rz, ang):
        r, zz = origin_rz; o = Oc + u * r + zup * zz
        x = u * math.cos(ang) + zup * math.sin(ang); z = u * (-math.sin(ang)) + zup * math.cos(ang)
        return frame(o, x, z)
    thf = math.radians(g["femur_deg"]); tht = math.radians(g["foot_abs_deg"]) - phi
    knee_rz = (g["coxa"] + g["femur"] * math.cos(thf), g["femur"] * math.sin(thf))
    T = {"coxa": leg_frame((0, 0), 0.0), "femur": leg_frame((g["coxa"], 0), thf), "tibia": leg_frame(knee_rz, tht)}
    g["cad_joint_deg"] = [0.0, g["femur_deg"], math.degrees(tht - thf)]
    lines = {"Coxa": (ax["Coxa"][1], ax["Coxa"][2]), "Femur": (ax["Femur"][1], ax["Femur"][2]), "Tibia": (ax["Tibia"][1], ax["Tibia"][2])}
    # Montage reel : le servo du femur fait partie du femur, celui du genou fait partie du
    # tibia (ils tournent avec leur segment) ; leur palonnier est fixe sur le segment amont.
    # Le servo de hanche est fixe dans le corps, son palonnier tourne avec la coxa.
    servo_link = {"Coxa": "body", "Femur": "femur", "Tibia": "tibia"}
    horn_link = {"Coxa": "coxa", "Femur": "coxa", "Tibia": "femur"}
    sub_link = {"Coxa": "coxa", "Femur": "femur", "Tibia": "tibia"}
    buckets = {"body": [], "coxa": [], "femur": [], "tibia": []}
    def nearest_axis(pt):
        return min(lines, key=lambda k: dist_line(pt, *lines[k]))
    for key, sub in subs.items():
        if key not in sub_link: continue
        for c in sub.Group:
            if c.TypeId == "App::Part" and "AX-18F" in c.Label:
                k = nearest_axis(M.multVec(c.getGlobalPlacement().Base))
                buckets[servo_link[k]] += [gshape(f) for f in feats(c)]
            elif c.TypeId == "App::Part" and "Horn" in c.Label:
                for f in feats(c):
                    s = gshape(f); buckets[horn_link[nearest_axis(s.BoundBox.Center)]].append(s)
            elif c.TypeId == "App::Part":
                buckets[sub_link[key]] += [gshape(f) for f in feats(c)]
            elif c.TypeId.startswith("Part::") and not HW.search(c.Label):
                buckets[sub_link[key]].append(gshape(c))
    for c in leg.Group:                                   # pied, entretoises : avec le tibia
        if c.TypeId.startswith("Part::") and not HW.search(c.Label):
            buckets["tibia"].append(gshape(c))
    for name in ("coxa", "femur", "tibia"):
        inv = T[name].inverse()
        shapes = [s.transformGeometry(inv) for s in buckets[name]]
        counts[f"leg_{side}_{name}"] = write(shapes, f"leg_{side}_{name}", 0.25)
    # servo de hanche : dans le repere de la hanche (coxa a 0), ajoute au corps pour chaque patte
    inv = T["coxa"].inverse()
    counts[f"hip_servo_{side}"] = write([s.transformGeometry(inv) for s in buckets["body"]], f"hip_servo_{side}", 0.25)
    print(side, "lt", round(lt, 2), "phi", round(math.degrees(phi), 2), "angles CAD", [round(a, 2) for a in g["cad_joint_deg"]],
          {k: len(v) for k, v in buckets.items()})
json.dump(geo, open(OUT + "/../config/cad_geometry.json", "w"), indent=1)
print(counts)
