import FreeCAD, Import, Part, MeshPart
doc = FreeCAD.newDocument("h"); Import.insert("/home/maxou/Hexapode2026/CAD/Hexapod Bas1ma Haz1r/Hexapod.STEP", "h")
root = [o for o in doc.Objects if not o.InList][0]
# repere STEP (y vers le haut) -> repere robot (REP-103) : X = z_step, Y = x_step, Z = y_step - 18.5 (axes des hanches)
M = FreeCAD.Matrix(0,0,1,0, 1,0,0,0, 0,1,0,-18.5, 0,0,0,1)
shapes = []
for c in root.Group:
    if c.TypeId.startswith("Part::Feature") and any(k in c.Label for k in ("body plate", "Body side plate", "Standoff")):
        s = c.Shape.copy(); s.Placement = c.getGlobalPlacement(); s = s.transformGeometry(M); shapes.append(s)
        print(c.Label)
comp = Part.makeCompound(shapes)
bb = comp.BoundBox; print("bbox robot mm x[%.0f,%.0f] y[%.0f,%.0f] z[%.1f,%.1f]" % (bb.XMin,bb.XMax,bb.YMin,bb.YMax,bb.ZMin,bb.ZMax))
m = MeshPart.meshFromShape(Shape=comp, LinearDeflection=0.2, AngularDeflection=0.5, Relative=False)
m.write("MESHES_DIR/body.stl"); print("triangles", m.CountFacets)
