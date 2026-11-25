#!/usr/bin/env python3
"""Convertit des STL (binaires) en COLLADA .dae avec <up_axis>Z_UP</up_axis>.

Pourquoi : un STL ne dit pas quel axe est "en haut". RViz suppose Z (convention ROS),
mais Foxglove suppose Y par defaut et tourne alors les pieces de 90 deg. Un .dae
declare son axe vertical : meme rendu dans RViz et Foxglove.

    python3 stl_to_dae.py meshes/*.stl          # ecrit meshes/<nom>.dae a cote
Les sommets identiques sont fusionnes ; les unites sont gardees (mm, scale dans l'URDF).
"""
import os
import struct
import sys

import numpy as np


def read_stl(path):
    data = open(path, "rb").read()
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    tri = rec["v"].reshape(-1, 3).astype(np.float64)
    verts, idx = np.unique(np.round(tri, 4), axis=0, return_inverse=True)
    return verts, idx.reshape(-1, 3)


def normals(verts, faces):
    fn = np.cross(verts[faces[:, 1]] - verts[faces[:, 0]], verts[faces[:, 2]] - verts[faces[:, 0]])
    vn = np.zeros_like(verts)
    for k in range(3):
        np.add.at(vn, faces[:, k], fn)
    ln = np.linalg.norm(vn, axis=1, keepdims=True)
    ln[ln == 0] = 1
    return vn / ln


def write_dae(path, name, verts, faces):
    vn = normals(verts, faces)
    fmt = lambda a: " ".join(f"{v:.4f}" for v in a.ravel())  # noqa: E731
    p = " ".join(str(i) for i in faces.ravel())
    nv, nf = len(verts), len(faces)
    xml = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries>
    <geometry id="{name}-mesh" name="{name}">
      <mesh>
        <source id="{name}-pos">
          <float_array id="{name}-pos-array" count="{nv * 3}">{fmt(verts)}</float_array>
          <technique_common><accessor source="#{name}-pos-array" count="{nv}" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common>
        </source>
        <source id="{name}-nrm">
          <float_array id="{name}-nrm-array" count="{nv * 3}">{fmt(vn)}</float_array>
          <technique_common><accessor source="#{name}-nrm-array" count="{nv}" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common>
        </source>
        <vertices id="{name}-vtx">
          <input semantic="POSITION" source="#{name}-pos"/>
          <input semantic="NORMAL" source="#{name}-nrm"/>
        </vertices>
        <triangles count="{nf}">
          <input semantic="VERTEX" source="#{name}-vtx" offset="0"/>
          <p>{p}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="scene">
      <node id="{name}" name="{name}"><instance_geometry url="#{name}-mesh"/></node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""
    open(path, "w", encoding="utf-8").write(xml)


def main():
    for src in sys.argv[1:]:
        name = os.path.splitext(os.path.basename(src))[0]
        verts, faces = read_stl(src)
        dst = os.path.splitext(src)[0] + ".dae"
        write_dae(dst, name, verts, faces)
        print(f"{dst} : {len(faces)} triangles, {len(verts)} sommets, {os.path.getsize(dst) / 1e6:.1f} Mo")


if __name__ == "__main__":
    main()
