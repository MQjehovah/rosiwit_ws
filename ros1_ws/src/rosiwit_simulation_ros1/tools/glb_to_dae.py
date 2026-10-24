"""glb_to_dae.py - 把 KeyShot 导出的 GLB 转成 Gazebo/RViz 可用的 COLLADA(.dae)。

为什么不用 trimesh/assimp:
  * Gazebo 9 与 RViz(melodic) 都只认 STL/DAE/OBJ，不认 GLB；
  * trimesh 的 DAE 导出不支持贴图，GLB 里 30+ 张 baseColor 纹理丢失后整车会变白。
本脚本直接解析 GLB(JSON + BIN)，生成带贴图的 COLLADA 1.4.1：
  * 顶点/法线/UV 原样保留，节点变换与 Y-up->Z-up 一次性烘焙进顶点；
  * 贴图从 GLB 的 bufferView 里解出来写成 textures/img_XXX.jpg，DAE 用相对路径引用；
  * UV 的 V 轴翻转 (glTF 原点左上, COLLADA/OpenGL 原点左下)。

用法:
  py -3 glb_to_dae.py <input.glb> <output_dir> [--yaw-deg 180]

输出:
  <output_dir>/gt.dae
  <output_dir>/textures/img_XXX.jpg

注意: 生成的 DAE 是纯 ASCII + 数字, 不含中文, 避免 Python2 工具链解析问题。
"""

import argparse
import json
import math
import os
import struct

COMP = {
    5120: ("b", 1), 5121: ("B", 1), 5122: ("h", 2),
    5123: ("H", 2), 5125: ("I", 4), 5126: ("f", 4),
}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def load_glb(path):
    data = open(path, "rb").read()
    magic, version, _ = struct.unpack_from("<III", data, 0)
    if magic != 0x46546C67:
        raise ValueError("not a GLB file")
    gltf, bin_chunk = None, b""
    off = 12
    while off < len(data):
        clen, ctype = struct.unpack_from("<II", data, off)
        chunk = data[off + 8:off + 8 + clen]
        if ctype == 0x4E4F534A:
            gltf = json.loads(chunk.decode("utf-8"))
        elif ctype == 0x004E4942:
            bin_chunk = chunk
        off += 8 + clen
    return gltf, bin_chunk


def read_accessor(gltf, blob, index):
    acc = gltf["accessors"][index]
    ncomp = NCOMP[acc["type"]]
    fmt, size = COMP[acc["componentType"]]
    count = acc["count"]
    if "bufferView" not in acc:
        return [tuple([0.0] * ncomp)] * count
    bv = gltf["bufferViews"][acc["bufferView"]]
    start = bv.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = bv.get("byteStride") or ncomp * size
    if stride == ncomp * size:
        raw = struct.unpack_from("<" + fmt * ncomp * count, blob, start)
        return [raw[i:i + ncomp] for i in range(0, len(raw), ncomp)]
    return [struct.unpack_from("<" + fmt * ncomp, blob, start + i * stride)
            for i in range(count)]


def mat_mul(a, b):
    r = [0.0] * 16
    for c in range(4):
        for row in range(4):
            r[c * 4 + row] = sum(a[k * 4 + row] * b[c * 4 + k] for k in range(4))
    return r


def node_matrix(n):
    if "matrix" in n:
        return list(n["matrix"])
    t = n.get("translation", [0, 0, 0])
    q = n.get("rotation", [0, 0, 0, 1])
    s = n.get("scale", [1, 1, 1])
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        (1 - 2 * (yy + zz)) * s[0], (2 * (xy + wz)) * s[0], (2 * (xz - wy)) * s[0], 0.0,
        (2 * (xy - wz)) * s[1], (1 - 2 * (xx + zz)) * s[1], (2 * (yz + wx)) * s[1], 0.0,
        (2 * (xz + wy)) * s[2], (2 * (yz - wx)) * s[2], (1 - 2 * (xx + yy)) * s[2], 0.0,
        t[0], t[1], t[2], 1.0,
    ]


def mat3_of(m):
    return [[m[0], m[4], m[8]], [m[1], m[5], m[9]], [m[2], m[6], m[10]]]


def inverse_transpose3(a):
    det = (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
           - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
           + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    if abs(det) < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    inv = [
        [(a[1][1] * a[2][2] - a[1][2] * a[2][1]) / det,
         (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det,
         (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det],
        [(a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det,
         (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det,
         (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / det],
        [(a[1][0] * a[2][1] - a[1][1] * a[2][0]) / det,
         (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / det,
         (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det],
    ]
    return [[inv[r][c] for r in range(3)] for c in range(3)]  # transpose


def rot_x(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return [1, 0, 0, 0, 0, c, s, 0, 0, -s, c, 0, 0, 0, 0, 1]


def rot_z(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return [c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def fnum(v, prec=4):
    s = ("%." + str(prec) + "f") % v
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s or "0"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("glb")
    ap.add_argument("outdir")
    ap.add_argument("--name", default="gt")
    ap.add_argument("--yaw-deg", type=float, default=0.0)
    ap.add_argument("--reduce", type=float, default=0.0,
                    help="0..1, fraction of triangles to remove (quadric decimation)")
    ap.add_argument("--min-tris", type=int, default=2000,
                    help="only decimate geometries with more triangles than this")
    args = ap.parse_args()

    gltf, blob = load_glb(args.glb)
    os.makedirs(args.outdir, exist_ok=True)
    texdir = os.path.join(args.outdir, "textures")
    os.makedirs(texdir, exist_ok=True)

    # ---- 1. 解出贴图 ----
    images = gltf.get("images", [])
    image_paths = {}
    for i, img in enumerate(images):
        if "bufferView" not in img:
            continue
        bv = gltf["bufferViews"][img["bufferView"]]
        start = bv.get("byteOffset", 0)
        raw = blob[start:start + bv["byteLength"]]
        ext = ".png" if img.get("mimeType") == "image/png" else ".jpg"
        rel = "textures/img_%03d%s" % (i, ext)
        with open(os.path.join(args.outdir, rel), "wb") as f:
            f.write(raw)
        image_paths[i] = rel

    textures = gltf.get("textures", [])
    materials = gltf.get("materials", [])

    # ---- 2. 收集几何 (烘焙节点变换) ----
    # world = Rz(yaw) * Rx(90) * node_world  (glTF Y-up -> COLLADA Z-up)
    base_xform = mat_mul(rot_z(args.yaw_deg), rot_x(90.0))
    nodes = gltf["nodes"]
    meshes = gltf["meshes"]
    geoms = []  # dict: name, vertices, normals, uvs, indices, material

    def walk(idx, parent):
        n = nodes[idx]
        m = mat_mul(parent, node_matrix(n))
        if "mesh" in n:
            nm3 = inverse_transpose3(mat3_of(m))
            for pi, prim in enumerate(meshes[n["mesh"]]["primitives"]):
                if prim.get("mode", 4) != 4:
                    continue
                attrs = prim["attributes"]
                pos = read_accessor(gltf, blob, attrs["POSITION"])
                nrm = read_accessor(gltf, blob, attrs["NORMAL"]) if "NORMAL" in attrs else None
                uv = read_accessor(gltf, blob, attrs["TEXCOORD_0"]) if "TEXCOORD_0" in attrs else None
                if "indices" in prim:
                    idxs = [v[0] for v in read_accessor(gltf, blob, prim["indices"])]
                else:
                    idxs = list(range(len(pos)))

                verts = []
                for p in pos:
                    x = m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12]
                    y = m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13]
                    z = m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]
                    verts.append((x, y, z))

                normals = None
                if nrm is not None:
                    normals = []
                    for p in nrm:
                        x = nm3[0][0] * p[0] + nm3[0][1] * p[1] + nm3[0][2] * p[2]
                        y = nm3[1][0] * p[0] + nm3[1][1] * p[1] + nm3[1][2] * p[2]
                        z = nm3[2][0] * p[0] + nm3[2][1] * p[1] + nm3[2][2] * p[2]
                        ln = math.sqrt(x * x + y * y + z * z) or 1.0
                        normals.append((x / ln, y / ln, z / ln))

                uvs = [(u, 1.0 - v) for (u, v) in uv] if uv is not None else None
                geoms.append({
                    "name": "geo%03d" % len(geoms),
                    "v": verts, "n": normals, "uv": uvs,
                    "idx": idxs, "mat": prim.get("material"),
                })
        for c in n.get("children", []):
            walk(c, m)

    identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    for root in gltf["scenes"][gltf.get("scene", 0)]["nodes"]:
        walk(root, base_xform)

    # ---- 2b. 可选减面 (保 UV: 用 replay 的 indice_mapping 重映射属性) ----
    if args.reduce > 0.0:
        import numpy as np
        from fast_simplification import simplify, replay_simplification

        for g in geoms:
            tris = len(g["idx"]) // 3
            if tris <= args.min_tris:
                continue
            pts = np.asarray(g["v"], dtype=np.float32)
            faces = np.asarray(g["idx"], dtype=np.int32).reshape(-1, 3)
            try:
                _, _, collapses = simplify(pts, faces, target_reduction=args.reduce,
                                           return_collapses=True)
                new_pts, new_faces, mapping = replay_simplification(pts, faces, collapses)
            except Exception as e:
                print("  decimate skipped for %s: %s" % (g["name"], e))
                continue
            valid = mapping >= 0
            uv_arr = np.asarray(g["uv"], dtype=np.float32) if g["uv"] is not None else None
            nrm_arr = np.asarray(g["n"], dtype=np.float32) if g["n"] is not None else None
            n_new = int(mapping.max()) + 1 if mapping.size else 0
            new_uv = np.zeros((n_new, 2), dtype=np.float32) if uv_arr is not None else None
            new_nrm = np.zeros((n_new, 3), dtype=np.float32) if nrm_arr is not None else None
            if uv_arr is not None:
                new_uv[mapping[valid]] = uv_arr[valid]
            if nrm_arr is not None:
                new_nrm[mapping[valid]] = nrm_arr[valid]
                ln = np.linalg.norm(new_nrm, axis=1)
                ln[ln == 0] = 1.0
                new_nrm /= ln[:, None]
            g["v"] = [tuple(p) for p in new_pts.tolist()]
            g["idx"] = [int(i) for i in new_faces.reshape(-1).tolist()]
            g["uv"] = [tuple(p) for p in new_uv.tolist()] if new_uv is not None else None
            g["n"] = [tuple(p) for p in new_nrm.tolist()] if new_nrm is not None else None

    # ---- 3. 材质 / 贴图 ----
    used_mats = sorted({g["mat"] for g in geoms if g["mat"] is not None})
    mat_id = {}
    effects = []
    mat_xml = []
    images_used = set()
    for mi in used_mats:
        mat = materials[mi]
        pbr = mat.get("pbrMetallicRoughness", {})
        factor = pbr.get("baseColorFactor", [1, 1, 1, 1])
        tex = pbr.get("baseColorTexture")
        eid = "fx_%d" % mi
        mid = "mat_%d" % mi
        mat_id[mi] = mid
        if tex is not None:
            tex_index = tex["index"]
            src = textures[tex_index].get("source")
            rel = image_paths.get(src)
            if rel:
                images_used.add(src)
                sid = "samp_%d" % mi
                effects.append(
                    '<effect id="%s"><profile_COMMON>'
                    '<newparam sid="surf_%d"><surface type="2D"><init_from>img_%03d</init_from></surface></newparam>'
                    '<newparam sid="%s"><sampler2D><source>surf_%d</source></sampler2D></newparam>'
                    '<technique sid="common"><phong>'
                    '<emission><color>0 0 0 1</color></emission>'
                    '<ambient><color>0.25 0.25 0.25 1</color></ambient>'
                    '<diffuse><texture texture="%s" texcoord="UVSET0"/></diffuse>'
                    '<specular><color>0.05 0.05 0.05 1</color></specular>'
                    '<shininess><float>8</float></shininess>'
                    '</phong></technique></profile_COMMON></effect>'
                    % (eid, mi, src, sid, mi, sid))
            else:
                eid, mid = "fxc_%d" % mi, "matc_%d" % mi
                mat_id[mi] = mid
                effects.append(
                    '<effect id="%s"><profile_COMMON><technique sid="common"><phong>'
                    '<emission><color>0 0 0 1</color></emission>'
                    '<ambient><color>0.25 0.25 0.25 1</color></ambient>'
                    '<diffuse><color>%s %s %s 1</color></diffuse>'
                    '<specular><color>0.05 0.05 0.05 1</color></specular>'
                    '<shininess><float>8</float></shininess>'
                    '</phong></technique></profile_COMMON></effect>'
                    % (eid, fnum(factor[0]), fnum(factor[1]), fnum(factor[2])))
        else:
            eid, mid = "fx_%d" % mi, "mat_%d" % mi
            effects.append(
                '<effect id="%s"><profile_COMMON><technique sid="common"><phong>'
                '<emission><color>0 0 0 1</color></emission>'
                '<ambient><color>0.25 0.25 0.25 1</color></ambient>'
                '<diffuse><color>%s %s %s 1</color></diffuse>'
                '<specular><color>0.05 0.05 0.05 1</color></specular>'
                '<shininess><float>8</float></shininess>'
                '</phong></technique></profile_COMMON></effect>'
                % (eid, fnum(factor[0]), fnum(factor[1]), fnum(factor[2])))
        mat_xml.append('<material id="%s" name="%s"><instance_effect url="#%s"/></material>'
                       % (mid, mid, eid))

    img_xml = []
    for i in sorted(images_used):
        img_xml.append('<image id="img_%03d" name="img_%03d"><init_from>%s</init_from></image>'
                       % (i, i, image_paths[i]))

    # ---- 4. 几何 ----
    geo_xml = []
    inst_xml = []
    tri_total = 0
    for gi, g in enumerate(geoms):
        if not g["idx"]:
            continue
        gid = "geo_%03d" % gi
        nv = len(g["v"])
        positions = " ".join("%s %s %s" % (fnum(x), fnum(y), fnum(z)) for x, y, z in g["v"])
        parts = [
            '<geometry id="%s" name="%s"><mesh>' % (gid, g["name"]),
            '<source id="%s_pos"><float_array id="%s_pos_a" count="%d">%s</float_array>'
            '<technique_common><accessor source="#%s_pos_a" count="%d" stride="3">'
            '<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>'
            '</accessor></technique_common></source>' % (gid, gid, 3 * nv, positions, gid, nv),
        ]
        inputs = []
        if g["n"] is not None:
            normals = " ".join("%s %s %s" % (fnum(x), fnum(y), fnum(z)) for x, y, z in g["n"])
            parts.append(
                '<source id="%s_nrm"><float_array id="%s_nrm_a" count="%d">%s</float_array>'
                '<technique_common><accessor source="#%s_nrm_a" count="%d" stride="3">'
                '<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>'
                '</accessor></technique_common></source>' % (gid, gid, 3 * nv, normals, gid, nv))
        if g["uv"] is not None:
            uvs = " ".join("%s %s" % (fnum(u), fnum(v)) for u, v in g["uv"])
            parts.append(
                '<source id="%s_uv"><float_array id="%s_uv_a" count="%d">%s</float_array>'
                '<technique_common><accessor source="#%s_uv_a" count="%d" stride="2">'
                '<param name="S" type="float"/><param name="T" type="float"/>'
                '</accessor></technique_common></source>' % (gid, gid, 2 * nv, uvs, gid, nv))

        parts.append('<vertices id="%s_vtx"><input semantic="POSITION" source="#%s_pos"/></vertices>'
                     % (gid, gid))

        inputs.append('<input semantic="VERTEX" source="#%s_vtx" offset="0"/>' % gid)
        offset = 1
        if g["n"] is not None:
            inputs.append('<input semantic="NORMAL" source="#%s_nrm" offset="%d"/>' % (gid, offset))
            offset += 1
        if g["uv"] is not None:
            inputs.append('<input semantic="TEXCOORD" source="#%s_uv" offset="%d" set="0"/>' % (gid, offset))

        # p 数组: 每个顶点三个 offset -> 写 VERTEX/NORMAL/TEXCOORD 需要的分量
        n_inputs = offset + (1 if g["uv"] is not None else 0)
        p = []
        for vi in g["idx"]:
            for o in range(n_inputs):
                p.append(str(vi))
        tri_count = len(g["idx"]) // 3
        tri_total += tri_count
        mat_name = mat_id.get(g["mat"], "") if g["mat"] is not None else ""
        parts.append('<triangles material="%s" count="%d">%s<p>%s</p></triangles>'
                     % (mat_name, tri_count, "".join(inputs), " ".join(p)))
        parts.append('</mesh></geometry>')
        geo_xml.append("".join(parts))
        inst_xml.append('<instance_geometry url="#%s"/>' % gid)

    dae = (
        '<?xml version="1.0" encoding="utf-8"?>\n'
        '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">\n'
        '<asset><contributor><authoring_tool>glb_to_dae.py</authoring_tool></contributor>'
        '<unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>\n'
        '<library_images>%s</library_images>\n'
        '<library_effects>%s</library_effects>\n'
        '<library_materials>%s</library_materials>\n'
        '<library_geometries>%s</library_geometries>\n'
        '<library_visual_scenes><visual_scene id="Scene" name="Scene">'
        '<node id="%s" name="%s">%s</node>'
        '</visual_scene></library_visual_scenes>\n'
        '<scene><instance_visual_scene url="#Scene"/></scene>\n'
        '</COLLADA>\n'
        % ("".join(img_xml), "".join(effects), "".join(mat_xml),
           "".join(geo_xml), args.name, args.name, "".join(inst_xml))
    )

    out = os.path.join(args.outdir, args.name + ".dae")
    with open(out, "w", encoding="ascii") as f:
        f.write(dae)

    print("wrote %s (%.1f MB), geometries=%d, triangles=%d, textures=%d"
          % (out, os.path.getsize(out) / 1e6, len(geoms), tri_total, len(images_used)))


if __name__ == "__main__":
    main()
