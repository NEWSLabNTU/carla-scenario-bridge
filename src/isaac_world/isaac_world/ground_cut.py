"""Cutting a box out of a map's ground meshes (roadmap 047 S3): a soil zone with its own ground
(`ground_z`, `soil_tiles`) needs the map's ground gone from under it, or a wheel's suspension
ray meets the road above the ruts and the lidar sees the road over them. Pure numpy: the
geometry here, the USD glue in `cut_stage`.

`cut_mesh` drops every face inside the box and splits a face across its edge into the convex
pieces outside, the new vertices' attributes interpolated along the cut edges: positions,
vertex primvars (per point) and face-varying ones (per face corner, as UVs usually are);
uniform ones (per face) follow their face, and `face_map` says which old face each new one
came from, for material subsets. Faces wholly outside are kept as they were, indices and all.
"""
import numpy as np

EPS = 1e-9


def _split(poly, attrs, axis, value, keep_below):
    """Sutherland-Hodgman against the half-plane coordinate[axis] <= value (keep_below) or
    >= value: the polygon's part on that side, attributes interpolated."""
    out_p, out_a = [], []
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        a, b = attrs[i], attrs[(i + 1) % n]
        dp, dq = p[axis] - value, q[axis] - value
        if not keep_below:
            dp, dq = -dp, -dq
        if dp <= EPS:
            out_p.append(p)
            out_a.append(a)
        if (dp < -EPS and dq > EPS) or (dp > EPS and dq < -EPS):
            t = dp / (dp - dq)
            out_p.append(p + t * (q - p))
            out_a.append(a + t * (b - a))
    return out_p, out_a


def _area(poly):
    if len(poly) < 3:
        return 0.0
    v = np.array([p[:2] for p in poly])
    x, y = v[:, 0], v[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def outside_pieces(poly, attrs, box):
    """The convex pieces of convex polygon `poly` (list of xyz) outside `box` (x0, x1, y0, y1),
    with their attribute rows."""
    x0, x1, y0, y1 = box
    pieces = []
    rest, rest_a = list(poly), list(attrs)
    for axis, value, below_is_out in ((0, x0, True), (0, x1, False), (1, y0, True),
                                      (1, y1, False)):
        out_p, out_a = _split(rest, rest_a, axis, value, keep_below=below_is_out)
        if _area(out_p) > 1e-10:
            pieces.append((out_p, out_a))
        rest, rest_a = _split(rest, rest_a, axis, value, keep_below=not below_is_out)
        if _area(rest) <= 1e-10:
            break
    return pieces


def cut_mesh(points, counts, indices, box, vertex=None, face_varying=None, uniform=None,
             z_band=None):
    """`points` (N x 3, the box's frame), faces `counts` / `indices`, primvars by
    interpolation (dicts name -> array: per point, per face corner, per face) less `box`:
    (points, counts, indices, vertex, face_varying, uniform, face_map), or None when no face
    reaches into the box. `z_band` (lo, hi): only faces lying wholly between those heights
    are ground and cut; a guardrail's upright faces or a bridge's sides stay."""
    points = np.asarray(points, dtype=np.float64)
    counts = np.asarray(counts, dtype=np.int64)
    indices = np.asarray(indices, dtype=np.int64)
    vertex = {k: np.asarray(v) for k, v in (vertex or {}).items()}
    face_varying = {k: np.asarray(v) for k, v in (face_varying or {}).items()}
    uniform = {k: np.asarray(v) for k, v in (uniform or {}).items()}
    x0, x1, y0, y1 = box
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    px, py = points[indices, 0], points[indices, 1]
    fmin_x = np.minimum.reduceat(px, starts) if len(counts) else px
    fmax_x = np.maximum.reduceat(px, starts) if len(counts) else px
    fmin_y = np.minimum.reduceat(py, starts) if len(counts) else py
    fmax_y = np.maximum.reduceat(py, starts) if len(counts) else py
    outside = (fmax_x <= x0 + EPS) | (fmin_x >= x1 - EPS) | (fmax_y <= y0 + EPS) \
        | (fmin_y >= y1 - EPS)
    if z_band is not None and len(counts):
        pz = points[indices, 2]
        outside |= (np.minimum.reduceat(pz, starts) < z_band[0]) \
            | (np.maximum.reduceat(pz, starts) > z_band[1])
    inside = (fmin_x >= x0 - EPS) & (fmax_x <= x1 + EPS) & (fmin_y >= y0 - EPS) \
        & (fmax_y <= y1 + EPS)
    if outside.all():
        return None
    # A row of attributes per corner: position, its vertex primvars, its face-varying ones.
    vkeys, fkeys = sorted(vertex), sorted(face_varying)

    def width(a):
        return int(np.prod(a.shape[1:])) if a.ndim > 1 else 1

    def corner_rows(f):
        idx = indices[starts[f]:starts[f] + counts[f]]
        cols = [points[idx]]
        cols += [vertex[k][idx].reshape(len(idx), -1) for k in vkeys]
        cols += [face_varying[k][starts[f]:starts[f] + counts[f]].reshape(len(idx), -1)
                 for k in fkeys]
        return np.concatenate(cols, axis=1).astype(np.float64)

    new_points = [points]
    new_vertex = {k: [vertex[k]] for k in vkeys}
    out_counts, out_indices, face_map = [], [], []
    out_fv = {k: [] for k in fkeys}
    n_points = len(points)
    for f in range(len(counts)):
        if inside[f] and not outside[f]:
            continue
        if outside[f]:
            idx = indices[starts[f]:starts[f] + counts[f]]
            out_counts.append(int(counts[f]))
            out_indices.extend(idx.tolist())
            for k in fkeys:
                out_fv[k].append(face_varying[k][starts[f]:starts[f] + counts[f]])
            face_map.append(f)
            continue
        rows = corner_rows(f)
        # fan into triangles: a face across the edge may not be convex
        for t in range(1, counts[f] - 1):
            tri = [rows[0], rows[t], rows[t + 1]]
            for piece_p, piece_a in outside_pieces([r[:3] for r in tri], tri, box):
                piece = np.array(piece_a)
                m = len(piece)
                new_points.append(piece[:, :3])
                col = 3
                for k in vkeys:
                    w = width(vertex[k])
                    new_vertex[k].append(piece[:, col:col + w].reshape((m,) + vertex[k].shape[1:])
                                         .astype(vertex[k].dtype))
                    col += w
                for k in fkeys:
                    w = width(face_varying[k])
                    out_fv[k].append(piece[:, col:col + w].reshape(
                        (m,) + face_varying[k].shape[1:]).astype(face_varying[k].dtype))
                    col += w
                out_counts.append(m)
                out_indices.extend(range(n_points, n_points + m))
                n_points += m
                face_map.append(f)
    face_map = np.array(face_map, dtype=np.int64)
    return (np.concatenate(new_points), np.array(out_counts, dtype=np.int32),
            np.array(out_indices, dtype=np.int32),
            {k: np.concatenate(v) for k, v in new_vertex.items()},
            {k: (np.concatenate(v) if v else face_varying[k][:0]) for k, v in out_fv.items()},
            {k: uniform[k][face_map] for k in uniform}, face_map)


def remap_subset(subset_faces, face_map):
    """A GeomSubset's face indices after `cut_mesh`: the new faces that came from them."""
    keep = np.isin(face_map, np.asarray(subset_faces, dtype=np.int64))
    return np.nonzero(keep)[0].astype(np.int32)
