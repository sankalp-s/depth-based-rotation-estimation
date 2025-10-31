import os
import sys
import sqlite3
import numpy as np
import math
import imageio.v2 as imageio
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull

# ---------- Configurable params ----------
RANSAC_ITERS = 1500
RANSAC_DIST_THRESH = 0.01   # meters (1 cm)
MIN_INLIERS = 50
FPS = 4
SAVE_PNGS = True
CMAP = "viridis"
# -----------------------------------------

def read_blobs_from_db(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT data FROM messages ORDER BY id;")
    rows = cur.fetchall()
    conn.close()
    return [r[0] for r in rows]

def parse_depth_from_blob(blob):
    """
    Parse a sensor_msgs/Image-like blob into a depth image (float32 meters).
    Heuristics: looks for 16UC1 or 32FC1, reads height/width from common offsets.
    """
    enc_16 = b"16UC1" in blob
    enc_32 = b"32FC1" in blob
    possible_offsets = [(44,48),(40,44),(32,36),(48,52)]
    height = width = None
    for ho, wo in possible_offsets:
        try:
            h = int.from_bytes(blob[ho:ho+4],'little')
            w = int.from_bytes(blob[wo:wo+4],'little')
            if 1 < h < 5000 and 1 < w < 5000:
                height, width = h, w
                break
        except Exception:
            continue
    if height is None:
        height, width = 480, 640

    if enc_32:
        bpp = 4
    else:
        bpp = 2

    expected_data_len = height * width * bpp
    header_len = len(blob) - expected_data_len
    if header_len < 0:
        raise RuntimeError("Blob too small to contain expected image bytes (height,width mismatch)")

    data = blob[header_len: header_len + expected_data_len]
    if bpp == 4:
        img = np.frombuffer(data, dtype=np.float32).reshape((height, width))
        depth_m = img.astype(np.float32)
    else:
        img = np.frombuffer(data, dtype=np.uint16).reshape((height, width)).astype(np.float32)
        if np.nanmedian(img) > 50:
            depth_m = img / 1000.0
        else:
            depth_m = img
    return depth_m

def depth_to_pointcloud(depth, fx=525.0, fy=525.0, cx=None, cy=None):
    H, W = depth.shape
    if cx is None: cx = W / 2.0
    if cy is None: cy = H / 2.0
    i, j = np.indices(depth.shape)
    z = depth
    x = (j - cx) * z / fx
    y = (i - cy) * z / fy
    pts = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    mask = np.isfinite(pts[:,2]) & (pts[:,2] > 0)
    return pts[mask]

def ransac_plane(points, iterations=RANSAC_ITERS, dist_thresh=RANSAC_DIST_THRESH):
    best_inliers = None
    best_plane = None
    n = points.shape[0]
    if n < 3:
        return None, None
    for _ in range(iterations):
        ids = np.random.choice(n, 3, replace=False)
        p1, p2, p3 = points[ids]
        v1 = p2 - p1; v2 = p3 - p1
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm == 0:
            continue
        normal = normal / norm
        d = -np.dot(normal, p1)
        dist = np.abs(np.dot(points, normal) + d)
        inliers = np.where(dist < dist_thresh)[0]
        if best_inliers is None or len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_plane = (normal, d)
    return best_plane, best_inliers

def project_points_to_plane_coords(points, plane_normal, origin=None):
    if origin is None:
        origin = points.mean(axis=0)
    u = np.cross(plane_normal, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(u) < 1e-6:
        u = np.array([1.0, 0.0, 0.0])
    u = u / np.linalg.norm(u)
    v = np.cross(plane_normal, u); v = v / np.linalg.norm(v)
    coords2d = np.dot(points - origin, np.stack([u, v], axis=1))
    return coords2d, origin, u, v

def compute_plane_measurements(depth, fx=525.0, fy=525.0):
    pts = depth_to_pointcloud(depth, fx, fy)
    plane, inliers_idx = ransac_plane(pts)
    if plane is None or inliers_idx is None or len(inliers_idx) < MIN_INLIERS:
        return None
    normal, d = plane
    if normal[2] > 0:
        normal = -normal; d = -d
    inlier_pts = pts[inliers_idx]
    coords2d, origin, u, v = project_points_to_plane_coords(inlier_pts, normal, None)
    try:
        hull = ConvexHull(coords2d)
        area = hull.volume if not hasattr(hull, "area") else hull.area
        hull_pts_2d = coords2d[hull.vertices]
    except Exception:
        mins = coords2d.min(axis=0); maxs = coords2d.max(axis=0)
        area = float(np.prod(maxs - mins))
        hull_pts_2d = np.array([ [mins[0], mins[1]], [maxs[0], mins[1]], [maxs[0], maxs[1]], [mins[0], maxs[1]] ])
    cam_norm = np.array([0.0, 0.0, 1.0])
    angle_rad = math.acos(max(-1.0, min(1.0, np.dot(normal, cam_norm) / (np.linalg.norm(normal) * np.linalg.norm(cam_norm)))))
    angle_deg = math.degrees(angle_rad)
    return {
        "normal": normal,
        "d": d,
        "inlier_pts": inlier_pts,
        "coords2d": coords2d,
        "origin": origin,
        "u": u, "v": v,
        "hull_pts_2d": hull_pts_2d,
        "area": area,
        "angle_deg": angle_deg
    }

def overlay_and_save(depth, measure, png_path, caption_text=None, cmap=CMAP, vmin=None, vmax=None):
    fig, ax = plt.subplots(figsize=(8,6), dpi=100)
    im = ax.imshow(depth, cmap=cmap, vmin=vmin, vmax=vmax)
    ax.axis('off')

    if measure is not None:
        pts3d = measure["inlier_pts"]
        xs = pts3d[:,0]; ys = pts3d[:,1]; zs = pts3d[:,2]
        H, W = depth.shape
        fx = fy = 525.0
        cx = W / 2.0; cy = H / 2.0
        js = (fx * xs / zs + cx)
        is_ = (fy * ys / zs + cy)
        step = max(1, len(js) // 2000)
        ax.scatter(js[::step], is_[::step], s=1.2, c='white', alpha=0.6)

        hull_2d = measure["hull_pts_2d"]
        if hull_2d is not None and hull_2d.shape[0] >= 3:
            origin = measure["origin"]
            u = measure["u"]; v = measure["v"]
            hull_3d = np.array([ origin + pt[0]*u + pt[1]*v for pt in hull_2d ])
            hx = hull_3d[:,0]; hy = hull_3d[:,1]; hz = hull_3d[:,2]
            hj = (fx * hx / hz + cx)
            hi = (fy * hy / hz + cy)
            poly_pts = np.vstack([hj, hi]).T
            polygon = Polygon(poly_pts, closed=True, facecolor=(1,0,0,0.25), edgecolor=(1,0,0,0.9), linewidth=1.2)
            ax.add_patch(polygon)

    if caption_text:
        ax.text(0.01, 0.97, caption_text, transform=ax.transAxes, fontsize=10,
                va='top', ha='left', color='white',
                bbox=dict(facecolor='black', alpha=0.45, pad=4, edgecolor='none'))

    cb = fig.colorbar(im, ax=ax, fraction=0.036, pad=0.03)
    cb.set_label("Distance (m)")

    # Save with fixed figure size - avoid bbox_inches='tight' to keep consistent canvas
    plt.savefig(png_path, bbox_inches=None, pad_inches=0)
    plt.close(fig)

def make_video_from_pngs(png_files, out_dir, gif_name="depth_animation_with_inliers.gif",
                         mp4_name="depth_animation_with_inliers.mp4", fps=FPS):
    # Read images as numpy arrays
    imgs = [imageio.imread(p) for p in png_files]
    # Ensure RGB
    imgs_rgb = []
    for im in imgs:
        if im.ndim == 3 and im.shape[2] == 4:
            alpha = im[:, :, 3] / 255.0
            rgb = (im[:, :, :3].astype(np.float32) * alpha[:, :, None] + 0 * (1 - alpha[:, :, None])).astype(np.uint8)
            imgs_rgb.append(rgb)
        else:
            if im.ndim == 2:
                im = np.stack([im, im, im], axis=2)
            imgs_rgb.append(im)

    heights = [im.shape[0] for im in imgs_rgb]
    widths  = [im.shape[1] for im in imgs_rgb]
    Hmax = max(heights)
    Wmax = max(widths)

    def roundup(x, m): return int(math.ceil(x / m) * m)
    Wtarget = roundup(Wmax, 16)
    Htarget = roundup(Hmax, 16)

    print(f"Normalizing frames to size: {Wtarget}x{Htarget} (width x height)")

    norm_imgs = []
    for im in imgs_rgb:
        h, w = im.shape[0], im.shape[1]
        canvas = np.zeros((Htarget, Wtarget, 3), dtype=np.uint8)
        top = (Htarget - h) // 2
        left = (Wtarget - w) // 2
        canvas[top:top+h, left:left+w] = im
        norm_imgs.append(canvas)

    gif_path = os.path.join(out_dir, gif_name)
    imageio.mimsave(gif_path, norm_imgs, duration=1.0/fps)
    print("Saved GIF ->", gif_path)

    mp4_path = os.path.join(out_dir, mp4_name)
    # use macro_block_size=1 to prevent forced resizing by ffmpeg plugin
    imageio.mimsave(mp4_path, norm_imgs, fps=fps, macro_block_size=1)
    print("Saved MP4 ->", mp4_path)
    return gif_path, mp4_path

def main():
    if len(sys.argv) < 3:
        print("Usage: python create_depth_animation_with_inliers.py ./depth.db3 out_dir")
        sys.exit(1)
    db_path = sys.argv[1]
    out_dir = sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)
    png_dir = os.path.join(out_dir, "frames_png")
    if SAVE_PNGS:
        os.makedirs(png_dir, exist_ok=True)

    print("Reading blobs from DB:", db_path)
    blobs = read_blobs_from_db(db_path)
    if not blobs:
        raise RuntimeError("No blobs found in DB")

    # Try to load precomputed results.csv in out_dir (optional)
    precomputed = {}
    results_csv = os.path.join(out_dir, "results.csv")
    if os.path.exists(results_csv):
        try:
            import csv
            with open(results_csv, newline='') as f:
                r = csv.reader(f)
                header = next(r)
                for row in r:
                    frame = int(row[0])
                    nx, ny, nz = float(row[1]), float(row[2]), float(row[3])
                    angle = float(row[4]); area = float(row[5])
                    precomputed[frame] = {"normal": np.array([nx, ny, nz]), "angle_deg": angle, "area": area}
            print("Loaded precomputed results for frames:", list(precomputed.keys()))
        except Exception:
            precomputed = {}

    depths = []
    for idx, blob in enumerate(blobs, start=1):
        try:
            depth = parse_depth_from_blob(blob)
            depths.append(depth)
        except Exception as e:
            print(f"Skipping frame {idx}: parse error: {e}")
            depths.append(None)

    valid_depths = [d for d in depths if d is not None]
    if not valid_depths:
        raise RuntimeError("No valid depth frames parsed.")
    vmin = float(np.nanmin([np.nanmin(d) for d in valid_depths]))
    vmax = float(np.nanmax([np.nanmax(d) for d in valid_depths]))
    print(f"colormap scale: vmin={vmin:.3f} m, vmax={vmax:.3f} m")

    png_files = []
    for i, depth in enumerate(depths, start=1):
        if depth is None:
            print(f"Frame {i}: no depth, skipping")
            continue
        print(f"Processing frame {i} ...")
        measure = None
        caption = f"Frame {i}"
        pre = precomputed.get(i, None)
        try:
            measure = compute_plane_measurements(depth)
            if measure is not None:
                caption = f"Frame {i} — angle: {measure['angle_deg']:.1f}°  area: {measure['area']:.3f} m²"
            elif pre is not None:
                caption = f"Frame {i} — angle: {pre['angle_deg']:.1f}°  area: {pre['area']:.3f} m²"
            else:
                caption = f"Frame {i} — plane not found"
        except Exception as e:
            print(f"Warning: plane computation failed for frame {i}: {e}")
            if pre is not None:
                caption = f"Frame {i} — angle: {pre['angle_deg']:.1f}°  area: {pre['area']:.3f} m²"
            else:
                caption = f"Frame {i} — plane failed"

        png_path = os.path.join(png_dir if SAVE_PNGS else out_dir, f"frame_{i:03d}.png")
        overlay_and_save(depth, measure, png_path, caption_text=caption, cmap=CMAP, vmin=vmin, vmax=vmax)
        png_files.append(png_path)

    gif_path, mp4_path = make_video_from_pngs(png_files, out_dir,
                                              gif_name="depth_animation_with_inliers.gif",
                                              mp4_name="depth_animation_with_inliers.mp4",
                                              fps=FPS)
    print("All done. Frames saved to:", png_dir if SAVE_PNGS else out_dir)
    print("Outputs:", gif_path, mp4_path)

if __name__ == "__main__":
    main()
