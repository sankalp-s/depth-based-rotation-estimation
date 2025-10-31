import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import imageio
import os
import sys

def read_all_depth_frames(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    
    try:
        cur.execute("SELECT data FROM messages ORDER BY id;")
        rows = cur.fetchall()
    except Exception as e:
        conn.close()
        raise RuntimeError("Failed to read messages from sqlite DB: " + str(e))
    conn.close()
    images = []
    for (blob,) in rows:
        try:
            enc = None
            if b'32FC1' in blob:
                enc = '32FC1'
            elif b'16UC1' in blob:
                enc = '16UC1'
            
            height = int.from_bytes(blob[44:48], 'little')
            width  = int.from_bytes(blob[48:52], 'little')
            bpp = 4 if enc == '32FC1' else 2
            header_len = len(blob) - (height * width * bpp)
            data = blob[header_len: header_len + height * width * bpp]
            if enc == '32FC1':
                img = np.frombuffer(data, dtype=np.float32).reshape((height, width))
                images.append(img.astype(np.float32))
            else:
                img = np.frombuffer(data, dtype=np.uint16).reshape((height, width)).astype(np.float32)
                # auto-detect units: if median > 50 assume millimeters
                if np.nanmedian(img) > 50:
                    img = img / 1000.0
                images.append(img)
        except Exception as ex:
            print(f"Warning: failed to parse a blob: {ex}")
            continue
    if not images:
        raise RuntimeError("No depth images parsed from the DB.")
    return images

def make_animation(frames, out_dir, save_pngs=False, fps=4, cmap='viridis', vmin=None, vmax=None):
    os.makedirs(out_dir, exist_ok=True)
    png_dir = os.path.join(out_dir, "frames_png")
    if save_pngs:
        os.makedirs(png_dir, exist_ok=True)

    # computing vmin/vmax if not provided to keep color scale constant
    all_vals = np.array([np.nanmin(f) for f in frames] + [np.nanmax(f) for f in frames])
    if vmin is None:
        vmin = float(np.nanmin([np.nanmin(f) for f in frames]))
    if vmax is None:
        vmax = float(np.nanmax([np.nanmax(f) for f in frames]))

    print(f"Using color scale vmin={vmin:.3f} m, vmax={vmax:.3f} m")

    png_files = []
    for i, frame in enumerate(frames, start=1):
        fig = plt.figure(figsize=(8,6), dpi=100)
        ax = fig.add_subplot(111)
        im = ax.imshow(frame, cmap=cmap, vmin=vmin, vmax=vmax)
        ax.set_title(f"Depth Frame {i}")
        ax.axis('off')
        cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label("Distance (m)")
        
        png_path = os.path.join(out_dir, f"_frame_{i:03d}.png")
        plt.savefig(png_path, bbox_inches='tight', pad_inches=0.01)
        plt.close(fig)
        png_files.append(png_path)
        if save_pngs:
            dest = os.path.join(png_dir, f"frame_{i:03d}.png")
            from shutil import copyfile
            copyfile(png_path, dest)

    # Creating GIF
    gif_path = os.path.join(out_dir, "depth_animation.gif")
    with imageio.get_writer(gif_path, mode='I', duration=1.0/fps) as writer:
        for p in png_files:
            image = imageio.imread(p)
            writer.append_data(image)
    print("Saved GIF:", gif_path)

    # Creating MP4 (imageio uses imageio-ffmpeg)
    mp4_path = os.path.join(out_dir, "depth_animation.mp4")
    with imageio.get_writer(mp4_path, fps=fps, codec='libx264') as writer:
        for p in png_files:
            image = imageio.imread(p)
            writer.append_data(image)
    print("Saved MP4:", mp4_path)

    return gif_path, mp4_path

def main():
    if len(sys.argv) < 3:
        print("Usage: python create_depth_animation.py ./depth.db3 out_dir")
        sys.exit(1)
    db_path = sys.argv[1]
    out_dir = sys.argv[2]
    print("Reading frames from:", db_path)
    frames = read_all_depth_frames(db_path)
    print(f"Read {len(frames)} frames. Generating animation...")
    gif, mp4 = make_animation(frames, out_dir, save_pngs=False, fps=4)
    print("Done. Outputs:", gif, mp4)

if __name__ == "__main__":
    main()
