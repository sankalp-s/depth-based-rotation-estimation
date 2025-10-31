import sqlite3
import numpy as np
import matplotlib.pyplot as plt

# Path to ROS2 SQLite bag file
DB_PATH = "./depth.db3"

conn = sqlite3.connect(DB_PATH)
cur = conn.cursor()

# Fetch all message blobs in time order
cur.execute("SELECT data FROM messages ORDER BY id;")
rows = cur.fetchall()
conn.close()

print(f"Total frames found: {len(rows)}")

# Loop through all frames and display them
for i, (blob,) in enumerate(rows, start=1):
    try:
        height = int.from_bytes(blob[44:48], "little")
        width = int.from_bytes(blob[48:52], "little")
        header_len = len(blob) - (height * width * 2)
        img_data = blob[header_len : header_len + height * width * 2]

        # --- Convert to numpy depth image ---
        depth_mm = np.frombuffer(img_data, dtype=np.uint16).reshape((height, width))
        depth_m = depth_mm.astype(np.float32) / 1000.0  # convert mm → m

        print(f"Frame {i}: shape={depth_m.shape}, min={np.min(depth_m):.3f}m, "
              f"max={np.max(depth_m):.3f}m, median={np.median(depth_m):.3f}m")

        # Display using matplotlib
        plt.figure(figsize=(8, 6))
        plt.imshow(depth_m, cmap="viridis")
        plt.title(f"Depth Frame {i} (in meters)")
        plt.colorbar(label="Distance (m)")
        plt.axis("off")
        plt.show()

    except Exception as e:
        print(f"⚠️ Error parsing frame {i}: {e}")
