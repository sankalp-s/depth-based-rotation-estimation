# Depth-Based Rotation Estimation

A perception engineering pipeline that processes **depth images** stored in a ROS2 SQLite bag (`depth.db3`), converts them into 3D point clouds, detects dominant planar surfaces using **RANSAC**, and estimates the **object’s rotation axis** across multiple frames.

---

## 🧠 Overview

This pipeline performs end-to-end depth data analysis using computer vision and geometry:

1. **Read Depth Frames** — Extract serialized ROS `sensor_msgs/Image` messages from a `.db3` bag file.
2. **Convert Depth to 3D Points** — Use camera intrinsics to compute `(X, Y, Z)` coordinates for each pixel.
3. **Plane Detection (RANSAC)** — Fit planes to noisy 3D data and isolate the dominant surface.
4. **Feature Computation** — Estimate visible area and angle between the plane normal and camera.
5. **Rotation Axis Estimation** — Aggregate normals across frames to find the object’s 3D rotation axis.
6. **Export Results** — Save per-frame data to `results.csv` and the final axis to `rotation_axis.txt`.

---

## 🧩 Pipeline Flow

```
Depth Image (.db3)
      ↓
Parse ROS Messages (BLOB → NumPy array)
      ↓
Convert to Point Cloud (fx, fy, cx, cy)
      ↓
RANSAC Plane Detection
      ↓
Plane Area + Orientation Estimation
      ↓
Compute Mean Normal (Rotation Axis)
      ↓
Save Results (CSV + TXT)
```

📊 _A visual diagram of this process can be found in:_  
[`assets/clean_depth_processing_flowchart.png`](./assets/clean_depth_processing_flowchart.png)

---

## 🛠️ Features

- ✅ Parse ROS2 SQLite bag files (`.db3`)
- ✅ Convert depth maps → 3D point clouds
- ✅ Fit planes robustly using **RANSAC**
- ✅ Compute surface normals, visible area, and rotation angles
- ✅ Estimate overall **rotation axis**
- ✅ Generate plots, GIFs, and animations (optional utilities)
- ✅ Outputs results in standard formats for downstream analysis

---

## 📂 Project Structure

```
depth-based-rotation-estimation/
│
├── estimate_box_rotation.py          # Main pipeline script
├── create_depth_animation_with_inliers.py  # Visualization and GIF creation
├── requirements.txt                  # Dependencies
├── metadata.yaml                     # ROS2 metadata
├── depth.db3                         # Example input data (ROS bag)
├── results.csv                       # Per-frame analysis output
├── rotation_axis.txt                 # Estimated rotation axis vector
├── frames/                           # (Optional) Depth frame visualizations
├── assets/
│   └── clean_depth_processing_flowchart.png  # Pipeline flow diagram
├── LICENSE
└── README.md
```

---

## ⚙️ Installation

### 1️⃣ Clone the Repository
```bash
git clone https://github.com/sankalp-s/depth-based-rotation-estimation.git
cd depth-based-rotation-estimation
```

### 2️⃣ Set Up Virtual Environment
```bash
python3 -m venv venv
source venv/bin/activate
```

### 3️⃣ Install Dependencies
```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

---

## ▶️ Usage

### Basic Command
```bash
python estimate_box_rotation.py ./depth.db3 ./out_dir
```

### Arguments
| Argument | Description |
|-----------|--------------|
| `depth.db3` | Input ROS2 SQLite bag file containing depth messages |
| `out_dir` | Directory to save output files (`results.csv`, `rotation_axis.txt`) |

### Example Output
```
Done. Outputs saved to ./out_dir
```

---

## 📄 Output Files

| File | Description |
|------|--------------|
| `results.csv` | Per-frame results including plane normal, angle, and visible area |
| `rotation_axis.txt` | Final estimated rotation axis (unit vector) |
| `depth_animation_with_inliers.gif` | Optional visualization (if generated) |

**Example (results.csv):**
| frame | nx | ny | nz | angle_deg | visible_area_m2 |
|-------|----|----|----|------------|----------------|
| 1 | 0.03 | -0.87 | -0.49 | 120.5 | 5.14 |
| 2 | 0.02 | -0.45 | -0.89 | 115.7 | 5.08 |

---

## 🧮 Core Algorithms

### 🔹 Depth to Point Cloud
Converts each pixel `(i, j, z)` into a 3D coordinate `(X, Y, Z)` using the **pinhole camera model**:
\[
X = (j - c_x) \cdot Z / f_x, \quad
Y = (i - c_y) \cdot Z / f_y, \quad
Z = Z
\]

### 🔹 Plane Fitting (RANSAC)
Repeatedly samples 3 random points, computes the plane normal via cross product, and selects the plane with the **most inliers** (within a distance threshold).

### 🔹 Rotation Axis
The **mean of all detected plane normals**, normalized to a unit vector, represents the estimated **rotation axis** of the object.

---

## 🖼️ Visualization (Optional)

To generate animations or GIFs of the depth frames:
```bash
python create_depth_animation_with_inliers.py ./depth.db3 ./out_dir
```

Outputs:
- `depth_animation_with_inliers.gif`
- `depth_animation_with_inliers.mp4`

---

## 🧾 License

This project is licensed under the **MIT License** — see the [LICENSE](./LICENSE) file for details.

---

## 👤 Author

**Sankalp Arora**  
Perception Engineer — 10xConstruction Apprentice Assignment  
📧 sankalp.arora@example.com  
🌐 [LinkedIn](https://www.linkedin.com/in/sankalp-arora/)

---

## 💡 Acknowledgements

- ROS2 bag format (`.db3`) handling inspired by `rosbag2_py` message serialization.  
- RANSAC implementation adapted from standard 3D geometry methods.  
- Visualization tools built using **Matplotlib** and **ImageIO**.

---

## 🧠 Future Enhancements
- Integrate real camera intrinsics via `camera_info` topic parsing.  
- Add interactive 3D visualization (e.g., Open3D, Plotly).  
- Extend to detect multiple planes and object pose changes over time.
