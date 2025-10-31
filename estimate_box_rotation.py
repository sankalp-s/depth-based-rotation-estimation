import sys, sqlite3, numpy as np, math, csv
from scipy.spatial import ConvexHull

def read_images_from_rosbag_sqlite(db_path):
    """
    Reads depth images from a ROS2 SQLite bag (.db3) file.
    Each message in the 'messages' table contains a serialized ROS Image (as a BLOB).
    This function extracts those BLOBs, parses them into NumPy arrays,
    and converts depth values into meters.
    """
    conn = sqlite3.connect(db_path)  # Open a connection to the SQLite database
    cur = conn.cursor() 
    cur.execute("SELECT data FROM messages ORDER BY id;")
    rows = cur.fetchall()  # Read all query results into a list of tuples
    conn.close()
    images = []  # Initialize a list to store decoded depth images
    for (blob,) in rows: 
        """ 
        Looping through each message (each row is one frame)
        parse height and width from positions observed in dataset
        Extract image height and width (found at fixed byte offsets)
        The positions 44:48 and 48:52 come from how ROS serializes sensor_msgs/Image
        """
        height = int.from_bytes(blob[44:48], 'little')
        width = int.from_bytes(blob[48:52], 'little')

        # Computing where the actual pixel data starts ("data" section)
        # The rest of the blob before that is metadata (header)
        header_len = len(blob) - (height * width * 2)
        data = blob[header_len:header_len + height * width * 2]

        # Convert binary pixel data into a 2D NumPy array (dtype=np.uint16 → 16-bit pixels, reshape → [height, width])
        img = np.frombuffer(data, dtype=np.uint16).reshape((height, width))
        images.append(img.astype(np.float32) / 1000.0) #mm to meters
    return images

def depth_to_pointcloud(depth, fx=525.0, fy=525.0, cx=None, cy=None):
    """
    Converting a 2D depth image into a 3D point cloud using camera intrinsics.

    Args:
    depth (np.ndarray): Depth map in meters.
    fx, fy (float): Focal lengths (pixels).
    cx, cy (float, optional): Principal point (defaults to image center).

    Returns:
    np.ndarray: (N, 3) array of 3D points (X, Y, Z) in camera coordinates.
    """

    H, W = depth.shape
    if cx is None: cx = W / 2.0
    if cy is None: cy = H / 2.0

    # Creating pixel coordinate grids (i, j)
    i, j = np.indices(depth.shape)
    z = depth

    # ----------------------------------------------------------
    # Computing real-world X and Y coordinates
    #  Using pinhole camera model:
    #     X = (u - cx) * Z / fx
    #     Y = (v - cy) * Z / fy
    #  where (u, v) are pixel coordinates, and Z is depth.
    # ----------------------------------------------------------
    x = (j - cx) * z / fx
    y = (i - cy) * z / fy

    # Combining X, Y, Z into a single 3D array, reshaing it to (N, 3), and filtering invalid points
    pts = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    mask = np.isfinite(pts[:, 2]) & (pts[:, 2] > 0)
    return pts[mask]

def ransac_plane(points, iterations=1500, dist_thresh=0.01):

    """
    Fiting a 3D plane to a noisy set of points using the RANSAC algorithm.

    Args:
    points (np.ndarray): Nx3 array of 3D points.
    iterations (int): Number of random trials (default: 1500).
    dist_thresh (float): Max distance (m) to consider a point an inlier.

    Returns:
    tuple: (best_plane, best_inliers)
    best_plane (tuple): (normal_vector, d) of the best-fit plane.
    best_inliers (np.ndarray): Indices of points close to the plane.
    """

    best_inliers = None
    best_plane = None
    n = points.shape[0]
    if n < 3:  # If we have fewer than 3 points, we can’t define a plane
        return None, None
    
    # Main RANSAC loop
    for _ in range(iterations):
                
        # plane uniquely defined by 3 non-collinear points selected randomly
        ids = np.random.choice(n, 3, replace=False)
        p1, p2, p3 = points[ids]

        v1 = p2 - p1; v2 = p3 - p1
        normal = np.cross(v1, v2) # normal vector = v1 × v2 (perpendicular to plane)

        # Normalize the normal vector (unit length)
        norm = np.linalg.norm(normal)
        if norm == 0: continue
        normal = normal / norm 

        # Compute plane equation: ax + by + cz + d = 0
        # Using the point-normal form: n·p + d = 0 -> d = -n·p
        d = -np.dot(normal, p1)

        # Computes distance of all points to this plane
        # dist = |(n·p + d)| / ||n||
        dist = np.abs(np.dot(points, normal) + d)

        # Identifying inliers — points close to the plane
        # Points whose perpendicular distance < dist_thresh (1 cm default)
        inliers = np.where(dist < dist_thresh)[0]

        if best_inliers is None or len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_plane = (normal, d)

    return best_plane, best_inliers # Returns the best-fit plane and its inliers

def process_images(images, fx=525.0, fy=525.0):
    """
    Processes all depth images to detect the dominant planar face (e.g., box surface)
    in each frame, calculate its normal vector, visible area, and the angle with
    respect to the camera. Finally, estimates the overall rotation axis of the box.

    Args:
    images (list[np.ndarray]): List of depth frames (2D arrays in meters)
    fx, fy (float): Camera focal lengths in pixels (intrinsics)

    Returns:
    tuple:
        results (list): [(frame_index, normal_vector, angle_deg, area_m2), ...]
        rotation_axis (np.ndarray): Estimated rotation axis (unit vector)
    """
    
    results = [] 
    normals = [] 
    H, W = images[0].shape 
    cx, cy = W / 2.0, H / 2.0 

    # Process each depth image frame
    for idx, depth in enumerate(images, start=1): 
        # Converting depth map → 3D point cloud in camera coordinates
        pts = depth_to_pointcloud(depth, fx, fy, cx, cy)

        # Using RANSAC to detect the largest planar surface in the point cloud
        plane, inliers_idx = ransac_plane(pts)

        if plane is None or len(inliers_idx) < 50:
            print(f"Frame {idx}: plane fit failed or too few inliers")
            continue

        normal, d = plane

        # Ensuring that normal faces camera negative z (flip if needed)
        if normal[2] > 0:
            normal = -normal; d = -d
        normals.append(normal)

        # Extract inlier points for area computation
        inlier_pts = pts[inliers_idx] 
        origin = inlier_pts.mean(axis=0) # Centroid

        # Compute local 2D coordinate axes (u,v) on the plane surface
        u = np.cross(normal, np.array([0,0,1.0]))
        if np.linalg.norm(u) < 1e-6:
            u = np.array([1.0, 0, 0]) 
        u = u / np.linalg.norm(u)
        v = np.cross(normal, u); v = v / np.linalg.norm(v)
        
        # Projecting 3D inlier points onto the (u,v) plane
        coords2d = np.dot(inlier_pts - origin, np.stack([u, v], axis=1))

        # Estimating visible planar area (Convex Hull)
        try:
            hull = ConvexHull(coords2d)
            area = hull.volume if not hasattr(hull, 'area') else hull.area
        except Exception:
            # Fallback method — use bounding box if hull fails
            mins = coords2d.min(axis=0); maxs = coords2d.max(axis=0)
            area = float(np.prod(maxs - mins))
        
        # Step 6 — Compute angle between plane normal and camera axis
        cam_norm = np.array([0,0,1.0])

        # Dot product gives cosine of angle between normals
        angle_rad = math.acos(max(-1.0, min(1.0, np.dot(normal, cam_norm) / (np.linalg.norm(normal) * np.linalg.norm(cam_norm)))))
        angle_deg = math.degrees(angle_rad)
        results.append((idx, normal, angle_deg, area))
    # Estimating rotation axis as normalized mean of normals

    # Compute mean normal vector (average direction of all plane normals)
    mean_normal = np.mean(np.stack(normals), axis=0)
    rotation_axis = mean_normal / np.linalg.norm(mean_normal)

    return results, rotation_axis

if __name__ == '__main__':
    import os

    # For that this script works using the command prompts
    if len(sys.argv) < 3:
        print('Usage: python3 estimate_box_rotation.py /path/to/depth.db3 out_dir')
        sys.exit(1)

    
    db = sys.argv[1]; out_dir = sys.argv[2]; os.makedirs(out_dir, exist_ok=True)
    images = read_images_from_rosbag_sqlite(db) # Reads depth frames from SQLite DB as NumPy arrays
    results, axis = process_images(images) 

    csv_path = os.path.join(out_dir, 'results.csv')

    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['frame','nx','ny','nz','angle_deg','visible_area_m2'])
        for idx, normal, angle, area in results:
            w.writerow([idx, normal[0], normal[1], normal[2], angle, area])
    with open(os.path.join(out_dir, 'rotation_axis.txt'), 'w') as f:
        f.write(f"{axis[0]} {axis[1]} {axis[2]}\n")
    print('Outputs saved to', out_dir)
