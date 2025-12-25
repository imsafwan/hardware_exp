import yaml
import numpy as np
import torch
from pyproj import Proj
import matplotlib.pyplot as plt


# ============================================================
# 1️⃣ GPS → Local XY conversion (relative to fixed reference)
# ============================================================
def gps_to_local_xy(lat, lon, ref_lat, ref_lon):
    """
    Convert (lat, lon) to local (x, y) in meters using Transverse Mercator projection.
    The reference GPS defines origin (0, 0).
    """
    proj_local = Proj(proj='tmerc', lat_0=ref_lat, lon_0=ref_lon, ellps='WGS84')
    x, y = proj_local(lon, lat)
    ref_x, ref_y = proj_local(ref_lon, ref_lat)
    return x - ref_x, y - ref_y


# ============================================================
# 2️⃣ Parse YAML and extract node information
# ============================================================
def load_scene_yaml(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    nodes = data["scenario"]["nodes"]
    connections = data["scenario"].get("connections", [])


    uav_location = (data['agents'][0]['location']['x'], data['agents'][0]['location']['y'])
    

    ugv_points, uav_points = [], []
    for node in nodes:
        lat, lon = node["location"]["x"], node["location"]["y"]
        if node["type"] == "road":
            ugv_points.append((lat, lon))
        elif node["type"] == "air_only" and node['time_last_service'] == 0:
            uav_points.append((lat, lon))

    depot = ugv_points.index(uav_location) if uav_location in ugv_points else None

    return np.array(ugv_points, dtype=np.float32), np.array(uav_points, dtype=np.float32), connections, depot


# ============================================================
# 3️⃣ Convert all GPS → local coordinates (relative to fixed ref)
# ============================================================
def convert_scene_to_xy(ugv_gps, uav_gps, ref_lat, ref_lon):
    ugv_xy = np.array([gps_to_local_xy(lat, lon, ref_lat, ref_lon) for lat, lon in ugv_gps], dtype=np.float32)
    uav_xy = np.array([gps_to_local_xy(lat, lon, ref_lat, ref_lon) for lat, lon in uav_gps], dtype=np.float32)
    return ugv_xy, uav_xy


# ============================================================
# 4️⃣ Build Shortest Path (SP) Matrix from connections
# ============================================================
def build_sp_matrix_from_connections(ugv_xy, connections, device="cuda"):
    N = len(ugv_xy)
    dist = np.full((N, N), np.inf, dtype=np.float32)
    np.fill_diagonal(dist, 0)

    for conn in connections:
        i = int(conn["end1"].replace("rnp", "")) - 1
        j = int(conn["end2"].replace("rnp", "")) - 1
        d = np.linalg.norm(ugv_xy[i] - ugv_xy[j])
        dist[i, j] = dist[j, i] = d

    sp_matrix = torch.from_numpy(dist).to(device)
    for k in range(N):
        sp_matrix = torch.minimum(sp_matrix, sp_matrix[:, k:k+1] + sp_matrix[k:k+1, :])
    return sp_matrix


# ============================================================
# 5️⃣ Visualization function
# ============================================================
def visualize_scene(ugv_xy, uav_xy, connections=None, ref_lat=None, ref_lon=None):
    """
    Visualize UGV road network and UAV AOIs in local coordinates.
    """
    plt.figure(figsize=(7, 7))
    plt.title("Scene Layout (Local XY Coordinates)")

    # Plot UGV points and connections
    if connections:
        for conn in connections:
            i = int(conn["end1"].replace("rnp", "")) - 1
            j = int(conn["end2"].replace("rnp", "")) - 1
            x_coords = [ugv_xy[i, 0], ugv_xy[j, 0]]
            y_coords = [ugv_xy[i, 1], ugv_xy[j, 1]]
            plt.plot(x_coords, y_coords, 'b-', alpha=0.6)

    plt.scatter(ugv_xy[:, 0], ugv_xy[:, 1], c='blue', s=80, marker='o', label='UGV Nodes (Road)')
    plt.scatter(uav_xy[:, 0], uav_xy[:, 1], c='red', s=80, marker='^', label='UAV Nodes (AOI)')

    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.axis('equal')

    if ref_lat and ref_lon:
        plt.suptitle(f"Reference GPS: ({ref_lat:.6f}, {ref_lon:.6f}) → Origin (0,0)", fontsize=9)

    plt.tight_layout()
    plt.show()


# ============================================================
# 6️⃣ Main scene-to-dataset function
# ============================================================
def scene_to_dataset(scene_path, ref_lat, ref_lon, device="cuda", visualize=True):
    """
    Converts a GPS-based scene.yaml to tensors usable by your dataset generator.
    ref_lat/lon define the global (0,0) origin.
    """
    ugv_gps, uav_gps, connections, depot_ix = load_scene_yaml(scene_path)
    
    ugv_xy, uav_xy = convert_scene_to_xy(ugv_gps, uav_gps, ref_lat, ref_lon)

    sp_matrix = build_sp_matrix_from_connections(ugv_xy, connections, device=device)

    ugv_loc_t = torch.from_numpy(ugv_xy).to(device)
    uav_loc_t = torch.from_numpy(uav_xy).to(device)

    print(f"Loaded scene: {scene_path}")
    print(f"UGV nodes: {ugv_loc_t.shape}, UAV nodes: {uav_loc_t.shape}")
    print(f"Reference GPS: ({ref_lat:.6f}, {ref_lon:.6f}) → origin (0, 0)")

    if visualize:
        visualize_scene(ugv_xy, uav_xy, connections, ref_lat, ref_lon)

    return ugv_loc_t, uav_loc_t, sp_matrix, depot_ix





# ============================================================
# 7️⃣ Example usage
# ============================================================
# if __name__ == "__main__":
#     # Fixed GPS origin (field reference)
#     REF_LAT = 41.869884
#     REF_LON = -87.650660

#     ugv_loc, uav_loc, sp_matrix = scene_to_dataset(
#         "scene.yaml",
#         REF_LAT,
#         REF_LON,
#         device="cuda",
#         visualize=True
#     )

#     print("\nUGV XY coordinates (meters):")
#     print(ugv_loc)

#     print("\nUAV XY coordinates (meters):")
#     print(uav_loc)

#     print("\nShortest-path matrix (partial):")
#     print(sp_matrix)