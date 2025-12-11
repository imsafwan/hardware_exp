import numpy as np
from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import Polygon as ShapelyPolygon
from pyproj import Proj, Transformer
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def discretize_into_AoIs(
        gps_vertices,
        W, H,
        overlap=0.3,
        plot=True):

    # ----------------------------------------------------
    # 1. Convert GPS → UTM
    # ----------------------------------------------------
    lats = [v[0] for v in gps_vertices]
    lons = [v[1] for v in gps_vertices]

    utm_zone = int((lons[0] + 180) / 6) + 1
    proj = Proj(proj='utm', zone=utm_zone, ellps='WGS84')
    transformer_back = Transformer.from_proj(
        proj, Proj('epsg:4326'), always_xy=True)

    utm_vertices = [proj(lon, lat) for lat, lon in gps_vertices]
    poly = Polygon(utm_vertices)

    # ----------------------------------------------------
    # 2. Bounding box
    # ----------------------------------------------------
    minx, miny, maxx, maxy = poly.bounds

    # ----------------------------------------------------
    # 3. Grid spacing
    # ----------------------------------------------------
    dx = W * (1 - overlap)
    dy = H * (1 - overlap)

    # ----------------------------------------------------
    # 4. Generate grid
    # ----------------------------------------------------
    xs = np.arange(minx, maxx + dx, dx)
    ys = np.arange(miny, maxy + dy, dy)

    mission_utm = []

    for cx in xs:
        for cy in ys:

            # Build footprint rectangle polygon
            footprint = ShapelyPolygon([
                (cx - W/2, cy - H/2),
                (cx + W/2, cy - H/2),
                (cx + W/2, cy + H/2),
                (cx - W/2, cy + H/2)
            ])

            # Keep point if ANY part intersects the search polygon
            if poly.intersects(footprint):
                mission_utm.append((cx, cy))

    # ----------------------------------------------------
    # 5. Plotting
    # ----------------------------------------------------
    if plot:
        fig, ax = plt.subplots(figsize=(9, 9))

        # Plot polygon boundary
        poly_x, poly_y = poly.exterior.xy
        ax.plot(poly_x, poly_y, color='grey', linestyle='--', linewidth=2, label='Search Area')

        # Bounding box
        # ax.add_patch(
        #     patches.Rectangle(
        #         (minx, miny), maxx - minx, maxy - miny,
        #         fill=False, edgecolor='gray', linewidth=1.5, linestyle='--'
        #     )
        # )

        # Grid lines
        # for x in xs:
        #     ax.axvline(x, color='lightgray', linewidth=0.5)
        # for y in ys:
        #     ax.axhline(y, color='lightgray', linewidth=0.5)

        # Mission points & footprints
        for (cx, cy) in mission_utm:
            # Draw footprint rectangle
            ax.add_patch(
                patches.Rectangle(
                    (cx - W/2, cy - H/2),
                    W, H,
                    fill=True,
                    edgecolor='blue',
                    linewidth=1,
                    alpha=0.1
                )
            )
            #ax.plot(cx, cy, 'ro', markersize=4)

        
        ax.axis('equal')
        
        

    # ----------------------------------------------------
    # 6. Convert mission points back to GPS if needed
    # ----------------------------------------------------
    mission_gps = []
    for (x, y) in mission_utm:
        lon, lat = transformer_back.transform(x, y)
        mission_gps.append((lat, lon))

    return mission_gps, mission_utm, ax




import yaml

# ---------- Inputs ----------- #

gps_vertices_of_search_area = [
    (41.870090, -87.650356),
    (41.870145, -87.650158),
    (41.869993, -87.650083),
    (41.869908, -87.650308),
]

W = 18   # sensor footprint width in meters
H = 14   # sensor footprint height in meters
overlap = 0.1

# Generate AOIs using discretization
AOIs_gps_points, _, ax = discretize_into_AoIs(
    gps_vertices_of_search_area,
    W, H,
    overlap=overlap,
    plot=True
)

# ---------- Road network inputs ----------- #
# Example: you will fill these with your actual road node GPS coords
gps_vertices_of_rnps = [
    (41.870069, -87.650315),
    (41.870057, -87.650281),
    (41.870044, -87.650249),
    (41.870024, -87.650205),
]






# Create AOI node entries
aoi_nodes = []

for idx, (lat, lon) in enumerate(AOIs_gps_points, start=1):
    aoi_nodes.append({
        "ID": f"AOI_{idx}",
        "type": "air_only",
        "location": {"x": float(lat), "y": float(lon)},
        "time_last_service": 0.0
    })


rnp_nodes = []

for idx, (lat, lon) in enumerate(gps_vertices_of_rnps, start=1):
    rnp_nodes.append({
        "ID": f"rnp{idx}",
        "type": "road",
        "location": {"x": float(lat), "y": float(lon)},
        "time_last_service": 0.0
    })

road_connections = []
for i in range(len(rnp_nodes) - 1):
    road_connections.append({
        "end1": rnp_nodes[i]["ID"],
        "end2": rnp_nodes[i + 1]["ID"]
    })


scene = {
    "ID": "state_uicfield_demo",
    "time": 0.0,
    "agents": [
        {
            "ID": "UAV_01",
            "type": "UAV",
            "subtype": "standard",
            "location": {"x": gps_vertices_of_rnps[0][0], "y": gps_vertices_of_rnps[0][1]},
            "battery_state": {
                "max_battery_energy": 5.0,
                "current_battery_energy": 0.0
            }
        },
        {
            "ID": "UGV_01",
            "type": "UGV",
            "subtype": "standard",
            "location": {"x": gps_vertices_of_rnps[0][0], "y": gps_vertices_of_rnps[0][1]},
        }
    ],
    
    "scenario": {
        "description": "Auto-generated Search Area Discretization",
        "type": "regular_surveillance",
        "subtype": "standard",

        "nodes": (
            aoi_nodes +  # AOI mission points
            rnp_nodes    # road nodes
        ),

        "connections": road_connections
    }
}




# Convert lists into separate arrays for convenience
lat0, lon0 = AOIs_gps_points[0]
utm_zone = int((lon0 + 180) / 6) + 1
proj = Proj(proj='utm', zone=utm_zone, ellps='WGS84')

# -------------------------------------------
# 2. Convert AOIs to UTM
# -------------------------------------------
AOIs_utm = []
for lat, lon in AOIs_gps_points:
    x, y = proj(lon, lat)
    AOIs_utm.append((x, y))

aoi_x = [p[0] for p in AOIs_utm]
aoi_y = [p[1] for p in AOIs_utm]

# -------------------------------------------
# 3. Convert RNPs to UTM
# -------------------------------------------
RNPs_utm = []
for lat, lon in gps_vertices_of_rnps:
    x, y = proj(lon, lat)
    RNPs_utm.append((x, y))

rnp_x = [p[0] for p in RNPs_utm]
rnp_y = [p[1] for p in RNPs_utm]



# AOIs (red)
ax.plot(aoi_x, aoi_y, 'ro', markersize=6, label="AOIs")

# RNPs (blue)
ax.plot(rnp_x, rnp_y, 'bo', markersize=6, label="RNPs")

# Connecting RNP path
ax.plot(rnp_x, rnp_y, 'k-', linewidth=1.5, label="Road Path")

plt.xlabel("UTM X (meters)")
plt.ylabel("UTM Y (meters)")
plt.title("AOIs and Road Nodes in UTM (meters)")
plt.legend()
plt.axis("equal")
plt.show()


import os
import yaml

output_folder = "Inputs"
os.makedirs(output_folder, exist_ok=True)

output_path = os.path.join(output_folder, "scene.yaml")

with open(output_path, "w") as f:
    yaml.safe_dump(scene, f, sort_keys=False)

print(f"Saved YAML to: {output_path}")

