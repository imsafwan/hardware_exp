import yaml
import matplotlib.pyplot as plt
import numpy as np
import math

# ===============================================================
# 1. Load YAML and Define Lat/Lon → Local Meters Conversion
# ===============================================================
def latlon_to_local_m(lat, lon, ref_lat, ref_lon):
    """Convert latitude/longitude to local Cartesian (meters)."""
    R = 6371000  # Earth radius in meters
    dx = math.radians(lon - ref_lon) * R * math.cos(math.radians(ref_lat))
    dy = math.radians(lat - ref_lat) * R
    return dx, dy  # East (x), North (y)

# Load YAML data
data = yaml.safe_load(open("scene.yaml"))
nodes = data["scenario"]["nodes"]

# Reference point (first node)
ref_lat = nodes[0]["location"]["x"]
ref_lon = nodes[0]["location"]["y"]

# ===============================================================
# 2. Separate AOIs and RNPs (Convert to meters)
# ===============================================================
aoi_x, aoi_y, aoi_labels = [], [], []
rnp_x, rnp_y, rnp_labels = [], [], []

for node in nodes:
    lat = node["location"]["x"]
    lon = node["location"]["y"]
    east, north = latlon_to_local_m(lat, lon, ref_lat, ref_lon)
    if node["type"] == "air_only":
        aoi_x.append(north)
        aoi_y.append(east)
        aoi_labels.append(node["ID"])
    elif node["type"] == "road":
        rnp_x.append(north)
        rnp_y.append(east)
        rnp_labels.append(node["ID"])

# ===============================================================
# 3. Plot setup
# ===============================================================
fig, ax = plt.subplots(figsize=(12, 10))

# --- Road Nodes (RNPs)
rnp_scatter = ax.scatter(
    rnp_y, rnp_x,
    color='#2E86DE', label='Road network points (RNPs)',
    s=150, alpha=0.8, edgecolors='#1A5490', linewidth=2,
    marker='s', zorder=3
)

# --- Air-Only Intersections (AOIs)
aoi_scatter = ax.scatter(
    aoi_y, aoi_x,
    facecolors='none', edgecolors='#EE5A6F',
    label='Area of interests (AOIs)',
    s=250, linewidth=3, marker='o', zorder=4
)

# ===============================================================
# 4. Labels with better styling (rasterized later)
# ===============================================================
for i, label in enumerate(aoi_labels):
    ax.annotate(
        label, (aoi_y[i], aoi_x[i]),
        xytext=(10, -10), textcoords='offset points',
        fontsize=15, color='#C23B51', fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.4', facecolor='white',
                  edgecolor='#EE5A6F', alpha=0.9, linewidth=1.5),
        zorder=5
    )

for i, label in enumerate(rnp_labels):
    ax.annotate(
        label, (rnp_y[i], rnp_x[i]),
        xytext=(10, 10), textcoords='offset points',
        fontsize=15, color='#1A5490', fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                  edgecolor='#2E86DE', alpha=0.9, linewidth=1.5),
        zorder=5
    )

# Rasterize annotation boxes to reduce Illustrator load
# for ann in ax.texts:
#     ann.set_rasterized(True)

# ===============================================================
# 5. Styling and Info Box
# ===============================================================
ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.8, color='gray')
ax.set_axisbelow(True)

ax.set_xlabel("Easting (m)", fontsize=13, fontweight='bold')
ax.set_ylabel("Northing (m)", fontsize=13, fontweight='bold')
ax.set_title("Geospatial Distribution of Nodes (in meters)\n"
             "Air-Only Intersections (AOIs) and Road Network Points (RNPs)",
             fontsize=15, fontweight='bold', pad=20)

# Legend
legend = ax.legend(
    loc='upper right', fontsize=11, framealpha=0.95,
    shadow=True, edgecolor='gray', fancybox=True,
    title='Node Types', title_fontsize=12
)
legend.get_title().set_fontweight('bold')

# Info box
info_text = f"Total Nodes: {len(nodes)}\nAOIs: {len(aoi_labels)} | RNPs: {len(rnp_labels)}"
ax.text(
    0.02, 0.98, info_text, transform=ax.transAxes,
    fontsize=10, verticalalignment='top', fontweight='bold',
    bbox=dict(boxstyle='round,pad=0.6', facecolor='#F8F9FA',
              edgecolor='gray', alpha=0.9, linewidth=1.5)
)

# ===============================================================
# 6. Layout, Background, and Limits
# ===============================================================
ax.set_facecolor('#FAFAFA')
fig.patch.set_facecolor('white')

if aoi_x and rnp_x:
    all_x = aoi_x + rnp_x
    all_y = aoi_y + rnp_y
    x_margin = (max(all_x) - min(all_x)) * 0.1
    y_margin = (max(all_y) - min(all_y)) * 0.1
    ax.set_xlim(min(all_y) - y_margin, max(all_y) + y_margin)
    ax.set_ylim(min(all_x) - x_margin, max(all_x) + x_margin)

ax.set_aspect('equal', adjustable='box')

# ===============================================================
# 7. Save and Show
# ===============================================================
plt.tight_layout()
plt.savefig("scene_map_meters.pdf", dpi=300)
plt.show()
