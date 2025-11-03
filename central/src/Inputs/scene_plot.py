import yaml
import matplotlib.pyplot as plt
import numpy as np

# Load YAML data
data = yaml.safe_load(open("scene.yaml"))
nodes = data["scenario"]["nodes"]

# Separate AOIs and RNPs
aoi_x, aoi_y, aoi_labels = [], [], []
rnp_x, rnp_y, rnp_labels = [], [], []

for node in nodes:
    if node["type"] == "air_only":
        aoi_x.append(node["location"]["x"])
        aoi_y.append(node["location"]["y"])
        aoi_labels.append(node["ID"])
    elif node["type"] == "road":
        rnp_x.append(node["location"]["x"])
        rnp_y.append(node["location"]["y"])
        rnp_labels.append(node["ID"])

# Create figure with better styling
fig, ax = plt.subplots(figsize=(12, 10))

# Plot Road Nodes (RNPs) with enhanced styling
rnp_scatter = ax.scatter(rnp_y, rnp_x, color='#2E86DE', label='Road network points (RNPs)', 
                         s=150, alpha=0.8, edgecolors='#1A5490', linewidth=2, 
                         marker='s', zorder=3)

# Plot Air-Only Intersections (AOIs) with enhanced styling
aoi_scatter = ax.scatter(aoi_y, aoi_x, facecolors='none', edgecolors='#EE5A6F', 
                         label='Area of interests (AOIs)', s=250, linewidth=3, 
                         marker='o', zorder=4)

# Add labels with better positioning and styling
for i, label in enumerate(aoi_labels):
    ax.annotate(label, (aoi_y[i], aoi_x[i]), 
               xytext=(10, -10), textcoords='offset points',
               fontsize=10, color='#C23B51', fontweight='bold',
               bbox=dict(boxstyle='round,pad=0.4', facecolor='white', 
                        edgecolor='#EE5A6F', alpha=0.9, linewidth=1.5),
               zorder=5)

for i, label in enumerate(rnp_labels):
    ax.annotate(label, (rnp_y[i], rnp_x[i]), 
               xytext=(10, 10), textcoords='offset points',
               fontsize=9, color='#1A5490', fontweight='bold',
               bbox=dict(boxstyle='round,pad=0.3', facecolor='white', 
                        edgecolor='#2E86DE', alpha=0.9, linewidth=1.5),
               zorder=5)

# Enhanced grid
ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.8, color='gray')
ax.set_axisbelow(True)

# Labels and title
ax.set_xlabel("Longitude", fontsize=13, fontweight='bold')
ax.set_ylabel("Latitude", fontsize=13, fontweight='bold')
ax.set_title("Geospatial Distribution of Nodes\nAir-Only Intersections (AOIs) and Road Network Points (RNPs)", 
            fontsize=15, fontweight='bold', pad=20)

# Enhanced legend
legend = ax.legend(loc='upper right', fontsize=11, framealpha=0.95, 
                  shadow=True, edgecolor='gray', fancybox=True,
                  title='Node Types', title_fontsize=12)
legend.get_title().set_fontweight('bold')

# Add count information
info_text = f"Total Nodes: {len(nodes)}\nAOIs: {len(aoi_labels)} | RNPs: {len(rnp_labels)}"
ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
       fontsize=10, verticalalignment='top', fontweight='bold',
       bbox=dict(boxstyle='round,pad=0.6', facecolor='#F8F9FA', 
                edgecolor='gray', alpha=0.9, linewidth=1.5))

# Set background color
ax.set_facecolor('#FAFAFA')
fig.patch.set_facecolor('white')

# Add padding to limits for better visibility
if aoi_x and rnp_x:
    all_x = aoi_x + rnp_x
    all_y = aoi_y + rnp_y
    x_margin = (max(all_x) - min(all_x)) * 0.1
    y_margin = (max(all_y) - min(all_y)) * 0.1
    ax.set_xlim(min(all_y) - y_margin, max(all_y) + y_margin)
    ax.set_ylim(min(all_x) - x_margin, max(all_x) + x_margin)

# Equal aspect ratio for proper geographic representation
ax.set_aspect('equal', adjustable='box')

plt.tight_layout()
plt.show()