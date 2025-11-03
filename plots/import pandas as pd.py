import pandas as pd
import matplotlib.pyplot as plt
import yaml

# === Load YAML ===
with open('state_uicfield_demo.yaml', 'r') as f:
    data = yaml.safe_load(f)

# === Extract AOI and RNP positions ===
aoi_positions = [
    (node['location']['x'], node['location']['y'])
    for node in data['scenario']['nodes'] if node['type'] == 'air_only'
]
rnp_positions = [
    (node['location']['x'], node['location']['y'])
    for node in data['scenario']['nodes'] if node['type'] == 'road'
]

# === Plot background map (AOI + RNP) ===
aoi_x, aoi_y = zip(*aoi_positions)
rnp_x, rnp_y = zip(*rnp_positions)

plt.figure(figsize=(8, 7))
plt.scatter(rnp_y, rnp_x, c='gray', label='Road Nodes (RNP)', s=40)
plt.scatter(aoi_y, aoi_x, c='red', label='AOI Points', s=40)
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('AOI and Road Node Positions')
plt.legend()
plt.grid(True)

# === Read all CSVs and combine ===
csvs = ['csv.1', 'csv.2', 'csv.3', 'csv.4']
all_data = []

for csv_file in csvs:
    df = pd.read_csv(csv_file)
    if not {'latitude_deg', 'longitude_deg', 'time'}.issubset(df.columns):
        print(f"Skipping {csv_file}: missing columns")
        continue
    all_data.append(df[['time', 'latitude_deg', 'longitude_deg']])

# Combine all data into one DataFrame
if all_data:
    combined_df = pd.concat(all_data, ignore_index=True)
    combined_df['time'] = combined_df['time'] - combined_df['time'].iloc[0]
else:
    raise ValueError("No valid CSVs found!")

# === Plot UAV trajectory ===
plt.scatter(
    combined_df['longitude_deg'],
    combined_df['latitude_deg'],
    c=combined_df['time'],
    cmap='viridis',
    s=30,
    label='UAV Path'
)
cbar = plt.colorbar(label='Time (s)')
plt.legend()
plt.show()

# === Optional: save combined UAV positions ===
uav_positions = list(zip(
    combined_df['latitude_deg'],
    combined_df['longitude_deg'],
    combined_df['time']
))
print("Total UAV points:", len(uav_positions))
print("First few UAV positions (lat, lon, t):", uav_positions[:5])