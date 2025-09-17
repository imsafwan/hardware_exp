import yaml
import math

# -------------------------------
# Origin of your Gazebo world
lat0 = 41.870028
lon0 = -87.650351

# conversion factors
m_per_deg_lat = 111132.0
m_per_deg_lon = 111132.0 * math.cos(math.radians(lat0))

def latlon_to_local(lat, lon):
    dlat = lat - lat0
    dlon = lon - lon0
    x = dlon * m_per_deg_lon   # east
    y = dlat * m_per_deg_lat   # north
    return (x, y)

# -------------------------------
# Load your YAML file
with open("Inputs/scene.yaml", "r") as f:
    data = yaml.safe_load(f)

print(f"Origin lat={lat0}, lon={lon0}")
print("Local coordinates relative to origin (meters):\n")

for i, node in enumerate(data["scenario"]["nodes"], start=1):
    lat = node["location"]["x"]
    lon = node["location"]["y"]
    x, y = latlon_to_local(lat, lon)
    if node['type'] == 'air_only':

        # XML snippet with proper formatting
        s = f"""<include>
<uri>https://fuel.gazebosim.org/1.0/openrobotics/models/Construction%20Cone</uri>
<name>construction_cone_{i}</name>
<pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
</include>"""

        #print(f"{node['ID']:6s}  ->  East: {x:7.2f} m , North: {y:7.2f} m")
        print(s)
        print()
