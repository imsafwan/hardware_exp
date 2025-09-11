import math
#import pymap3d as pm





def gps_to_local(origin_lat, origin_lon, target_lat, target_lon):
    """
    Convert GPS coordinates to local ENU coordinates (x, y) in meters
    relative to a given origin.

    Args:
        origin_lat (float): Origin latitude in degrees
        origin_lon (float): Origin longitude in degrees
        target_lat (float): Target latitude in degrees
        target_lon (float): Target longitude in degrees

    Returns:
        (x, y): Tuple of local ENU coordinates in meters
    """
    # Constants
    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(origin_lat))

    # Differences
    dlat = target_lat - origin_lat
    dlon = target_lon - origin_lon

    # Convert to meters
    x = dlon * meters_per_deg_lon   # East
    y = dlat * meters_per_deg_lat   # North

    return (x, y)


# Example usage
origin = (41.870028, -87.650351)

targets = [
    ("AOI_1", 41.87008, -87.6504),
    ("AOI_2", 41.86985, -87.6503),
    ("AOI_3", 41.87005, -87.6501),
    ("AOI_4", 41.87002, -87.65012),
]

for name, lat, lon in targets:
    x, y = gps_to_local(origin[0], origin[1], lat, lon)
    print(f"{name}: x={x:.2f} m, y={y:.2f} m")
