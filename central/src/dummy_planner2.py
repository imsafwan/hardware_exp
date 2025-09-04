import networkx as nx
import math

# Parameters
UAV_SPEED = 0.67  # m/s
UGV_SPEED = 0.9   # m/s
TAKEOFF_TIME = 17  # seconds
LANDING_TIME = 25  # seconds

def haversine_distance(coord1, coord2):
    """Calculate distance between two GPS coordinates in meters."""
    R = 6371000  # Earth radius in meters
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = (math.sin(dlat/2)**2 + 
         math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

# Extended Road Network (where UGV can travel and UAV can land)
road_network = nx.Graph()

# Road network nodes (UAV MUST land at these points)
road_network.add_node("A", lat=41.870046, lon=-87.650351)  # Start position
road_network.add_node("B", lat=41.870007, lon=-87.650315)  # Road intersection 1
road_network.add_node("C", lat=41.869950, lon=-87.650280)  # Road intersection 2
road_network.add_node("D", lat=41.869890, lon=-87.650240)  # Road intersection 3
road_network.add_node("E", lat=41.869960, lon=-87.650180)  # Road intersection 4
road_network.add_node("F", lat=41.870020, lon=-87.650120)  # Road intersection 5

# Calculate distances and add edges
edges = [
    ("A", "B"),
    ("B", "C"), 
    ("C", "D"),
    ("D", "E"),
    ("E", "F"),
    ("B", "E"),  # Shortcut road
    ("A", "C"),  # Alternative route
    ("C", "F")   # Another route option
]

for start, end in edges:
    start_coord = (road_network.nodes[start]["lat"], road_network.nodes[start]["lon"])
    end_coord = (road_network.nodes[end]["lat"], road_network.nodes[end]["lon"])
    distance = haversine_distance(start_coord, end_coord)
    road_network.add_edge(start, end, weight=distance)

# UAV Survey Points (UAV can visit these but cannot land here)
survey_points = {
    "TARGET_1": (41.870080, -87.650400),  # Building surveillance
    "TARGET_2": (41.869920, -87.650350),  # Intersection monitoring  
    "TARGET_3": (41.870100, -87.650200),  # Park area survey
    "TARGET_4": (41.869850, -87.650300),  # Industrial area check
    "TARGET_5": (41.870050, -87.650100),  # Residential survey
    "RECON_1": (41.870150, -87.650250),  # Reconnaissance point 1
    "RECON_2": (41.869800, -87.650180),  # Reconnaissance point 2
}

print("=== ROAD NETWORK (Landing Points) ===")
for node, data in road_network.nodes(data=True):
    print(f"Node {node}: ({data['lat']:.6f}, {data['lon']:.6f})")

print("\n=== SURVEY POINTS (UAV Visit Only) ===")
for name, coord in survey_points.items():
    print(f"{name}: ({coord[0]:.6f}, {coord[1]:.6f})")

print("\n=== ROAD DISTANCES ===")
for edge in road_network.edges(data=True):
    print(f"{edge[0]} -> {edge[1]}: {edge[2]['weight']:.1f}m")

# =============================================================================
# MISSION 1: Survey then return A -> TARGET_1 -> TARGET_2 -> B (land)
# =============================================================================
print("\n" + "="*60)
print("MISSION 1: Survey Mission A -> TARGET_1 -> TARGET_2 -> B (land)")
print("="*60)

# Mission 1 waypoints
start_point = (41.870046, -87.650351)  # Node A (takeoff)
target_1 = survey_points["TARGET_1"]   # Survey point 1
target_2 = survey_points["TARGET_2"]   # Survey point 2  
landing_point = (41.870007, -87.650315)  # Node B (land on UGV)

# Calculate UAV flight times
current_time = 0
uav_waypoints_1 = [start_point, target_1, target_2, landing_point]
uav_times_1 = [current_time]

# Takeoff
current_time += TAKEOFF_TIME
uav_times_1.append(current_time)

# Flight segments
for i in range(len(uav_waypoints_1) - 1):
    dist = haversine_distance(uav_waypoints_1[i], uav_waypoints_1[i + 1])
    flight_time = dist / UAV_SPEED
    current_time += flight_time
    uav_times_1.append(current_time)
    print(f"UAV segment {i+1}: {dist:.1f}m, {flight_time:.1f}s")

# Landing
current_time += LANDING_TIME
uav_times_1.append(current_time)

# UGV route: A -> B (via road network)
ugv_dist_1 = road_network.edges[("A", "B")]["weight"]
ugv_travel_time_1 = ugv_dist_1 / UGV_SPEED
ugv_arrival_time_1 = max(ugv_travel_time_1, uav_times_1[-2])  # Arrive before UAV lands

print(f"UGV route A->B: {ugv_dist_1:.1f}m, {ugv_travel_time_1:.1f}s")
print(f"Mission duration: {current_time:.1f}s")

plan_uav_1 = {
    "msg_type": "UAV_PLAN",
    "plan_id": 1,
    "new_plan": [
        {
            "type": "takeoff_from_UGV", 
            "ins_id": "take_off_1", 
            "end_time": uav_times_1[1]
        },
        {
            "type": "move_to_location", 
            "ins_id": "survey_1",
            "location": {"lat": target_1[0], "lon": target_1[1]}, 
            "end_time": uav_times_1[2]
        },
        {
            "type": "move_to_location",
            "ins_id": "survey_2", 
            "location": {"lat": target_2[0], "lon": target_2[1]},
            "end_time": uav_times_1[3]
        },
        {
            "type": "move_to_location",
            "ins_id": "return_to_ugv",
            "location": {"lat": landing_point[0], "lon": landing_point[1]},
            "end_time": uav_times_1[4]
        },
        {
            "type": "land_on_UGV", 
            "ins_id": "land_1", 
            "end_time": uav_times_1[5]
        }
    ]
}

plan_ugv_1 = {
    "msg_type": "UGV_PLAN",
    "plan_id": 1,
    "new_plan": [
        {
            "type": "allow_take_off_from_UGV",
            "location": {"lat": start_point[0], "lon": start_point[1]},
            "ins_id": "take_off_1",
            "start_time": 0.0,
            "end_time": uav_times_1[1]
        },
        {
            "type": "move_to_location",
            "ins_id": "move_to_pickup",
            "location": {"lat": landing_point[0], "lon": landing_point[1]},
            "start_time": uav_times_1[1],
            "end_time": ugv_arrival_time_1
        },
        {
            "type": "allow_land_on_UGV",
            "location": {"lat": landing_point[0], "lon": landing_point[1]},
            "ins_id": "land_1",
            "start_time": ugv_arrival_time_1,
            "end_time": uav_times_1[5]
        }
    ]
}

print("\nMISSION 1 PLANS:")
print("UAV Plan:", plan_uav_1)
print("\nUGV Plan:", plan_ugv_1)

# =============================================================================
# MISSION 2: Multi-survey with relay A -> RECON_1 -> TARGET_3 -> RECON_2 -> E (land)
# =============================================================================
print("\n" + "="*60)
print("MISSION 2: Multi-Survey A -> RECON_1 -> TARGET_3 -> RECON_2 -> E (land)")
print("="*60)

# Mission 2 waypoints  
start_point_2 = (41.870046, -87.650351)  # Node A
recon_1 = survey_points["RECON_1"]       # Recon point 1
target_3 = survey_points["TARGET_3"]     # Target survey
recon_2 = survey_points["RECON_2"]       # Recon point 2
landing_point_2 = (41.869960, -87.650180)  # Node E

# Calculate UAV times
current_time = 0
uav_waypoints_2 = [start_point_2, recon_1, target_3, recon_2, landing_point_2]
uav_times_2 = [current_time]

# Takeoff
current_time += TAKEOFF_TIME
uav_times_2.append(current_time)

# Flight segments
for i in range(len(uav_waypoints_2) - 1):
    dist = haversine_distance(uav_waypoints_2[i], uav_waypoints_2[i + 1])
    flight_time = dist / UAV_SPEED
    current_time += flight_time
    uav_times_2.append(current_time)
    print(f"UAV segment {i+1}: {dist:.1f}m, {flight_time:.1f}s")

# Landing
current_time += LANDING_TIME
uav_times_2.append(current_time)

# UGV route: A -> B -> E (using road network)
ugv_path_2 = nx.shortest_path(road_network, "A", "E", weight="weight")
ugv_total_dist_2 = nx.shortest_path_length(road_network, "A", "E", weight="weight")
ugv_travel_time_2 = ugv_total_dist_2 / UGV_SPEED

print(f"UGV route {' -> '.join(ugv_path_2)}: {ugv_total_dist_2:.1f}m, {ugv_travel_time_2:.1f}s")
print(f"Mission duration: {current_time:.1f}s")

# Build UGV plan with intermediate stops
ugv_plan_2_actions = []
ugv_current_time = 0

# Allow takeoff
ugv_plan_2_actions.append({
    "type": "allow_take_off_from_UGV",
    "location": {"lat": start_point_2[0], "lon": start_point_2[1]},
    "ins_id": "take_off_1", 
    "start_time": 0.0,
    "end_time": uav_times_2[1]
})

ugv_current_time = uav_times_2[1]

# Move through road network to final position
for i in range(len(ugv_path_2) - 1):
    start_node = ugv_path_2[i]
    end_node = ugv_path_2[i + 1] 
    
    segment_dist = road_network.edges[(start_node, end_node)]["weight"]
    segment_time = segment_dist / UGV_SPEED
    ugv_current_time += segment_time
    
    end_coord = (road_network.nodes[end_node]["lat"], road_network.nodes[end_node]["lon"])
    ugv_plan_2_actions.append({
        "type": "move_to_location",
        "ins_id": f"move_to_{end_node}",
        "location": {"lat": end_coord[0], "lon": end_coord[1]},
        "start_time": ugv_current_time - segment_time,
        "end_time": ugv_current_time
    })

# Allow landing
ugv_plan_2_actions.append({
    "type": "allow_land_on_UGV",
    "location": {"lat": landing_point_2[0], "lon": landing_point_2[1]},
    "ins_id": "land_1",
    "start_time": max(ugv_current_time, uav_times_2[-2]),
    "end_time": uav_times_2[-1]
})

plan_uav_2 = {
    "msg_type": "UAV_PLAN",
    "plan_id": 2,
    "new_plan": [
        {
            "type": "takeoff_from_UGV",
            "ins_id": "take_off_1", 
            "end_time": uav_times_2[1]
        },
        {
            "type": "move_to_location",
            "ins_id": "recon_1",
            "location": {"lat": recon_1[0], "lon": recon_1[1]},
            "end_time": uav_times_2[2]
        },
        {
            "type": "move_to_location",
            "ins_id": "survey_target_3",
            "location": {"lat": target_3[0], "lon": target_3[1]},
            "end_time": uav_times_2[3]
        },
        {
            "type": "move_to_location",
            "ins_id": "recon_2", 
            "location": {"lat": recon_2[0], "lon": recon_2[1]},
            "end_time": uav_times_2[4]
        },
        {
            "type": "move_to_location",
            "ins_id": "return_to_ugv_e",
            "location": {"lat": landing_point_2[0], "lon": landing_point_2[1]},
            "end_time": uav_times_2[5]
        },
        {
            "type": "land_on_UGV",
            "ins_id": "land_1",
            "end_time": uav_times_2[6]
        }
    ]
}

plan_ugv_2 = {
    "msg_type": "UGV_PLAN",
    "plan_id": 2,
    "new_plan": ugv_plan_2_actions
}

print("\nMISSION 2 PLANS:")
print("UAV Plan:", plan_uav_2)
print("\nUGV Plan:", plan_ugv_2)

# =============================================================================
# MISSION 3: Complex surveillance A -> TARGET_1 -> TARGET_4 -> TARGET_5 -> F (land)
# =============================================================================
print("\n" + "="*60)
print("MISSION 3: Complex Surveillance A -> TARGET_1 -> TARGET_4 -> TARGET_5 -> F")
print("="*60)

# Mission 3 waypoints
start_point_3 = (41.870046, -87.650351)  # Node A
target_1_3 = survey_points["TARGET_1"]   # Building surveillance
target_4_3 = survey_points["TARGET_4"]   # Industrial area
target_5_3 = survey_points["TARGET_5"]   # Residential survey
landing_point_3 = (41.870020, -87.650120)  # Node F

# Calculate UAV times
current_time = 0
uav_waypoints_3 = [start_point_3, target_1_3, target_4_3, target_5_3, landing_point_3]
uav_times_3 = [current_time]

# Takeoff
current_time += TAKEOFF_TIME
uav_times_3.append(current_time)

# Flight segments
for i in range(len(uav_waypoints_3) - 1):
    dist = haversine_distance(uav_waypoints_3[i], uav_waypoints_3[i + 1])
    flight_time = dist / UAV_SPEED
    current_time += flight_time
    uav_times_3.append(current_time)
    print(f"UAV segment {i+1}: {dist:.1f}m, {flight_time:.1f}s")

# Landing
current_time += LANDING_TIME
uav_times_3.append(current_time)

# UGV route: A -> C -> F (optimal road route)
ugv_path_3 = nx.shortest_path(road_network, "A", "F", weight="weight")
ugv_total_dist_3 = nx.shortest_path_length(road_network, "A", "F", weight="weight")
ugv_travel_time_3 = ugv_total_dist_3 / UGV_SPEED

print(f"UGV route {' -> '.join(ugv_path_3)}: {ugv_total_dist_3:.1f}m, {ugv_travel_time_3:.1f}s")
print(f"Mission duration: {current_time:.1f}s")

# Build detailed UGV plan
ugv_plan_3_actions = []
ugv_current_time = 0

# Allow takeoff
ugv_plan_3_actions.append({
    "type": "allow_take_off_from_UGV",
    "location": {"lat": start_point_3[0], "lon": start_point_3[1]},
    "ins_id": "take_off_1",
    "start_time": 0.0,
    "end_time": uav_times_3[1]
})

ugv_current_time = uav_times_3[1]

# Move through road network
for i in range(len(ugv_path_3) - 1):
    start_node = ugv_path_3[i]
    end_node = ugv_path_3[i + 1]
    
    segment_dist = road_network.edges[(start_node, end_node)]["weight"]
    segment_time = segment_dist / UGV_SPEED
    ugv_current_time += segment_time
    
    end_coord = (road_network.nodes[end_node]["lat"], road_network.nodes[end_node]["lon"])
    ugv_plan_3_actions.append({
        "type": "move_to_location",
        "ins_id": f"move_to_{end_node}",
        "location": {"lat": end_coord[0], "lon": end_coord[1]},
        "start_time": ugv_current_time - segment_time,
        "end_time": ugv_current_time
    })

# Allow landing
ugv_plan_3_actions.append({
    "type": "allow_land_on_UGV",
    "location": {"lat": landing_point_3[0], "lon": landing_point_3[1]},
    "ins_id": "land_1",
    "start_time": max(ugv_current_time, uav_times_3[-2]),
    "end_time": uav_times_3[-1]
})

plan_uav_3 = {
    "msg_type": "UAV_PLAN",
    "plan_id": 3,
    "new_plan": [
        {
            "type": "takeoff_from_UGV",
            "ins_id": "take_off_1",
            "end_time": uav_times_3[1]
        },
        {
            "type": "move_to_location",
            "ins_id": "survey_building",
            "location": {"lat": target_1_3[0], "lon": target_1_3[1]},
            "end_time": uav_times_3[2]
        },
        {
            "type": "move_to_location", 
            "ins_id": "survey_industrial",
            "location": {"lat": target_4_3[0], "lon": target_4_3[1]},
            "end_time": uav_times_3[3]
        },
        {
            "type": "move_to_location",
            "ins_id": "survey_residential",
            "location": {"lat": target_5_3[0], "lon": target_5_3[1]},
            "end_time": uav_times_3[4]
        },
        {
            "type": "move_to_location",
            "ins_id": "return_to_ugv_f",
            "location": {"lat": landing_point_3[0], "lon": landing_point_3[1]},
            "end_time": uav_times_3[5]
        },
        {
            "type": "land_on_UGV",
            "ins_id": "land_1",
            "end_time": uav_times_3[6]
        }
    ]
}

plan_ugv_3 = {
    "msg_type": "UGV_PLAN", 
    "plan_id": 3,
    "new_plan": ugv_plan_3_actions
}

print("\nMISSION 3 PLANS:")
print("UAV Plan:", plan_uav_3)
print("\nUGV Plan:", plan_ugv_3)

print("\n" + "="*60)
print("MISSION SUMMARY")
print("="*60)
print(f"Mission 1: Survey mission - Duration: {uav_times_1[-1]:.1f}s ({uav_times_1[-1]/60:.1f} min)")
print(f"Mission 2: Multi-recon mission - Duration: {uav_times_2[-1]:.1f}s ({uav_times_2[-1]/60:.1f} min)")
print(f"Mission 3: Complex surveillance - Duration: {uav_times_3[-1]:.1f}s ({uav_times_3[-1]/60:.1f} min)")
print("\nKey Constraint: UAV can visit any GPS point, but MUST land only at road network nodes")
print("Road network nodes for landing: A, B, C, D, E, F")
print("Survey points (UAV visit only): TARGET_1, TARGET_2, TARGET_3, TARGET_4, TARGET_5, RECON_1, RECON_2")