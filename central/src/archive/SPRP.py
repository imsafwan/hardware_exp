from copy import deepcopy
import networkx as nx

uav_flying_time = 1.85*60 # min*60
uav_velocity = 1.0 # m/s
land_time = 30.0 # seconds
ugv_velocity = 0.5 # m/s





def replan_with_rendezvous(uav_state, ugv_state, uav_actions, road_network):
    
    actions = deepcopy(uav_actions)

    # 0. Compute candidate rendezvous points (reachable by both) ---
    candidate_rendezvous_points = compute_candidate_points(uav_state, ugv_state, road_network)

    # 1. Try full sortie
    if can_uav_reach(actions, uav_state):
        return actions, "KEEP_OLD_RENDEZVOUS"
    
    # 2. Try new rendezvous (without trimming)
    feasible, new_rp = find_new_rp(actions[:-1], uav_state, candidate_rendezvous_points)
    if feasible:
        new_land = {"type": "land_on_UGV", "ins_id": "land_new", "location": new_rp}
        return actions[:-1] + [new_land], "NEW_RENDEZVOUS"
        
    # 3. Rollback search (trim move-to-location one by one before final land)
    k = len(actions) - 1  # last is land
    for i in range(k-1, -1, -1):
        candidate_seq = actions[:i]  # trim
        #if can_uav_reach(candidate_seq, uav_state):
        feasible, new_rp = find_new_rp(candidate_seq, uav_state, candidate_rendezvous_points)
        if feasible:
            new_land = {"type": "land_on_UGV", "ins_id": "land_new", "location": new_rp}
            return candidate_seq + [new_land], f"ROLLOUT_NEW_R (trimmed to {i})"

    return [], "GLOBAL_REPLAN"  # nothing worked















# ------------------------
# Helper functions (to be implemented properly)


def can_uav_reach(seq, uav_cur_state):
    uav_cur_loc = uav_cur_state["loc"]
    elapsed = uav_cur_state["time_elapsed"]
    endurance = uav_flying_time

    remaining_time = endurance - elapsed
    future_actions_time = 0.0

    for act in seq:
        if act["type"] == "move_to_location":
            tgt = (act["location"]["lat"], act["location"]["lon"])
            dist = haversine_distance(uav_cur_loc, tgt)
            action_time = dist / uav_velocity
            future_actions_time += action_time
            uav_cur_loc = tgt
        elif act["type"] == "land_on_UGV":
            future_actions_time += land_time

    return future_actions_time <= remaining_time

# def can_uav_reach_with_landing(seq, uav_cur_state):
#     uav_cur_loc = uav_cur_state["loc"]
#     elapsed = uav_cur_state["time_elapsed"]
#     endurance = uav_flying_time

#     remaining_time = endurance - elapsed
#     future_actions_time = 0.0

#     for act in seq:
#         if act["type"] == "move_to_location":
#             tgt = (act["location"]["lat"], act["location"]["lon"])
#             dist = haversine_distance(uav_cur_loc, tgt)
#             action_time = dist / uav_velocity
#             future_actions_time += action_time
#             uav_cur_loc = tgt
#         elif act["type"] == "land_on_UGV":
#             raise KeyError("Should not have landing as action")
    
#     future_actions_time += land_time

#     return future_actions_time <= remaining_time


def find_new_rp(seq, uav_cur_state, candidate_rendezvous_points):
    uav_cur_loc = uav_cur_state["loc"]
    elapsed = uav_cur_state["time_elapsed"]
    endurance = uav_flying_time
    remaining_time = endurance - elapsed
    future_actions_time = 0.0

    # simulate all actions except last "move+land" pair
    for act in seq:
        if act["type"] == "move_to_location":
            tgt = (act["location"]["lat"], act["location"]["lon"])
            dist = haversine_distance(uav_cur_loc, tgt)
            future_actions_time += dist / uav_velocity
            uav_cur_loc = tgt
        elif act["type"] == "land_on_UGV":
            future_actions_time += land_time

    # check each candidate rendezvous
    min_time = float('inf')
    best_rp = None
    for r in candidate_rendezvous_points:
        tgt = (r["lat"], r["lon"])
        dist = haversine_distance(uav_cur_loc, tgt)
        action_time = dist / uav_velocity

        total_time = future_actions_time + action_time + land_time

        if total_time <= remaining_time and total_time < min_time:
            min_time = total_time
            best_rp = r

    return best_rp is not None, best_rp
        




import math

def haversine_distance(coord1, coord2):
    """
    Calculate great-circle distance (in meters) between two GPS points.
    coord1, coord2 = (lat, lon) in decimal degrees
    """
    R = 6371000  # Earth radius in meters

    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    return R * c







    
def compute_candidate_points(uav_state, ugv_state, road_network):
    
    uav_loc = uav_state["loc"]
    uav_elapsed = uav_state["time_elapsed"]
    uav_remaining_time = uav_flying_time - uav_elapsed - land_time

    ugv_loc = ugv_state["loc"]
    ugv_next_road_point = ugv_state["next_road_point"]

    dist_cur_loc_2_nxt_road = haversine_distance(ugv_loc, ugv_next_road_point)

    candidate_points = []
    for node in road_network.nodes:
        node_lat = road_network.nodes[node]["lat"]
        node_lon = road_network.nodes[node]["lon"]
        node_coord = (node_lat, node_lon)

        # UGV time to reach node
        dist_to_ugv = 0# nx.shortest_path_length(road_network, ugv_next_road_point, node, weight="weight")   # meters
        total_dist_to_ugv = dist_to_ugv + dist_cur_loc_2_nxt_road
        time_ugv = total_dist_to_ugv / ugv_velocity

        # UAV time to reach node
        dist_uav = haversine_distance(uav_loc, node_coord)
        time_uav = dist_uav / uav_velocity

        # Rendezvous feasible if both can arrive within UAV endurance
        if max(time_ugv, time_uav) <= uav_remaining_time:
            candidate_points.append({"lat": node_lat, "lon": node_lon})

    return candidate_points 



import networkx as nx

# --- UAV state ---
uav_state = {
    "loc": (41.870050, -87.650200),   # current GPS position
    "time_elapsed": 60.0              # 1 minute into mission
}

# --- UGV state ---
ugv_state = {
    "loc": (41.870000, -87.650400),          # current GPS pos (off-road a bit)
    "next_road_point": (41.870000, -87.650300)  # nearest road node
}

# --- Road network (simple 3-node graph) ---
road_network = nx.Graph()

road_network.add_node("A", lat=41.870000, lon=-87.650300)
road_network.add_node("B", lat=41.870100, lon=-87.650250)
road_network.add_node("C", lat=41.870050, lon=-87.650150)

# Add edges with weights (meters, just dummy distances)
road_network.add_edge("A", "B", weight=15.0)
road_network.add_edge("B", "C", weight=20.0)
road_network.add_edge("A", "C", weight=25.0)

# --- UAV planned actions (as per your YAML) ---
uav_actions = [
    {"type": "takeoff_from_UGV", "ins_id": "take_off_1"},
    {"type": "move_to_location", "ins_id": "move_1",
     "location": {"lat": 41.870111, "lon": -87.650272}},
    {"type": "move_to_location", "ins_id": "move_2",
     "location": {"lat": 41.870007, "lon": -87.650315}},
    {"type": "land_on_UGV", "ins_id": "land_1"},
    {"type": "takeoff_from_UGV", "ins_id": "take_off_2"},
    {"type": "move_to_location", "ins_id": "move_3",
     "location": {"lat": 41.870017, "lon": -87.650253}},
    {"type": "move_to_location", "ins_id": "move_4",
     "location": {"lat": 41.870021, "lon": -87.650211}},
    {"type": "land_on_UGV", "ins_id": "land_2"},
]



new_plan, status = replan_with_rendezvous(uav_state, ugv_state, uav_actions, road_network)

print("Replan status:", status)
print("New plan:")
for act in new_plan:
    print(act)
