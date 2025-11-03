from copy import deepcopy
import networkx as nx
import math
from geopy.distance import geodesic
import logging
from scenario_helper import ScenarioParameters

scene = ScenarioParameters('Inputs/scene.yaml')





logger = logging.getLogger(__name__)

def logger_print(msg, level="info"):
    print(msg)
    if level == "info":
        logger.info(msg)
    elif level == "warn":
        logger.warn(msg)
    elif level == "error":
        logger.error(msg)
    elif level == "debug":
        logger.debug(msg)


# Function to find nearest node
def find_nearest_node(graph, loc):
    nearest_node = None
    nearest_node_id = None
    min_dist = float("inf")
    for node, data in graph.nodes(data=True):
        node_loc = (data["lat"], data["lon"])
        dist = geodesic(loc, node_loc).meters
        if dist < min_dist:
            min_dist = dist
            nearest_node = node_loc
            nearest_node_id = node
    return nearest_node, nearest_node_id


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


def get_node_id(road_network, coord):
    min_dist = float("inf")
    nearest_node = None
    for node in road_network.nodes:
        node_coord = (road_network.nodes[node]["lat"], road_network.nodes[node]["lon"])
        d = haversine_distance(coord, node_coord)
        if d < min_dist:
            min_dist = d
            nearest_node = node
    return nearest_node

#---------------------------------------------------------------------------------#  
uav_full_flying_time = 100
uav_flying_time = uav_full_flying_time*0.9 # sec = min*60

uav_velocity = 0.9 # m/s
land_time = 20.0 # seconds
ugv_velocity = 0.2 # m/s
#---------------------------------------------------------------------------------#


def uav_needs_replan(uav_state, ugv_state, uav_actions, road_network):
    
    actions = deepcopy(uav_actions)

    #preliminary checks
    if len(actions) == 1:   # only landing remains 
        return actions, "KEEP_OLD_RENDEZVOUS"
    
    if len(actions) == 0:   # all actions are terminated
        return [], "PLAN_END"

    # 0. Try full sortie
    if can_uav_reach(actions, uav_state):
        return actions, "KEEP_OLD_RENDEZVOUS"
    
    # 1. Compute candidate rendezvous points (reachable by both) ---
    candidate_rendezvous_points = compute_candidate_points(uav_state, ugv_state, road_network)
    
    actions_trim = actions[:-2].copy()  # remove last move + land pair (to try new land first)

    if actions_trim != []:
        found = False
        k = len(actions_trim)
        for i in range(k, -1, -1):
            candidate_seq = actions[:i]
            feasible, new_rp = find_new_rp(candidate_seq, uav_state, candidate_rendezvous_points)
            if feasible:
                move_to_new_rp = {"type": "move_to_location", "ins_id": "move_new", "location": new_rp}
                new_land = {"type": "land_on_UGV", "ins_id": "land_1", "location": new_rp}
                return candidate_seq + [move_to_new_rp, new_land], f"ROLLOUT_NEW_R (trimmed to {i})"
        # If we reach here → loop failed
        logger_print('no feasible rp found in rollback — trying relaxed endurance (0.95× full)')
    else:
        logger_print('cannot find new rp — trying relaxed endurance (0.95× full)')

    # Relaxed endurance fallback (common to both cases)
    relaxed_uav_state = deepcopy(uav_state)
    temp_flying_time = globals()["uav_flying_time"]
    globals()["uav_flying_time"] = 0.95 * uav_full_flying_time

    candidate_rendezvous_points = compute_candidate_points(uav_state, ugv_state, road_network)
    feasible_relaxed, new_rp_relaxed = find_new_rp([], relaxed_uav_state, candidate_rendezvous_points)
    globals()["uav_flying_time"] = temp_flying_time

    if feasible_relaxed:
        move_to_new_rp = {"type": "move_to_location", "ins_id": "move_relaxed", "location": new_rp_relaxed}
        new_land = {"type": "land_on_UGV", "ins_id": "land_1", "location": new_rp_relaxed}
        return [move_to_new_rp, new_land], "RELAXED_NEW_RP_ONLY"
    else:
        logger_print('still no feasible rp after relaxation')
        return [], "GLOBAL_REPLAN"


def create_ugv_plan(ugv_state, new_rp, road_network):
    """
    Create UGV move actions to reach new rendezvous point.
    """

    ugv_loc = ugv_state["loc"]
    if ugv_state["curr_road_point"] is None:
        nearest_node, nearest_node_id = find_nearest_node(road_network, ugv_loc)
        ugv_state["curr_road_point"] = nearest_node

    start_node_id = get_node_id(road_network,    ugv_state["curr_road_point"])
    target_node_id = get_node_id(road_network,    ( new_rp["lat"], new_rp["lon"]))

    # shortest path on road network
    path = nx.shortest_path(road_network, start_node_id, target_node_id, weight="weight")

    ugv_plan = []
    dist = haversine_distance(ugv_loc, ugv_state["curr_road_point"])
    action_time = dist / ugv_velocity
    elapsed = ugv_state['time_elapsed'] + action_time
    prev_loc = ugv_state["curr_road_point"]

    # # First move (from current/next_road_point)
    # tgt = (ugv_state["curr_road_point"][0], ugv_state["curr_road_point"][1])
    # dist = haversine_distance(prev_loc, tgt)
    # action_time = dist / ugv_velocity if dist > 0 else 0
    # elapsed += action_time
    # ugv_plan.append({
    #     "type": "move_to_location",
    #     "ins_id": f"ugv_move_{0}",
    #     "location": {"lat": tgt[0], "lon": tgt[1]},
    #     'end_time': elapsed
    # })
    # prev_loc = tgt

    for idx, node in enumerate(path[1:]):  # skip start node
        lat = road_network.nodes[node]["lat"]
        lon = road_network.nodes[node]["lon"]
        tgt = (lat, lon)
        dist = haversine_distance(prev_loc, tgt)
        action_time = dist / ugv_velocity
        elapsed += action_time
        ugv_plan.append({
            "type": "move_to_location",
            "ins_id": f"ugv_move_{idx+1}",
            "location": {"lat": lat, "lon": lon},
            'end_time': elapsed
        })
        prev_loc = tgt

    # Add land action for UAV
    elapsed += land_time
    ugv_plan.append({
        "type": "allow_land_on_UGV",
        "ins_id": "land_1",
        "location": {"lat": new_rp["lat"], "lon": new_rp["lon"]},
        'end_time': elapsed
    })

    return ugv_plan


def uav_is_replanning(uav_state, ugv_state, uav_actions, ugv_remaining_actions, road_network):

    

    uav_plan, status = uav_needs_replan(uav_state, ugv_state, uav_actions, road_network)

    logger_print(f"Replan status: {status}")

    if status == "PLAN_END":
        ugv_plan = ugv_remaining_actions
        uav_plan = []
        logger_print("UAV has completed all actions.")
        
    elif status == "GLOBAL_REPLAN": # uav will immediately land
        uav_loc = uav_state["loc"]
        uav_time = uav_state["time_elapsed"]
        uav_plan = [  {"type": "land_on_UGV", "ins_id": "land_1", "location": {"lat": uav_loc[0],"lon": uav_loc[1]},'end_time': uav_time  + land_time}, ]
        ugv_plan = [{
        "type": "allow_land_on_UGV",
        "location": {
            "lat": uav_loc[0],
            "lon": uav_loc[1]
        },
        "ins_id": "land_1",
        "end_time": uav_time  + land_time
    }]



    elif status == 'KEEP_OLD_RENDEZVOUS':

        uav_cur_loc = uav_state["loc"]
        elapsed = uav_state["time_elapsed"]

        # add end time to each action
        for act in uav_plan:
            if act["type"] == "move_to_location":
                tgt = (act["location"]["lat"], act["location"]["lon"])
                act['aoi_id'] =    scene.corrd_to_id_map[tgt]   #'AOI_unknown'  # to be updated later
                dist = haversine_distance(uav_cur_loc, tgt)
                action_time = dist / uav_velocity
                elapsed += action_time
                act['end_time'] = elapsed
                uav_cur_loc = tgt

            elif act["type"] == "land_on_UGV":
                elapsed += land_time
                act['end_time'] = elapsed

        ugv_plan = create_ugv_plan(ugv_state, uav_plan[-2]["location"], road_network)  # already comes with end_time

      
        #ugv_plan = ugv_remaining_actions
        logger_print(f"UAV plan: {uav_plan} \n")
        logger_print(f"UGV plan: {ugv_plan} \n")
        logger_print("No change on UAV/UGV plans.")

    else: # new rendezvous point

        uav_cur_loc = uav_state["loc"]
        elapsed = uav_state["time_elapsed"]

        # add end time to each action
        for act in uav_plan:
            if act["type"] == "move_to_location":
                tgt = (act["location"]["lat"], act["location"]["lon"])
                act['aoi_id'] =    scene.corrd_to_id_map[tgt] #'AOI_unknown'  # to be updated later
                dist = haversine_distance(uav_cur_loc, tgt)
                action_time = dist / uav_velocity
                elapsed += action_time
                act['end_time'] = elapsed
                uav_cur_loc = tgt

            elif act["type"] == "land_on_UGV":
                elapsed += land_time
                act['end_time'] = elapsed

        ugv_plan = create_ugv_plan(ugv_state, uav_plan[-2]["location"], road_network)  # already comes with end_time
        
        logger_print(f"uav state: {uav_state}")
        logger_print(f"old plan: {uav_actions}") 
        logger_print("New plan:")
        logger_print(f"UAV plan: {uav_plan} \n")
        logger_print(f"UGV plan: {ugv_plan} \n")
        
        

    return uav_plan , ugv_plan


# ------------------------> Helper functions (to be implemented properly) <------------------------ #


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
            #logger_print('Time to ', tgt, ' from ', uav_cur_loc, ' is ', dist / uav_velocity)
            future_actions_time += action_time
            uav_cur_loc = tgt
        elif act["type"] == "land_on_UGV":
            future_actions_time += land_time
    #logger_print('Total future actions time: ', future_actions_time, ' Remaining time: ', remaining_time)
    return future_actions_time <= remaining_time







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
            #logger_print('Time to ', tgt, ' from ', uav_cur_loc, ' is ', dist / uav_velocity)
            future_actions_time += dist / uav_velocity
            uav_cur_loc = tgt
        elif act["type"] == "land_on_UGV":
            raise KeyError("Should not have landing as action")
            


    

    # check each candidate rendezvous
    min_time = float('inf')
    best_rp = None
    for r in candidate_rendezvous_points:
        tgt = (r["lat"], r["lon"])
        dist = haversine_distance(uav_cur_loc, tgt)
        action_time = dist / uav_velocity

        logger_print( f"Time to reach rp {tgt} from {uav_cur_loc} is {dist / uav_velocity}")

        total_time = future_actions_time + action_time + land_time
        #logger_print('Total time to land on rp ', tgt, ' is ', total_time, ' Remaining time: ', remaining_time)

        if total_time <= remaining_time and total_time < min_time:
            min_time = total_time
            best_rp = r

    return best_rp is not None, best_rp
        



    
def compute_candidate_points(uav_state, ugv_state, road_network):
    
    uav_loc = uav_state["loc"]
    uav_elapsed = uav_state["time_elapsed"]
    uav_remaining_time = uav_flying_time - uav_elapsed - land_time

    ugv_loc = ugv_state["loc"]
    ugv_next_road_point = ugv_state["curr_road_point"]
    if ugv_next_road_point is None:
        nearest_node, nearest_node_id = find_nearest_node(road_network, ugv_loc)
        ugv_next_road_point = nearest_node
        


    ugv_next_road_point_id = get_node_id(road_network, ugv_next_road_point)

    dist_cur_loc_2_nxt_road = haversine_distance(ugv_loc, ugv_next_road_point)

    candidate_points = []
    
    for node in road_network.nodes:
        node_lat = road_network.nodes[node]["lat"]
        node_lon = road_network.nodes[node]["lon"]
        node_coord = (node_lat, node_lon)


        # UGV time to reach node
        dist_to_ugv =  nx.shortest_path_length(road_network, ugv_next_road_point_id, node, weight="weight")   # meters
        total_dist_to_ugv = dist_to_ugv + dist_cur_loc_2_nxt_road
        time_ugv = total_dist_to_ugv / ugv_velocity

        # UAV time to reach node
        dist_uav = haversine_distance(uav_loc, node_coord)
        time_uav = dist_uav / uav_velocity

        # Rendezvous feasible if both can arrive within UAV endurance
        if max(time_ugv, time_uav) <= uav_remaining_time:
            candidate_points.append({"lat": node_lat, "lon": node_lon})

    return candidate_points 








    


























# # --- UAV state ---
# uav_state = {
#     "loc": (41.870050, -87.650200),   # current GPS position
#     "time_elapsed": 60.0              # 1 minute into mission
# }

# # --- UGV state ---
# ugv_state = {
#     "loc": (41.870000, -87.650400),          # current GPS pos (off-road a bit)
#     "next_road_point": (41.870000, -87.650300)  # nearest road node
# }

# # --- Road network (simple 3-node graph) ---
# road_network = nx.Graph()

# road_network.add_node("A", lat=41.870000, lon=-87.650300)
# road_network.add_node("B", lat=41.870100, lon=-87.650250)
# road_network.add_node("C", lat=41.870050, lon=-87.650150)

# # Add edges with weights (meters, just dummy distances)
# road_network.add_edge("A", "B", weight=15.0)
# road_network.add_edge("B", "C", weight=20.0)
# road_network.add_edge("A", "C", weight=25.0)

# # --- UAV planned actions (as per your YAML) ---
# uav_actions = [
#     {"type": "move_to_location", "ins_id": "move_1",
#      "location": {"lat": 41.870111, "lon": -87.650272}},
#     {"type": "move_to_location", "ins_id": "move_2",
#      "location": {"lat": 41.870007, "lon": -87.650315}},
#     {"type": "move_to_location", "ins_id": "move_3",
#      "location": {"lat": 41.870017, "lon": -87.650253}},
#     {"type": "move_to_location", "ins_id": "move_4",
#      "location": {"lat": 41.870100, "lon": -87.650250}},
#     {"type": "land_on_UGV", "ins_id": "land_2"},
# ]




# uav_plan, ugv_plan = uav_is_replanning(uav_state, ugv_state, uav_actions, road_network)

