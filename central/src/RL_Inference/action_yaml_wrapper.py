# -*- coding: utf-8 -*-
"""
Created on Sun Nov  5 00:57:06 2023
@author: safwan

Used to calculate mission cost (time) from given UAV-UGV cooperative actions.
Helpful for heuristic or DRL-based solutions.
"""

import ast
import math
import os
import pandas as pd
import yaml
import networkx as nx
import numpy as np


# ============================================================
# Helper functions
# ============================================================
def haversine(coord1, coord2):
    """Calculate great-circle distance (m) between two GPS coordinates."""
    R = 6371000  # Earth radius in meters
    lat1, lon1 = np.radians(coord1)
    lat2, lon2 = np.radians(coord2)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = np.sin(dlat / 2.0)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2.0)**2
    return R * 2 * np.arcsin(np.sqrt(a))


def yaml_to_dict(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


# ============================================================
# Scenario Reader (YAML-based)
# ============================================================
class ScenarioParameters:
    """Parses YAML file to extract UAV and UGV node information."""

    def __init__(self, file_path):
        data = yaml_to_dict(file_path)
        nodes_info = data['scenario']['nodes']
        connections = data['scenario']['connections']

        # Map node IDs to coordinates
        id_to_coord = {n['ID']: (round(n['location']['x'], 6), round(n['location']['y'], 6))
                       for n in nodes_info}
        self.id_to_coord = id_to_coord
        coord_to_id = {v: k for k, v in id_to_coord.items()}
        self.coord_to_id = coord_to_id

        # Separate UAV and UGV points
        self.uav_data_points = [
                (round(n['location']['x'], 6), round(n['location']['y'], 6))
                for n in nodes_info
                if n['type'] == 'air_only' and n['time_last_service'] == 0
            ]
        self.ugv_data_points = [(round(n['location']['x'], 6), round(n['location']['y'], 6))
                                for n in nodes_info if n['type'] == 'road']

        # Build road network graph
        G = nx.Graph()
        for c in connections:
            c1, c2 = id_to_coord[c['end1']], id_to_coord[c['end2']]
            G.add_edge(c1, c2, weight=haversine(c1, c2))
        self.road_network_G = G

        # Starting positions (UAV & UGV assumed co-located)
        agents = data['agents']
        self.starting_loc = (round(agents[0]['location']['x'], 6),
                             round(agents[0]['location']['y'], 6))


# ============================================================
# Environment to calculate UAV-UGV mission cost
# ============================================================
class UAV_UGV_Env:
    def __init__(self, scenario_path):
        """Initialize from a given scenario YAML file."""
        self.scene = ScenarioParameters(scenario_path)

        # Constants
        self.uav_speed = 0.89
        self.ugv_speed = 0.2
        self.takeoff_time = 12
        self.landing_time = 20

        # Mission nodes
        self.uav_mission_points = self.scene.uav_data_points
        self.ugv_mission_points = self.scene.ugv_data_points
        self.action_space = self.ugv_mission_points + self.uav_mission_points
        self.N_ugv = len(self.ugv_mission_points)

        # Initial states
        self.uav_position = self.scene.starting_loc
        self.ugv_position = self.scene.starting_loc

    def cost_cal(self, actions):
        """
        actions: [(index, agent)]
        agent = 1 → UAV executes this step
        Returns: dict of sortie breakdowns for UAV & UGV.
        """
        sortie_n = 1
        uav_mov_id = 0
        ugv_mov_id = 0
        uav_position, ugv_position = self.uav_position, self.ugv_position
        uav_time, ugv_time = self.takeoff_time, self.takeoff_time

        # Initialize action logs
        uav_action_yaml = {f'sortie {sortie_n}': [
            {'type': 'takeoff_from_UGV', 'location': {'lat':uav_position[0], 'lon': uav_position[1]}, 'ins_id': 'take_off_1', 'aoi_id': self.scene.coord_to_id[uav_position],  'end_time': round(uav_time, 2)}
        ]}
        ugv_action_yaml = {f'sortie {sortie_n}': [
            {'type': 'allow_take_off_from_UGV', 'location': {'lat':ugv_position[0], 'lon': ugv_position[1]}, 'ins_id': 'take_off_1', 'end_time': round(ugv_time, 2)}
        ]}

        
        if actions[-1][0] == 0:  # If last action is UGV, remove it (no landing)
              actions = actions[:-1]

        # === Iterate through actions ===
        for ix, (j, agent) in enumerate(actions):
            # UAV actions only
            if agent != 1:
                continue

            target = self.action_space[j]
            step_time = haversine(uav_position, target) / self.uav_speed
            uav_time += step_time
            uav_position = target

            # UAV movement
            uav_action_yaml[f'sortie {sortie_n}'].append(
                {'type': 'move_to_location', 'location': {'lat':uav_position[0], 'lon': uav_position[1]}, 'ins_id': 'move_{}'.format(uav_mov_id), 'aoi_id': self.scene.coord_to_id[uav_position],  'end_time': round(uav_time,2)}
            )
            uav_mov_id += 1
            

            # --- Recharge event ---
            if j < self.N_ugv:
                # Compute UGV path to target
                path = nx.shortest_path(self.scene.road_network_G, ugv_position, target, weight='weight')

                for k in range(1, len(path)):
                    step_dist = haversine(path[k - 1], path[k])
                    step_time = step_dist / self.ugv_speed
                    ugv_time += step_time
                    ugv_position = path[k]
                    ugv_action_yaml[f'sortie {sortie_n}'].append(
                        {'type': 'move_to_location', 'location': {'lat':ugv_position[0], 'lon': ugv_position[1]}, 'ins_id': 'move_{}'.format(ugv_mov_id), 'aoi_id': self.scene.coord_to_id[ugv_position], 'end_time': round(ugv_time, 2)}
                    )
                    ugv_mov_id += 1

                ugv_action_yaml[f'sortie {sortie_n}'].append(
                    {'type': 'allow_land_on_UGV', 'location': {'lat':ugv_position[0], 'lon': ugv_position[1]}, 'ins_id': 'land_1', 'aoi_id': self.scene.coord_to_id[ugv_position],
                     'end_time': round(ugv_time + self.landing_time, 2)}
                )

                # UAV landing
                uav_time += self.landing_time
                uav_action_yaml[f'sortie {sortie_n}'].append(
                    {'type': 'land_on_UGV', 'location': {'lat':uav_position[0], 'lon': uav_position[1]}, 'ins_id': 'land_1', 'aoi_id': self.scene.coord_to_id[uav_position],  'end_time': round(uav_time, 2)}
                )

                # Start next sortie
                if ix != len(actions)-1:  # Not the last action

                    sortie_n += 1
                    uav_mov_id = 0
                    ugv_mov_id = 0
                    uav_time, ugv_time = self.takeoff_time, self.takeoff_time

                    uav_action_yaml[f'sortie {sortie_n}'] = [
                        {'type': 'takeoff_from_UGV', 'location': {'lat':uav_position[0], 'lon': uav_position[1]}, 'ins_id': 'take_off_1', 'aoi_id': self.scene.coord_to_id[uav_position], 'end_time': round(uav_time, 2)}
                    ]
                    ugv_action_yaml[f'sortie {sortie_n}'] = [
                        {'type': 'allow_take_off_from_UGV', 'location': {'lat':ugv_position[0], 'lon': ugv_position[1]}, 'ins_id': 'take_off_1', 'aoi_id': self.scene.coord_to_id[ugv_position], 'end_time': round(ugv_time, 2)}
                    ]

        # Compute total mission duration (max of both)
        total_time = max(uav_time, ugv_time)

        return {
            "uav_action_yaml": uav_action_yaml,
            "ugv_action_yaml": ugv_action_yaml,
            "total_mission_time": round(total_time, 2)
        }


# ============================================================
# Driver: Compute cost for heuristic/DRL solutions
# ============================================================
def get_yaml(actions_list, scenario_path):
    """Reads actions_{method}.csv and returns sortie breakdowns."""
    results = []
    for ins, route_str in enumerate(actions_list):
        env = UAV_UGV_Env(scenario_path)

        # Only parse if it's a string
        if isinstance(route_str, str):
            actions = ast.literal_eval(route_str)
        else:
            actions = route_str  # already a list

        sortie_yaml = env.cost_cal(actions)
        
        results.append(sortie_yaml)
    return results


# ============================================================
# Example usage
# ============================================================
# if __name__ == "__main__":
#     scenario_yaml = "Inputs/scene.yaml"
#     method = "RL_greedy"

#     results = get_scores_for_method(method, scenario_yaml)
#     print("\n--- UAV Action YAML ---")
#     print(results[0]["uav_action_yaml"])
#     print("\n--- UGV Action YAML ---")
#     print(results[0]["ugv_action_yaml"])
#     print(f"\nTotal mission time: {results[0]['total_mission_time']} s")