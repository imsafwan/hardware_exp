import yaml
import networkx as nx
import numpy as np
import os

def haversine(coord1, coord2):
    R = 6371000  # Earth radius in meters
    lat1, lon1 = np.radians(coord1)
    lat2, lon2 = np.radians(coord2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = np.sin(dlat/2.0)**2 + np.cos(lat1)*np.cos(lat2)*np.sin(dlon/2.0)**2
    c = 2 * np.arcsin(np.sqrt(a))

    return R * c  # meters

def yaml_to_dict(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

class ScenarioParameters:

    def __init__(self, file_path):
        data_dict = yaml_to_dict(file_path)
        nodes_info = data_dict['scenario']['nodes']
        
        nodes_connections = data_dict['scenario']['connections']
        

        corrd_to_id_map = {}
        id_to_coord_map = {}
        uav_data_points = []
        ugv_data_points = []

        for i in nodes_info:
            corrd_to_id_map[(round(i['location']['x'], 6), round(i['location']['y'], 6))] = i['ID']
            id_to_coord_map[i['ID']] = (round(i['location']['x'], 6), round(i['location']['y'], 6))

        for i in nodes_info:
            if i['type'] == 'air_only' :
                uav_data_points.append(  ( round(i['location']['x'], 6), round(i['location']['y'], 6) ) )

        for i in nodes_info:
            if i['type'] != 'air_only' :
                ugv_data_points.append( ( round(i['location']['x'], 6), round(i['location']['y'], 6) ) )


        # Initialize the graph
        G = nx.Graph()

        # Add edges with weights and ensure that all nodes are referred by identifiers
        for connection in nodes_connections:
            end1 = connection['end1']
            end2 = connection['end2']
            coord1 = id_to_coord_map[end1]
            coord2 = id_to_coord_map[end2]
            weight = haversine(coord1, coord2) #int(np.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2) * 1)  # m
            G.add_edge(coord1, coord2, weight=weight)

        self.corrd_to_id_map = corrd_to_id_map
        self.id_to_coord_map = id_to_coord_map
        self.road_network_G = G
        self.ugv_data_points = ugv_data_points
        uav_data_points_nv = []
        for i in nodes_info:
            if i['type'] == 'air_only' and i['time_last_service'] == 0.0 :
                uav_data_points_nv.append(  ( round(i['location']['x'], 6), round(i['location']['y'], 6) ) )
        
        self.uav_data_points = uav_data_points_nv
        self.mission_elapsed_time = data_dict['time']
        self.depot_index = 0

        # self.start_coordinates = [id_to_coord_map['DepotA']]
        # self.end_coordinates = [ id_to_coord_map['DepotB'], id_to_coord_map['DepotC'] ]
        # self.state_ID = data_dict['ID']
        # self.end_depot = id_to_coord_map['DepotB']
        

        # self.starting_point = []

        # for agent in data_dict['agents']:
        #     starting_p = (round(agent['location']['x'], 6), round(agent['location']['y'], 6))
        #     if starting_p in G.nodes:
        #         sp = starting_p
        #     else:
        #         # Find nearest node in G based on Euclidean distance
        #         sp = min(G.nodes, key=lambda node: np.linalg.norm(np.array(starting_p) - np.array(node)))
        #         print('\n {} agent is not at any road network node, it assumes to start its journey from nearest node {} on road network'.format(agent['ID'], sp))

        #     self.starting_point.append(sp)
        

        #self.api_v = data_dict['API_version']


        # self.agent_id_dict= {}
        # agent_list = data_dict['agents']
        # for agent_ix in range(len(agent_list)):
        #     if agent_ix == 0:
        #        self.agent_id_dict['UAV_01'] = agent_list[agent_ix]['ID']
        #     elif agent_ix == 1:
        #        self.agent_id_dict['UAV_02'] = agent_list[agent_ix]['ID']
        #     elif agent_ix == 2:
        #        self.agent_id_dict['UGV_01'] = agent_list[agent_ix]['ID']

        

        # self.agent_id_dict_reverse = {value: key for key, value in self.agent_id_dict.items()}



        # self.age_period_dict = {}
        # self.last_visit_dict = {}

        # for node_i in nodes_info:
            
        #     if node_i['service_times']:  # Check if service_times is not empty
        #         last_service_time = node_i['service_times'][-1]
        #     else:
        #         last_service_time = node_i['appearance_time']  # Default value when service_times is empty


        #     self.last_visit_dict[( round(node_i['location']['x'], 6), round(node_i['location']['y'], 6) )] = last_service_time

        #     self.age_period_dict[( round(node_i['location']['x'], 6), round(node_i['location']['y'], 6) )] = self.mission_elapsed_time - last_service_time

        
        # if all(value == 0 for value in self.age_period_dict.values()):
        #     self.age_period_dict = {key: 500 for key in self.age_period_dict}












        # Correct position mapping: Map identifiers to their coordinates
        # import matplotlib.pyplot as plt
        # pos = {node: node for node in G.nodes()}

        # label_G = {node: corrd_to_id_map[node] for node in G.nodes()}

        # # Draw the graph
        # nx.draw(G, pos, labels= label_G, node_color='skyblue', node_size=700) 
        # nx.draw_networkx_edge_labels(G, pos, edge_labels=nx.get_edge_attributes(G, 'weight'))

        # plt.title('Graph with Weighted Edges')
        # plt.grid(True)
        # plt.show()
 