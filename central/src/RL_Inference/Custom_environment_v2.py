

import gym
import torch
from termcolor import cprint



device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

index_of_interest = 25                            

class UAV_UGV_Env(gym.Env):
    
    
    def __init__(self):
        
         
         '''-------------------------------------------------------------------''' 
        
         
         self.batch_size = 256
         self.uav_speed = 0.89    # m/sec constant
         self.ugv_speed = 0.2   # m/sec constant
         self.prv_action = [None] * self.batch_size                            
         self.mission_points = None
         self.uav_fuel_constant = 58 # sec
         self.unit_factor = 1
         
         
         '''-------------------------------------------------------------------'''
         
         self.uav_mission_points = None                        # tensor [N, d]
         self.ugv_mission_points = None                        # tensor [N, d] 
         self.all_mission_points = None                       # tensor [N, d] 
         self.unique_mission_points = None
         self.unique_mission_points = None                    # tensor [1, N_unique, d]                                
         self.encoder_mission_space = None 
         self.agent =  torch.full((self.batch_size,), 1, dtype=torch.float32, device = device)                               # tensor [B]
         self.uav_fuel = torch.full((self.batch_size,), self.uav_fuel_constant, dtype=torch.float32, device = device)        # tensor [B]
         self.uav_pos_coord = None                                                                                           # tensor [B, d]
         self.ugv_pos_coord = None                                                                                           # tensor [B, d]
         self.refuel_stop = None                                                                                             # [B,d]
         self.depot_index = None
         self.mission_status = None                                                                                          # tensor [B, N]
         self.mission_time_elapsed = torch.zeros(self.batch_size, device = device)                                           # tensor [B, N] 
         self.infeasibility = torch.zeros(self.batch_size, device = device).bool()                                           # tensor [B]
         self.recharging_location = torch.zeros(self.batch_size, device = device).bool()                                     # tensor [B]
         self.refuel_stop_time = torch.zeros(self.batch_size, device = device)                                               # tensor [B]
         self.UGV_time = torch.zeros(self.batch_size, device = device)
         self.done_completely = torch.zeros(self.batch_size, device = device) 
         
         
         
         '''-------------------------------------------------------------------'''
         
         
    def cu_dis1(self, node1, node2):
        """
        node1, node2: [B, 2] (coordinates)
        self.ugv_loc: [B, N, 2]          # node coordinates
        self.sp_matrix: [B, N, N]        # shortest-path distance matrix along graph
        returns: [B]                     # graph distance between node1 and node2
        """
        loc = self.ugv_mission_points.to(device)        # [B, N, 2]
        sp_mat = self.dist_matrix.to(device)   # [B, N, N]
        B, N, _ = loc.shape

        # find nearest graph node to each given node
        d1 = torch.norm(loc - node1.unsqueeze(1), dim=-1)  # [B, N]
        d2 = torch.norm(loc - node2.unsqueeze(1), dim=-1)  # [B, N]

        ix1 = torch.argmin(d1, dim=1)  # [B]
        ix2 = torch.argmin(d2, dim=1)  # [B]

        # get graph (shortest-path) distance from sp_matrix
        graph_dist = sp_mat[torch.arange(B, device=device), ix1, ix2]  # [B]

        return graph_dist*self.unit_factor    
    
    def cu_dis_matrix(self, node_start, node_targets):
        """
        Compute graph distances from node_start [B, 2] to each target node [B, N, 2]
        using self.dist_matrix [B, N, N].

        Returns: [B, N]
        """
        loc = self.ugv_mission_points.to(device)     # [B, N, 2]
        sp_mat = self.dist_matrix.to(device)         # [B, N, N]
        B, N, _ = loc.shape

        # Nearest graph index to start node
        d_start = torch.norm(loc - node_start.unsqueeze(1), dim=-1)  # [B, N]
        ix_start = torch.argmin(d_start, dim=1)  # [B]

        # Precompute all target indices (0...N-1)
        target_indices = torch.arange(N, device=device).unsqueeze(0).expand(B, -1)  # [B, N]

        # Gather distances from shortest-path matrix
        graph_dist = sp_mat[torch.arange(B).unsqueeze(1), ix_start.unsqueeze(1), target_indices]  # [B, N]
        return graph_dist*self.unit_factor

    def calculate_distances(self, p1, p2):
          # Compute Euclidean distance between two sets of points
         return torch.norm((p1 - p2), dim=-1)* self.unit_factor # m #torch.sqrt(torch.sum((p1 - p2) ** 2, dim=-1))*1
    
    
   

    def step(self, actions, step):
    
    
        #print('action-->', actions[index_of_interest])
        
        
        if step == 0:
            if not ((self.refuel_stop_time == 0).all()):
                non_zero_elements = self.refuel_stop_time[self.refuel_stop_time != 0]
                print(f"Non-zero elements in refuel_stop_time: {non_zero_elements}")
                cprint(self.refuel_stop_time[index_of_interest], 'cyan')
            assert (self.refuel_stop_time == 0).all()
        
        
        action_indices = actions
        self.prv_action = actions

        batch_size = self.batch_size
        assert batch_size == actions.size(0)
        

        infeasible_penalties = torch.zeros(batch_size, device = device)        # initialize infeasible penalties 

        
        #------------ action coordinates ---------- #
        
        is_refuel_stop = action_indices < len(self.ugv_mission_points[0])       # action : recharge
        is_refuel_stop = is_refuel_stop.unsqueeze(-1)
        
        is_visit_point = (action_indices >= len(self.ugv_mission_points[0]))    # action: visiting mission points
        is_visit_point = is_visit_point.unsqueeze(-1)


        if self.encoder_mission_space.shape[0] != batch_size:
               self.encoder_mission_space = self.encoder_mission_space.repeat(batch_size, 1, 1)
        
               
        temp_refuel_points = self.encoder_mission_space[:, :, :2].clone() 
        
        refuel_coords = torch.where(is_refuel_stop, temp_refuel_points[torch.arange(batch_size, device = device), action_indices], torch.zeros(action_indices.shape[0], 2, device = device))
        visit_coords = torch.where(is_visit_point, temp_refuel_points[torch.arange(batch_size, device = device), action_indices], torch.zeros(action_indices.shape[0], 2, device = device))
        
        
        action_coords = refuel_coords + visit_coords                            # action coordinates [B, 2]
        
        

        # ------- unique action indices ------ #
        
        expanded_action_coords = action_coords.unsqueeze(1)
        matches = torch.all(self.unique_mission_points == expanded_action_coords, dim=-1)  # Shape: [B, N]
        matches_int = matches.int()
        unique_action_indices = torch.argmax(matches_int, dim=-1)                          # indices of action shape [B]
        assert batch_size == len(unique_action_indices)
        is_refuel_stop = is_refuel_stop.any(dim=1)
        is_visit_point = is_visit_point.any(dim=1)
        
        
        # ------- effect of actions --------- #
        
        
        
        step_elapsed_times = (torch.norm((action_coords - self.uav_pos_coord), dim=1)*self.unit_factor / self.uav_speed)
        
        
        
        
        mask = (is_refuel_stop == True)
        
        
        
        
        self.refuel_stop_time[mask] =  self.refuel_stop_time[mask] + (self.cu_dis1(self.refuel_stop, action_coords)/self.ugv_speed)[mask]
        

        #print('UAV arrival time --->', (self.mission_time_elapsed + step_elapsed_times)[index_of_interest])

        waiting_time = torch.where(is_refuel_stop, ( self.refuel_stop_time - (self.mission_time_elapsed + step_elapsed_times) ) , torch.zeros_like(action_indices, device = device).float())  # waiting time of UAV 
        waiting_time = torch.clamp(waiting_time, min=0) # if UGV arrives earlier UAV will have no waiting time

        
        #print('waiting time --->', waiting_time[index_of_interest])

        step_elapsed_times = step_elapsed_times +  waiting_time # accounts for waiting time
        
        exceeded_indices = torch.where(self.done_completely == 1)[0]
        if exceeded_indices.numel() > 0:
               step_elapsed_times[exceeded_indices] = 0

        self.uav_fuel = self.uav_fuel - (step_elapsed_times * 1) # fuel depletion
        #print('uav fuel ---->', self.uav_fuel[index_of_interest])
        fuel_depleted_mask = self.uav_fuel < 0 # Check for fuel levels less than zero for each batch instance
        
        #print('step elapsed times-->', self.mission_time_elapsed[index_of_interest])
        infeasible_penalties = torch.where(fuel_depleted_mask, torch.ones(batch_size, device = device), torch.zeros(batch_size, device = device))
        
        recharge_time = torch.where(is_refuel_stop, torch.tensor(30).float().to(device), torch.zeros_like(action_indices).float().to(device))
        
        step_elapsed_times = step_elapsed_times + recharge_time   # accounts for recharging time

        exceeded_indices = torch.where(self.done_completely == 1)[0]
        if exceeded_indices.numel() > 0:
               step_elapsed_times[exceeded_indices] = 0
        
        
        # -------- reward collection -------- #
        
        
        reward_collect = step_elapsed_times/60    #[B]
        
         
        # ----------- update state --------- #
        
        
        self.mission_status[torch.arange(self.batch_size), unique_action_indices] = 1 # updates age period of visited mission points as 0

        #print(self.mission_status[index_of_interest])
        
        
        
        self.mission_time_elapsed = self.mission_time_elapsed + step_elapsed_times # mission time elapsed
        #print('step elapsed times after recharging-->', self.mission_time_elapsed[index_of_interest])
        
        
        #print('mission time ----->', self.mission_time_elapsed[index_of_interest]/60)
        self.uav_pos_indices = actions.unsqueeze(1) 
        self.uav_pos_coord = action_coords
        
        self.uav_fuel[mask] = torch.full((self.batch_size,), self.uav_fuel_constant, dtype=torch.float32).to(device) [mask]
        
        
        
        self.recharging_location[is_visit_point] = torch.ones(self.batch_size).bool().to(device)[is_visit_point]    #   in those instances where after visiting self.recharging_location becomes false
        self.recharging_location[mask] = torch.zeros(self.batch_size).bool().to(device)[mask]   #   in those instances where after recharging self.recharging_location becomes false
        
        
        
        self.refuel_stop[mask] = action_coords[mask]
        self.refuel_stop_idx[mask] = unique_action_indices[mask].unsqueeze(1)
        self.refuel_stop_time[mask] = self.mission_time_elapsed.clone()[mask]
        
        

        # --------- reward from step ---------- #
        

        rewards = reward_collect 
        
        # -------- call state ------- #
        
        states = self.get_observation() 
        exceeded_indices = torch.where(self.done_completely == 1)[0]
        
    
        
        if exceeded_indices.numel() > 0:
            shrink = exceeded_indices.shape[0] 
        else :
            shrink = 0
        
        
        step_condition = torch.tensor([step > 60], dtype=torch.bool, device = device)
        shrink_condition = torch.tensor([shrink > int(batch_size*0.90)], dtype=torch.bool, device = device)
        
        N_uav_start = len(self.ugv_mission_points[0])  # UAVs start at index 5
        all_uavs_visited = torch.all(self.mission_status[:, N_uav_start:] == 1, dim=1)
        dones = torch.logical_and(all_uavs_visited, mask)
        dones = torch.logical_or(dones, self.infeasibility)
        
        self.done_completely = dones.clone().int()
        
        if self.done_completely[0] == 1 :
            cprint('<------------------Done-------------------->', 'yellow', attrs = ['bold'])
            print('Total mission time (min) : ', self.mission_time_elapsed[index_of_interest].item()/60.0)
        
        exceeded_indices = torch.where(self.done_completely == 1)[0]
        if exceeded_indices.numel() > 0:
            self.agent[exceeded_indices] = 0
        

        #dones = torch.logical_or(torch.all(self.mission_status == 1, dim=1), self.infeasibility)
        dones = torch.logical_or(dones, step_condition)
        dones = torch.logical_or(dones, shrink_condition)
        dones = dones.to(device)
        
        rewards = rewards.to(device)
        if dones.all():
           unfinished_batch_indices = torch.where(self.done_completely != 1)[0]
           

           # assign penalty = max mission time (so longer missions give higher penalty)
           rewards[unfinished_batch_indices] = 800 #max_mission_time / 60.0  # optional scaling to minutes
           
           for i, time1 in enumerate(self.mission_time_elapsed):
               if time1 == 0:
                  cprint(f"Instance {i} has a value of zero", 'cyan', attrs = ['bold'] )
                  print(self.encoder_mission_space[i])
                  cprint(self.depot_index[i], 'cyan', attrs = ['bold'])
        
        info = {}  
        
        
        return states, rewards, dones, info





    def reset(self, input):
       
    
         self.uav_mission_points =  input['uav loc']          # tensor [B, N, d]
         self.batch_size = self.uav_mission_points.size(0)
         
         self.ugv_mission_points =   input ['ugv loc']           # tensor [B, N, d]
         self.dist_matrix = input ['sp_matrix']                  # tensor [B, N, d]
         
         
         
         self.unique_mission_points = input['unique loc']     # tensor [B, N, d]
         
         self.encoder_mission_space = input['encoder space']  # tensor [B, N, d]
         
         self.agent =  torch.full((self.batch_size,), 1, dtype=torch.float32, device = device)

         depot_ix = input['depot'].clone()              # [B, 1]
         self.depot_index = depot_ix
         self.uav_pos_coord = input['ugv loc'][torch.arange(self.batch_size).unsqueeze(1),depot_ix].squeeze(1)      # tensor [B, d]
         self.ugv_pos_coord = input['ugv loc'][torch.arange(self.batch_size).unsqueeze(1),depot_ix].squeeze(1)      # tensor [B, d]
         self.refuel_stop =   input['ugv loc'][torch.arange(self.batch_size).unsqueeze(1),depot_ix].squeeze(1)      # tensor [B, d]   
         self.refuel_stop_idx = depot_ix.clone()  
         self.uav_pos_indices = depot_ix.clone()                 # tensor [B,1] 
         
         self.uav_fuel = torch.full((self.batch_size,), self.uav_fuel_constant, dtype=torch.float32, device = device)                     # tensor [B]
         self.refuel_stop_time = torch.zeros(self.batch_size, device = device)      
         self.refuel_stop_time_uav = torch.zeros(self.batch_size, device = device)      
             
  
         self.mission_status = torch.zeros_like(self.unique_mission_points, device = device)[:,:,0]  # tensor [B, N]
         self.mission_time_elapsed = torch.zeros(self.batch_size, device = device)                   # tensor [B]
         self.uav_time_elapsed = torch.zeros(self.batch_size, device = device)                   # tensor [B]
         self.ugv_time_elapsed = torch.zeros(self.batch_size, device = device)                   # tensor [B]
         self.infeasibility = torch.zeros(self.batch_size, device = device).bool()                   # tensor [B]
         self.recharging_location = torch.zeros(self.batch_size, device = device).bool()             # tensor [B]
         
         self.prv_action = [None] * self.batch_size                                               #initialize
         
         
         
         self.done_completely = torch.zeros(self.batch_size, device = device)
         
         
         state = self.get_observation()
         
         return state
    
    
    
    
    def get_observation(self):
       return (self.uav_pos_indices, self.uav_fuel, self.mission_status, self.refuel_stop_idx )

            
        
    



    def feasible_action(self):
        
       batch_size = self.batch_size
       N = self.encoder_mission_space.shape[1]
       assert batch_size == self.uav_pos_coord.size(0)

       if self.ugv_mission_points.dim() == 2:
                 self.ugv_mission_points = self.ugv_mission_points.unsqueeze(0).repeat(batch_size, 1, 1)   # [B, N, d ]
                 
       if self.uav_mission_points.dim() == 2:
              self.uav_mission_points = self.uav_mission_points.unsqueeze(0).repeat(batch_size, 1, 1)  # [B, N, d ]


       #------- mission points that are already visited not need to be visited --------#
       
       

       mission_status = self.mission_status.to(device)          # [B, N_mission]
       batch_size = mission_status.shape[0]
       N_ugv = len( self.ugv_mission_points[0])               # number of UGV nodes
       feasible_region = (mission_status != 1).int()
       feasible_region[:, :N_ugv] = 1
       repeated_feasible_region = feasible_region  # [B, N_mission]


    
       #--------feasible region where uav can go and wil have feasible landing locs after going there -----------#

       # --- Compute distances ---
       action_positions = self.encoder_mission_space[:, :, :2]  # [B, N, 2]
       expanded_uav_pos = self.uav_pos_coord.unsqueeze(1).expand(-1, N, -1)
       dist_to_uav_nodes = self.calculate_distances(expanded_uav_pos, action_positions)  # [B, N]

       # --- Distances from mission nodes to all UGVs ---
       action_positions_expanded = action_positions.unsqueeze(2)  # [B, N, 1, 2]
       ugv_positions_expanded = self.ugv_mission_points.unsqueeze(1)  # [B, 1, N_ugv, 2]
       dist_to_all_ugv = self.calculate_distances(action_positions_expanded, ugv_positions_expanded)  # [B, N, N_ugv]

       # --- Compute times ---
       uav_to_action_time = dist_to_uav_nodes / self.uav_speed  # [B, N]
       action_to_ugv_time = dist_to_all_ugv / self.uav_speed    # [B, N, N_ugv]

       # --- UGV arrival time at all UGV nodes ---
       ugv_arrival_time_all = self.refuel_stop_time.unsqueeze(1).unsqueeze(2) + \
                            (self.cu_dis_matrix(self.refuel_stop, self.ugv_mission_points).unsqueeze(1) / self.ugv_speed)
       # Shape: [B, 1, N_ugv]

       # --- UAV arrival time at UGV nodes after completing mission ---
       uav_arrival_time_all = self.mission_time_elapsed.unsqueeze(1).unsqueeze(2) + \
                            uav_to_action_time.unsqueeze(2) + action_to_ugv_time
       # [B, N, N_ugv]

       # --- Waiting time per (mission node, UGV node) pair ---
       waiting_time_all = torch.clamp(ugv_arrival_time_all - uav_arrival_time_all, min=0.0)  # [B, N, N_ugv]

       # --- Total airborne time for all possible UGVs ---
       total_air_time_all = uav_to_action_time.unsqueeze(2) + action_to_ugv_time + waiting_time_all  # [B, N, N_ugv]

       # --- Check feasibility (any UGV satisfies the constraint) ---
       feasible_with_any_ugv = (total_air_time_all < self.uav_fuel.unsqueeze(1).unsqueeze(2))  # [B, N, N_ugv]
       return_feasible_region = feasible_with_any_ugv.any(dim=2).to(device)  # [B, N]

       rows_with_all_zeros = ~torch.any(return_feasible_region, dim=1)




       # -------- uav can wait but not by violating fuel limit -------- #

       # Time for UAV to reach each UGV node
       # self.uav_pos_coord: [B, 2]
       # self.ugv_mission_points: [B, N_ugv, 2]
       time_to_ugv_nodes_uav = self.calculate_distances(
            self.uav_pos_coord.unsqueeze(1), 
            self.ugv_mission_points
        ) / self.uav_speed  # Shape: [B, N_ugv]
       
       #print('time_to_ugv_nodes_uav --->', time_to_ugv_nodes_uav[index_of_interest])

       # Time for UGV to reach each UGV node from its current refuel stop
       # self.refuel_stop: [B, 2]
       time_to_ugv_nodes_ugv = self.cu_dis_matrix(
            self.refuel_stop, 
            self.ugv_mission_points
        ) / self.ugv_speed    # Shape: [B, N_ugv]
       
       uav_arrival_time = self.mission_time_elapsed.unsqueeze(1) + time_to_ugv_nodes_uav  # [B, N_ugv]
       ugv_arrival_time = self.refuel_stop_time.unsqueeze(1) + time_to_ugv_nodes_ugv      # [B, N_ugv]

       waiting_time = torch.clamp(ugv_arrival_time - uav_arrival_time, min=0.0)  # [B, N_ugv]

       

       # --- Total airborne time until recharge ---
       air_time_until_recharge = time_to_ugv_nodes_uav + waiting_time  # [B, N_ugv]

       # --- Check fuel feasibility ---
       rendezvous_waiting_mask = (air_time_until_recharge < self.uav_fuel.unsqueeze(1)).to(device)

       # --- Pad with ones for UAV mission points ---
       uav_mask_padding = torch.ones(
            (rendezvous_waiting_mask.shape[0], len(self.uav_mission_points[0])),
            device=device,
            dtype=rendezvous_waiting_mask.dtype
        )

       # --- Concatenate to match [B, N_ugv + N_uav] ---
       rendezvous_waiting_mask = torch.cat(
            (rendezvous_waiting_mask, uav_mask_padding), dim=1
        )  # → [B, N_total]

    


       # -------- create mask -----------#
                                                                     #
       feasible_mask = torch.zeros(batch_size, self.encoder_mission_space.shape[1], device = device)
       feasible_mask[:,:] =  return_feasible_region* repeated_feasible_region * rendezvous_waiting_mask
       


       #print('\n return feasible mask --->', return_feasible_region[index_of_interest])
       #print('repeated_feasible_regio mask --->', repeated_feasible_region[index_of_interest])
       #print('rendezvous waiting mask  --->', rendezvous_waiting_mask[index_of_interest])
       #print(' feasible mask --->', feasible_mask[index_of_interest], '\n')


       rows_with_all_zeros = ~torch.any(feasible_mask, dim=1)
       rows_with_all_zeros_indices = torch.where(rows_with_all_zeros)[0]
       if torch.any(rows_with_all_zeros):
           feasible_mask[rows_with_all_zeros_indices, :] = 0
           feasible_mask[rows_with_all_zeros_indices, self.prv_action[rows_with_all_zeros_indices]] = 1
       
         
       a = feasible_mask.clone()
       
       
       assert torch.any(feasible_mask, dim=1).all(), "A row with all zeros (False) found in the mask."
       
       ugv_indices = torch.where(~self.recharging_location)[0].to(device)
       ugv_slice = slice(0, len(self.ugv_mission_points[0]))
       feasible_mask[ugv_indices, ugv_slice] = torch.zeros(batch_size, len(self.ugv_mission_points[0]), device = device)[ugv_indices]   # makes sure after recharging visiting action is performed
       
       #print(' feasible mask --->', feasible_mask[index_of_interest], '\n')
       
       rows_with_all_zeros = ~torch.any(feasible_mask, dim=1)
       if torch.any(rows_with_all_zeros):
             rows_with_all_zeros_indices = torch.where(rows_with_all_zeros)[0]
             feasible_mask[rows_with_all_zeros_indices] = a.clone()[rows_with_all_zeros_indices]
                  
       # when all points are visited UAV needs to return to UGV       
       exceeded_indices = torch.where(torch.all(self.mission_status == 1, dim=1))[0]
       if exceeded_indices.numel() > 0:
         feasible_mask[exceeded_indices, :] = 0 
         feasible_mask[exceeded_indices, ugv_slice] = (return_feasible_region[exceeded_indices, ugv_slice].float() * rendezvous_waiting_mask[exceeded_indices, ugv_slice] )

       
         
       #print(' feasible mask --->', feasible_mask[index_of_interest], '\n')

       rows_with_all_zeros = ~torch.any(feasible_mask, dim=1)
       rows_with_all_zeros_indices = torch.where(rows_with_all_zeros)[0]
       if torch.any(rows_with_all_zeros):
           feasible_mask[rows_with_all_zeros_indices, :] = 0
           feasible_mask[rows_with_all_zeros_indices, self.prv_action[rows_with_all_zeros_indices]] = 1

       #print(' feasible mask --->', feasible_mask[index_of_interest], '\n')
       
       assert torch.any(feasible_mask, dim=1).all(), "A row with all zeros (False) found in the mask."


       exceeded_indices = torch.where(self.done_completely == 1)[0]
       if exceeded_indices.numel() > 0:
         feasible_mask[exceeded_indices, :] = 0
         feasible_mask[exceeded_indices, self.prv_action[exceeded_indices]] = 1
    
       assert torch.any(feasible_mask, dim=1).all(), "A row with all zeros (False) found in the mask."
       
       
       #print('feasible mask --->', feasible_mask[index_of_interest], '\n')

       return feasible_mask










        
       
        
    

































