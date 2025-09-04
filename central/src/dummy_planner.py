import socket, json

UAV_IP   = "192.168.0.161"   # replace with UAV’s IP
UAV_PORT = 5007           # replace with UAV’s listening port
UGV_PORT = 5008 

sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

plan_uav = {'msg_type': 'UAV_PLAN', 'plan_id': 1, 'new_plan': [{'type': 'takeoff_from_UGV', 'ins_id': 'take_off_1', 'end_time': 7}, {'type': 'move_to_location', 'ins_id': 'survey_building', 'location': {'lat': 41.87008, 'lon': -87.6504}, 'end_time': 25.277191717918285}, {'type': 'move_to_location', 'ins_id': 'survey_industrial', 'location': {'lat': 41.86985, 'lon': -87.6503}, 'end_time': 65.39938113996548}, {'type': 'move_to_location', 'ins_id': 'survey_residential', 'location': {'lat': 41.87005, 'lon': -87.6501}, 'end_time': 106.78395327848907}, {'type': 'move_to_location', 'ins_id': 'return_to_ugv_f', 'location': {'lat': 41.87002, 'lon': -87.65012}, 'end_time': 112.34260422606508}, {'type': 'land_on_UGV', 'ins_id': 'land_1', 'end_time': 137.34260422606508}]}


plan_ugv = {'msg_type': 'UGV_PLAN', 'plan_id': 1, 'new_plan': [{'type': 'allow_take_off_from_UGV', 'location': {'lat': 41.870046, 'lon': -87.650351}, 'ins_id': 'take_off_1', 'start_time': 0.0, 'end_time': 7}, {'type': 'move_to_location', 'ins_id': 'move_to_B', 'location': {'lat': 41.870007, 'lon': -87.650315}, 'start_time': 17.0, 'end_time': 22.847002422890125}, {'type': 'move_to_location', 'ins_id': 'move_to_E', 'location': {'lat': 41.86996, 'lon': -87.65018}, 'start_time': 22.84700242289012, 'end_time': 36.55777925334982}, {'type': 'move_to_location', 'ins_id': 'move_to_F', 'location': {'lat': 41.87002, 'lon': -87.65012}, 'start_time': 36.55777925334982, 'end_time': 45.80033163512124}, {'type': 'allow_land_on_UGV', 'location': {'lat': 41.87002, 'lon': -87.65012}, 'ins_id': 'land_1', 'start_time': 112.34260422606508, 'end_time': 137.34260422606508}]}


# plan = {
#     "msg_type": "PLAN_UPDATE",
#     "plan_id": 2,  # increment this for every new plan
#     "new_plan": [
        
#         {"type": "move_to_location", "ins_id": "move_new",
#          "location": {"lat": 41.870017, "lon": -87.650253}},
#         {"type": "move_to_location", "ins_id": "move_4",
#          "location": {"lat": 41.870021, "lon": -87.650211}},
#         {"type": "land_on_UGV", "ins_id": "land_2"},
#     ]
# }

msg_uav= json.dumps(plan_uav).encode()
sock_tx.sendto(msg_uav, (UAV_IP, UAV_PORT))

msg_ugv = json.dumps(plan_ugv).encode()
sock_tx.sendto(msg_ugv, (UAV_IP, UGV_PORT))

print("✅ Sent plan update")


