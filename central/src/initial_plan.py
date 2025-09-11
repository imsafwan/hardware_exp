import socket, json, yaml, os

UAV_IP   = "192.168.0.161"   # replace with UAV’s IP
UAV_PORT = 5007              # UAV listening port
UGV_PORT = 5008              # UGV listening port

sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# Load UAV plan from YAML
with open(os.path.join("output", "uav_actions.yaml"), "r") as f:
    plan_uav_yaml = yaml.safe_load(f)
    plan_uav = {
    'msg_type': 'UAV_PLAN',
    'plan_id': 1,
    'new_plan': plan_uav_yaml['sortie 1']
}

# Load UGV plan from YAML
with open(os.path.join("output", "ugv_actions.yaml"), "r") as f:
    plan_ugv_yaml = yaml.safe_load(f)
    plan_ugv = {
    'msg_type': 'UGV_PLAN',
    'plan_id': 1,
    'new_plan': plan_ugv_yaml['sortie 1']
}
    



# Encode and send UAV plan
msg_uav = json.dumps(plan_uav).encode()
sock_tx.sendto(msg_uav, (UAV_IP, UAV_PORT))

# Encode and send UGV plan
msg_ugv = json.dumps(plan_ugv).encode()
sock_tx.sendto(msg_ugv, (UAV_IP, UGV_PORT))

print("Sent plan update from YAML files")
