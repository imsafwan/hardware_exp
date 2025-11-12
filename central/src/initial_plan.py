import socket, json, yaml, os, time, logging
import datetime



# ensure log directory exists
LOG_DIR = "logs"
os.makedirs(LOG_DIR, exist_ok=True)

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_FILE = os.path.join(LOG_DIR, f"initial_plan_{timestamp}.log")

logging.basicConfig(
    filename=LOG_FILE,   # now stored inside logs/
    level=logging.INFO,  # DEBUG, INFO, WARNING, ERROR, CRITICAL
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)

logger = logging.getLogger(__name__)


#logger = logging.getLogger(__name__)

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



UAV_IP   = "192.168.0.161"
UAV_PORT = 5007
UGV_PORT = 5008

sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Listen for ACKs
sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx.bind(("0.0.0.0", 6000))   # pick a free port for ACK reception
sock_rx.settimeout(0.5)

# Load UAV plan
with open(os.path.join("output", "uav_actions.yaml"), "r") as f:
    plan_uav_yaml = yaml.safe_load(f)
    plan_uav = {
        'msg_type': 'UAV_PLAN',
        'plan_id': 1,
        'new_plan': plan_uav_yaml['sortie 1']
    }

# Load UGV plan
with open(os.path.join("output", "ugv_actions.yaml"), "r") as f:
    plan_ugv_yaml = yaml.safe_load(f)
    plan_ugv = {
        'msg_type': 'UGV_PLAN',
        'plan_id': 1,
        'new_plan': plan_ugv_yaml['sortie 1']
    }

def send_with_ack(msg, addr, plan_id):
    msg = json.dumps(msg).encode()
    ack_received = False
    deadline = time.time() + 30  # try for 3s
    while not ack_received and time.time() < deadline:
        sock_tx.sendto(msg, addr)
        try:
            data, _ = sock_rx.recvfrom(1024)
            ack = json.loads(data.decode())
            if ack.get("msg_type") == "ACK" and ack.get("plan_id") == plan_id:
                logger_print(f"ACK received for plan {plan_id}")
                ack_received = True
                time.sleep(0.5)  # wait a bit before next send
        except socket.timeout:
            logger_print("No ACK yet, retrying...")

# Send UAV plan with ack
logger_print('UAV:')
send_with_ack(plan_uav, (UAV_IP, UAV_PORT), plan_uav["plan_id"])

logger_print('UGV:')
# Send UGV plan with ack
send_with_ack(plan_ugv, (UAV_IP, UGV_PORT), plan_ugv["plan_id"])
