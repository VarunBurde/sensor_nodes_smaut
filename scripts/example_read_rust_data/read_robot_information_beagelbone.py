from dataclasses import dataclass
import zmq
import msgpack
import time
 
@dataclass
class Info:
    timestamp: float
    id: str
    serial_number: str
 
@dataclass
class PowerManagement:
    batery_voltage: float
 
@dataclass
class NavigationGNSS:
    lon: float
    lat: float
    alt: float
    accuracy: float
    num_sats: int
    pdop: float
    hdop: float
    vdop: float
    geoid_height: float
 
@dataclass
class RobotStatus:
    autonomous_state: bool
    x_map: float
    y_map: float
    speed: float
    heading: float
    wheel_angle: float
    speed_left: float
    speed_right: float
    rtk_fix: int
 
@dataclass
class RobotDynamics:
    speed_change: float
    heading_change: float
    wheel_angle_change: float
 
@dataclass
class RobotModel:
    info: Info
    robot_status: RobotStatus
    navigation_gnss: NavigationGNSS
    power_management: PowerManagement
    emergency_active: bool
    dynamics: RobotDynamics
   
 
def parse_robot_model(data: dict) -> RobotModel:
    return RobotModel(
        info=Info(**data["info"]),
        robot_status=RobotStatus(**data["robot_status"]),
        navigation_gnss=NavigationGNSS(**data["navigation_gnss"]),
        power_management=PowerManagement(**data["power_management"]),
        emergency_active=data["emergency_active"],
        dynamics=RobotDynamics(**data["dynamics"]),
    )
 
def receive_robot_model(endpoint="tcp://192.168.232.50:35000"):
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(endpoint)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
 
    print(f"Listening on {endpoint}...")
    t = time.time()
    dt = 1.0
    n = 0
    while True:
        msg = socket.recv()
        try:
            # Očekáváme MessagePack serializaci
            unpacked = msgpack.unpackb(msg, raw=False)
            robot = parse_robot_model(unpacked)
            n += 1
            if time.time() - t > dt:
                print(robot)
                print(f"Received {n} messages in the last {dt} seconds.")
                t = time.time()
                n = 0
        except Exception as e:
            print(f"Failed to decode message: {e}")
 
if __name__ == "__main__":
    receive_robot_model()