import zmq
import msgpack
from dataclasses import dataclass
from typing import List
import time
 
@dataclass
class SensorDataCart:
    timestamp: float
    id_board: int
    id_sensor: int
    id_deployment: int
    x: float
    y: float
    z: float
    velocity: float
    cov_mat_xx: float
    cov_mat_yy: float
    cov_mat_zz: float
    cov_mat_vv: float
    snr: float
 
def decode_sensor_data(bundle: bytes) -> List[SensorDataCart]:
    raw_list = msgpack.unpackb(bundle, raw=False)
    return [SensorDataCart(**item) for item in raw_list]
 
def main():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://192.168.232.50:35001")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
 
    sensor_ids = set()
    while True:
        bundle = socket.recv()
        sensors = decode_sensor_data(bundle)
        for s in sensors:
            sensor_ids.add(s.id_sensor)
        print("All unique Sensor IDs seen so far:", sensor_ids)
            
        # if not hasattr(main, "last_time"):
        #     main.last_time = time.time()
        #     main.count = 0

        # main.count += len(sensors)
        # current_time = time.time()
        # elapsed = current_time - main.last_time

        # if elapsed >= 1.0:
        #     print(f"Estimated point clouds per second: {main.count / elapsed:.2f}")
        #     main.last_time = current_time
        #     main.count = 0

if __name__ == "__main__":
    print("Program started")
    main()