import time
import numpy as np
from multiprocessing.connection import Connection
import threading
from DDS.Cloud2DPublisher import Cloud2DPublisher

exit_event = threading.Event()

class Delta:

    MAX_ACCEL = 14000 # mm/s^2
    MAX_VEL = 200_000.0/60.0 # mm/s
    MIN_LIMIT = [-400,-400,-850]
    MAX_LIMIT = [400,400,-350]
    HOME_POSITION = [0, 0, -500]

    def __init__(self):
        self.MIN_LIMIT = Delta.MIN_LIMIT
        self.MAX_LIMIT = Delta.MAX_LIMIT
        self.POSITION = np.array(self.HOME_POSITION, dtype=np.float64)
        self.VELOCITY = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        self.pos_queue = []

        self.lock = threading.Lock()
        self.pos_publisher = Cloud2DPublisher("DELTA_POSITION")
        self.target_point_publisher = Cloud2DPublisher("TARGET_POINTS")

    def update(self):
        dt = 0.01
        while not exit_event.is_set():
            with self.lock:
                if len(self.pos_queue) == 0:
                    self.VELOCITY = np.array([0.0, 0.0, 0.0], dtype=np.float64)
                    continue

                target_pos, vel = self.pos_queue[0]
                self.target_point_publisher.publish(np.array([target_pos[:2]]))
                if np.linalg.norm(self.POSITION - target_pos) < 1e-1:
                    self.VELOCITY = np.array([0.0, 0.0, 0.0], dtype=np.float64)
                    self.pos_queue.pop(0)
                    continue
                else:
                    # Calculate the direction and distance to move
                    direction = target_pos - self.POSITION
                    distance = np.linalg.norm(direction)
                    if distance < 1e-3:
                        continue
                    
                    # Normalize the direction vector
                    direction /= distance

                    travel_time = Delta.calculate_travel_time(self.POSITION, target_pos, vel=vel)
                    velocity = distance / travel_time

                    self.VELOCITY = direction * velocity
                    
                    self.POSITION += self.VELOCITY * dt
                    self.pos_publisher.publish(np.array([self.POSITION], dtype=np.float64))

            time.sleep(dt)


                    

    def get_position(self):
        return self.POSITION

    def get_velocity(self):
        return self.VELOCITY

    def add_position_to_queue(self, position, feedrate=200_000):
        with self.lock:
            self.pos_queue.append((position, feedrate))

    def is_in_range(pos):
        return all([Delta.MIN_LIMIT[i] <= pos[i] <= Delta.MAX_LIMIT[i] for i in range(3)])

    def calculate_travel_time_from_position(self, pos, feedrate=1000):
        if not Delta.is_in_range(pos):
            #print(f"Position {pos} is out of range")
            return np.inf
        curr_pos = self.get_position()
        return Delta.calculate_travel_time(curr_pos, pos, vel=feedrate)
    
    def calculate_travel_time(pos1, pos2, vel=1000):
        mm_per_sec = vel
        curr_pos = pos1
        diff = np.array(pos2) - curr_pos
        distance = np.linalg.norm(diff)
        # Distance covered when accelerating to max velocity
        accel_distance = 0.5 * Delta.MAX_ACCEL * (mm_per_sec / Delta.MAX_ACCEL)**2

        # Can't reach max velocity - triangle
        if 2 * accel_distance > distance:
            time = 2 * np.sqrt(distance / Delta.MAX_ACCEL)
        else:
        # Can reach max velocity - trapezoid
            constant_vel_distance = distance - 2 * accel_distance
            time = (2 * mm_per_sec / Delta.MAX_ACCEL # accel and decel time
                    + constant_vel_distance / mm_per_sec) # constant velocity time

        return time

    def enable_path_blending(self):
        pass

    def disable_path_blending(self):
        pass

    def enable_vacuum(self):
        pass
        #print("Vacuum enabled")

    def disable_vacuum(self):
        pass
        #print("Vacuum disabled")

    def is_in_position(self):
        with self.lock:
            if len(self.pos_queue) == 0:
                return True
            target_pos, _ = self.pos_queue[0]
            #print(self.POSITION, target_pos)
            return np.linalg.norm(self.POSITION - target_pos) < 1e-3




    def delta_process(pipe_parent : Connection, pipe_child : Connection):
        print("STARTED DELTA PROCESS")
        delta = Delta()

        delta_thread = threading.Thread(target=delta.update)
        delta_thread.start()

        pipe_child.send(("READY", None))

        while True:
            try:
                if pipe_parent.poll(timeout=None):
                    msg = pipe_parent.recv()
            except KeyboardInterrupt:
                break
            if msg is not None:
                if msg[0] == "EXIT":
                    print("EXITING DELTA PROCESS")
                    exit_event.set()
                    delta_thread.join()
                    break
                elif msg[0] == "MOVE_ROBOT":
                    #print(f"RECEIVED MOVE ROBOT TO {msg[1]}")
                    delta.add_position_to_queue(msg[1], feedrate=msg[2])
                    #delta.add_position_with_interpolation(msg[1], samples=4)
                elif msg[0] == "GET_POSITION":
                    pipe_parent.send(("POSITION", delta.get_position()))
                elif msg[0] == "GET_VELOCITY":
                    pipe_parent.send(("VELOCITY", delta.get_velocity()))
                elif msg[0] == "GET_TRAVEL_TIME":
                    pipe_parent.send(("TRAVEL_TIME", delta.calculate_travel_time_from_position(msg[1])))
                elif msg[0] == "GET_TRAVEL_TIMES":
                    pipe_parent.send(("TRAVEL_TIMES", [delta.calculate_travel_time_from_position(pos, msg[2]) for pos in msg[1]]))
                elif msg[0] == "ENABLE_PATH_BLENDING":
                    delta.enable_path_blending()
                elif msg[0] == "DISABLE_PATH_BLENDING":
                    delta.disable_path_blending()
                elif msg[0] == "ENABLE_VACUUM":
                    #print("RECEIVED ENABLE VACUUM")
                    delta.enable_vacuum()
                elif msg[0] == "DISABLE_VACUUM":
                    #print("RECEIVED DISABLE VACUUM")
                    delta.disable_vacuum()
                elif msg[0] == "IS_IN_POSITION":
                    pipe_parent.send(("IS_IN_POSITION", delta.is_in_position()))
                elif msg[0] == "ABORT":
                    pass
                else:
                    print(f"Unknown message: {msg}")

        print("EXITED DELTA PROCESS")
