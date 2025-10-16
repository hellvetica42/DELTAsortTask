import time
import numpy as np
from filterpy.kalman import KalmanFilter
class ConveyorTracker:
    def __init__(self, dt, measurement_std, process_std, id):
        """
        Initialize Kalman Filter for tracking objects on conveyor
        dt: time step between measurements/predictions
        measurement_std: standard deviation of measurement noise
        process_std: standard deviation of process noise
        """
        self.id = id
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        
        # State vector [x, v_x, y, v_y]
        # x and y are position, v_x and v_y are velocities
        self.kf.x = np.zeros(4)
        
        # State transition matrix (physics model)
        self.kf.F = np.array([
            [1, dt, 0, 0],   # x = x + v_x*dt
            [0, 1, 0, 0],    # v_x = v_x
            [0, 0, 1, dt],   # y = y + v_y*dt
            [0, 0, 0, 1]     # v_y = v_y
        ])
        
        # Measurement matrix (we only measure position, not velocity)
        self.kf.H = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0]
        ])
        
        # Measurement noise
        self.kf.R = np.eye(2) * measurement_std**2
        #self.kf.R = np.array([
                    #[measurement_std_x**2, 0],
                    #[0, measurement_std_y**2]
                #])
        
        # Process noise
        q = process_std**2
        self.kf.Q = np.array([
            [q*dt**4/4, q*dt**3/2, 0, 0],
            [q*dt**3/2, q*dt**2, 0, 0],
            [0, 0, q*dt**4/4, q*dt**3/2],
            [0, 0, q*dt**3/2, q*dt**2]
        ])
        
        # Initial state covariance
        self.kf.P *= 1000

        self.N_measurements = 0
        self.last_seen = time.time()
        self.visible = True
    
    def update(self, measurement):
        """Update state with new measurement"""
        self.kf.predict()
        self.kf.update(measurement)
        if measurement is not None:
            self.N_measurements += 1
            self.last_seen = time.time()
            self.visible = True
        else:
            self.visible = False
        
    def get_current_state(self):
        """Return current position and velocity"""
        return {
            'position': np.array((self.kf.x[0], self.kf.x[2])),
            'velocity': np.array((self.kf.x[1], self.kf.x[3])),
            'visible': self.visible
        }
    
    def predict_future_position(self, time_ahead):
        """Predict position after time_ahead seconds"""
        x = self.kf.x[0] + self.kf.x[1] * time_ahead
        y = self.kf.x[2] + self.kf.x[3] * time_ahead
        return np.array((x, y))

    def time_until_position(self, position):
        """Predict time until the object reaches a given position"""
        #x, y, z = position
        #time_x = (x - self.kf.x[0]) / self.kf.x[1]
        #time_y = (y - self.kf.x[2]) / self.kf.x[3]
        #return (time_x + time_y) / 2
        distance = np.linalg.norm(position[:2] - self.get_current_state()['position'])
        velocity = np.linalg.norm(self.get_current_state()['velocity'])
        return distance / velocity

    def get_x_when_y_is(self, ypos):
        """Predict x position when y position is ypos"""
        # (ypos - y0) / v_y = (x - x0) / v_x
        return self.kf.x[0] + self.kf.x[1] * (ypos - self.kf.x[2]) / self.kf.x[3]

    def get_velocity(self):
        """Return current velocity"""
        return np.array([self.kf.x[1], self.kf.x[3]])

