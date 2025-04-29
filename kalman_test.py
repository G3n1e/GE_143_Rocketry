import serial
import math as h
import numpy as np
import time

class KalmanFilter:
    def __init__(self, process_noise=0.01, measurement_noise=4.0, estimate_error=1.0, initial_value=0):
        self.x = initial_value  # Initial estimate
        self.P = estimate_error  # Initial error covariance
        self.Q = process_noise  # Process noise
        self.R = measurement_noise  # Measurement noise

    def update(self, measurement):
        # Prediction update
        self.P = self.P + self.Q

        # Measurement update
        K = self.P / (self.P + self.R)  # Kalman gain
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * self.P

        return self.x
    
def get_altitude(altitude, pressure, temperature):
    # Constants
    p0 = 1013.25                    # Standard atmospheric pressure at sea level in hPa
    temp_k = temperature + 273.15   # Convert to Kelvin
    g = 9.80665                     # Gravity (m/s^2)
    R = 287.05                      # Specific gas constant for dry air (J/(kg·K))

    # Barometric formula assuming constant temperature
    altitude.append((temp_k / g) * R * (1 - (pressure / p0) ** (R * g / (R * temp_k))))
    
    return altitude

def get_orientation(orientation, gyroscope, delta_t):
    tmp = orientation[-1]
    for i in range(len(orientation)):
        tmp[i] += (gyroscope[1][i] - gyroscope[0][i]) * delta_t
    orientation.append(tmp)

    return orientation

def orient_acceleration(acceleration, acceleration_data, orientation):
    # Rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, h.cos(orientation[0]), -h.sin(orientation[0])],
        [0, h.sin(orientation[0]), h.cos(orientation[0])]
    ])

    Ry = np.array([
        [h.cos(orientation[1]), 0, h.sin(orientation[1])],
        [0, 1, 0],
        [-h.sin(orientation[1]), 0, h.cos(orientation[1])]
    ])

    Rz = np.array([
        [h.cos(orientation[2]), -h.sin(orientation[2]), 0],
        [h.sin(orientation[2]), h.cos(orientation[2]), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix: R = Rz * Ry * Rx
    R = Rz @ Ry @ Rx

    # Apply rotation
    acceleration.append(R @ np.array(acceleration_data))

    return acceleration

def integrate_velocity(prev_velocity, acceleration, delta_t):
    return [prev_velocity + (acceleration[-1] - acceleration[-2]) * delta_t for i in range(3)]

def integrate_position(prev_position, velocity, delta_t):
    return [prev_position + (velocity[-1] - velocity[-2]) * delta_t for i in range(3)]

def differentiate_velocity(altitude, delta_t):
    return (altitude[-1]-altitude[-2])*delta_t

# ---------- Global Data Variable ----------
SCALE_THRESHOLD = 0.5

raw_data = [] * 8
filtered_data = [KalmanFilter()] * 8
gyroscope = [[0, 0, 0]]
altitude = [0]
position = [[0, 0, 0]]
velocity = [[0, 0, 0]]
acceleration = [[0, 0, 0]]
orientation = [[0, 0, 0]]

# ---------- Setup ----------
# port = 'COM4' # for windows computers
port = '/dev/ttyUSB0' # for linux machines /dev/ttyUSB4' 
baudrate = 9600
ser = serial.Serial(port, baudrate, timeout=1)
time.sleep(2)

START_PHRASE = "BEGIN_TRANSMISSION"
print(f"Waiting for '{START_PHRASE}' to start...\n")

while True:
    try:
        line = ser.readline().decode('utf-8').strip()
        if line == START_PHRASE:
            break
    except Exception as e:
        print(f"Serial read error while waiting: {e}")

last_time = time.time()

# ---------- Main Loop ----------
print("Reading and filtering data... Press Ctrl+C to stop.\n")

try:
    last_line = ""

    while True:
        current_time = time.time()
        delta_t = current_time - last_time

        line = ser.readline().decode('utf-8').strip()

        # Ignore if same as previous
        if line == last_line:
            continue

        # Try to parse floats
        parts = line.split()
        if len(parts) != 8:
            print(f"Skipping invalid line: {line}")
            continue

        try:
            raw_data = list(map(float, parts))

            # Kalman Filter
            for i in range(len(raw_data)):
                filtered_data[i] = filtered_data[i].update(raw_data[i])

            # Update direct measurements
            gyroscope.append(filtered_data[5:])
            altitude = get_altitude(altitude, filtered_data[0], filtered_data[1])
            orientation = get_orientation(orientation, gyroscope[-2:], delta_t)
            acceleration = orient_acceleration(acceleration, filtered_data[2:5], orientation[-1])

            # Calculate indirect measurements
            acc_tmp_velocity = integrate_velocity(velocity[-1], acceleration, delta_t)
            acc_tmp_position = integrate_position(position[-1], [velocity[-1], acc_tmp_velocity], delta_t)

            bmp_tmp_velocity = differentiate_velocity(altitude, delta_t)

            # Scale indirect measurements
            pos_mag = h.sqrt(sum(acc_tmp_position[i] - position[-1][i] for i in range(3)))
            vel_mag = h.sqrt(sum(acc_tmp_velocity[i] - velocity[-1][i] for i in range(3)))

            if pos_mag > 10:
                position_scale_weight = acc_tmp_position[-1][2] / pos_mag
                position_scalar = (altitude[-1] - altitude[-2]) * position_scale_weight + (acc_tmp_position[2] - position[-1][2]) * (1 - position_scale_weight)
                position.append([pos * position_scalar for pos in acc_tmp_position])
            else:
                position.append(acc_tmp_position)

            if vel_mag > 10:
                velocity_scale_weight = acc_tmp_velocity[-1][2] / vel_mag
                velocity_scalar = (bmp_tmp_velocity - velocity[-1][2]) * velocity_scale_weight + (acc_tmp_velocity[2] - velocity[-1][2]) * (1 - velocity_scale_weight)
                velocity.append([vel * velocity_scalar for vel in acc_tmp_velocity])
            else:
                velocity.append(acc_tmp_velocity)

            last_time = current_time

            print(position[-1])
            
        except ValueError:
            print(f"Skipping non-numeric line: {line}")
            continue

except KeyboardInterrupt:
    print("\nStopped reading.")
    ser.close()