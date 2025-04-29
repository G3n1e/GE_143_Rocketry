import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import inv

# === CONFIGURATION ===
SERIAL_PORT = '/dev/ttyUSB'  
BAUD_RATE = 9600
TIMEOUT = 2

# === SETUP SERIAL ===
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
time.sleep(2)

# KALMAN FILTER INITIALIZATION 

# Initial state (altitude and vertical velocity)
x = np.array([[0.0],   # altitude
              [0.0]])  # vertical velocity

# Initial uncertainty
P = np.eye(2) * 500

# Next state function (constant velocity model)
dt = 1.0  # Assume ~1 second between measurements (we can improve this)
F = np.array([[1, dt],
              [0, 1]])

# Measurement function (we only measure altitude)
H = np.array([[1, 0]])

# Measurement uncertainty
R = np.array([[5]])   # Measurement noise covariance

# Process noise
Q = np.array([[0.01, 0],
              [0, 0.01]])

# === DATA STORAGE ===
times = []
altitudes_raw = []
altitudes_filtered = []

print("Reading data... Press Ctrl+C to stop.")

try:
    last_time = None
    while True:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue
        
        parts = line.split()
        
        if len(parts) != 10:
            print(f"Unexpected format: {line}")
            continue
        
        try:
            temperature = float(parts[0])
            pressure = float(parts[1])
            altitude = float(parts[2])
            ax = int(parts[3])
            ay = int(parts[4])
            az = int(parts[5])
            gx = int(parts[6])
            gy = int(parts[7])
            gz = int(parts[8])
            timestamp = int(parts[9])

            # Update dt based on timestamps
            if last_time is not None:
                dt = (timestamp - last_time) / 1000.0
                F = np.array([[1, dt],
                              [0, 1]])
            last_time = timestamp

            # 1. Prediction
            x = F @ x
            P = F @ P @ F.T + Q

            # 2. Measurement Update
            z = np.array([[altitude]])
            y = z - H @ x
            S = H @ P @ H.T + R
            K = P @ H.T @ inv(S)

            x = x + K @ y
            P = (np.eye(2) - K @ H) @ P

            # Save data
            times.append(timestamp / 1000.0)
            altitudes_raw.append(altitude)
            altitudes_filtered.append(x[0,0])

            print(f"Time: {timestamp} ms, Altitude: {altitude:.2f} m (Filtered: {x[0,0]:.2f})")
        
        except ValueError as e:
            print(f"Parse error: {line} | {e}")

except KeyboardInterrupt:
    print("Data collection stopped.")

finally:
    ser.close()

# === PLOT RESULTS ===
plt.figure(figsize=(10,6))
plt.plot(times, altitudes_raw, label='Raw Altitude', alpha=0.5)
plt.plot(times, altitudes_filtered, label='Kalman Filtered Altitude', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.title('Altitude over Time (Raw vs Filtered)')
plt.legend()
plt.grid()
plt.show()