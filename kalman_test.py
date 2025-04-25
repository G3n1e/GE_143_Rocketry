import serial
import re
import numpy as np
import time

# ---------- Kalman Filter Setup ----------
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

# ---------- Serial Setup ----------
port = 'COM4'  # Update this to match your system (e.g., COM3 on Windows)
baudrate = 9600
ser = serial.Serial(port, baudrate, timeout=1)
time.sleep(2)  # Give time for Arduino to reset

# ---------- Filters ----------
temp_filter = KalmanFilter()
pressure_filter = KalmanFilter()

# ---------- Main Loop ----------
print("Reading and filtering data... Press Ctrl+C to stop.\n")

try:
    while True:
        line = ser.readline().decode('utf-8').strip()

        match = re.match(r"Temperature: ([\d\.\-]+) °C, Pressure: ([\d\.\-]+) hPa", line)
        if match:
            raw_temp = float(match.group(1))
            raw_pressure = float(match.group(2))

            filtered_temp = temp_filter.update(raw_temp)
            filtered_pressure = pressure_filter.update(raw_pressure)

            print(f"Raw: T={raw_temp:.2f}°C, P={raw_pressure:.2f} hPa | Filtered: T={filtered_temp:.2f}°C, P={filtered_pressure:.2f} hPa")
except KeyboardInterrupt:
    print("\nStopped reading.")
    ser.close()
