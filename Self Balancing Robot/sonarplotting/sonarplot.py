import serial
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Set up serial connection
SERIAL_PORT = 'COM3'  # Change to match your port
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Initialize plot
fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("Live XY Plot from Polar Coordinates")

x_vals = []
y_vals = []

def update(frame):
    global x_vals, y_vals
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            angle_deg, distance_cm = map(float, line.split(","))
            angle_rad = np.radians(angle_deg)
            distance_m = distance_cm / 100.0

            # Convert polar to Cartesian
            x = distance_m * np.cos(angle_rad)
            y = distance_m * np.sin(angle_rad)

            x_vals.append(x)
            y_vals.append(y)

            ax.clear()
            ax.set_xlim(-2, 2)
            ax.set_ylim(-2, 2)
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_title("Live XY Plot from Polar Coordinates")
            ax.scatter(x_vals, y_vals, color='blue')

        except Exception as e:
            print("Error:", e)

# Animate plot
ani = FuncAnimation(fig, update, interval=100)
plt.tight_layout()
plt.show()
