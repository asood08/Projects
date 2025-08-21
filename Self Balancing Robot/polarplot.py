import serial
import matplotlib.pyplot as plt
import numpy as np

# --- Serial Configuration ---
SERIAL_PORT = 'COM3'  # Change to your port (e.g., '/dev/ttyUSB0' on Linux/Mac)
BAUD_RATE = 9600      # Match with your microcontroller's baud rate

# Initialize Serial
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# --- Plot Configuration ---
plt.ion()  # Turn on interactive mode
fig = plt.figure()
ax = fig.add_subplot(111, polar=True)
sc = ax.scatter([], [], c='r', s=10)

# Set plot limits
ax.set_ylim(0, 50)  # Adjust based on max distance in cm or meters
ax.set_theta_zero_location("N")  # 0 degrees at the top
ax.set_theta_direction(-1)       # Clockwise

angles = []
distances = []

def update_plot():
    sc.set_offsets(np.c_[angles, distances])
    plt.draw()
    plt.pause(0.01)

try:
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            
            try:
                # Expecting data in the format: angle,distance (e.g., "45,100")
                angle_str, distance_str = line.split(',')
                angle = np.radians(float(angle_str))  # Convert to radians for polar plot
                distance = float(distance_str)

                angles.append(angle)
                distances.append(distance)

                update_plot()

            except ValueError:
                print(f"Invalid data: {line}")

except KeyboardInterrupt:
    print("Stopped by user.")
    ser.close()
