import asyncio
import customtkinter as ctk
from bleak import BleakClient, BleakScanner
import threading
from pynput import keyboard
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import sys
import io

laptop_master_service_uuid = "00000000-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_data_receive_uuid = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_send_characteristic_uuid = "00000002-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_sonar_uuid = "00000003-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_kpoutput_uuid = "00000004-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_kioutput_uuid = "00000005-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_kdoutput_uuid = "00000006-5EC4-4083-81CD-A10B8D5CF6EC"



# The name of the Arduino device
arduino_device_name = "C5-BLE"  # Replace with your Arduino device name

global client, ble_loop
client = None
ble_loop = None

# Throttle BLE messages
last_sent_time = 0
send_interval = 0.0001  # Minimum time between BLE sends (1ms)

# Angle Data for Live Plotting
angles = []  # List to store received angles
stepperAngles = [] 
robot_x_array = []
robot_y_array = []
robot_yaw_array = []
distance_sensed_x_array = []
distance_sensed_y_array = []
time_stamps = []  # List to store time of each angle reading
OutputCase = 0
DESIRED_ANGLE = 1.05 #1.45
robot_x_received = 0
robot_y_received = 0

data_per_second = 0
t0 = 0

def send_ble_command():
    global last_sent_time
    current_time = time.time()
    if current_time - last_sent_time >= send_interval:
        last_sent_time = current_time
        if client and ble_loop:
            command = f"OC:{OutputCase:.1f} DA:{DESIRED_ANGLE:.2f}"
            test_str_bytes = bytearray(command, encoding="utf-8")
            asyncio.run_coroutine_threadsafe(
                client.write_gatt_char(laptop_master_send_characteristic_uuid, test_str_bytes, response=True),
                ble_loop
            )
            print(command)

def on_press(key):
    global OutputCase, DESIRED_ANGLE
    try:
        if(key.char == 'w'): #forward
            OutputCase = 1
        elif(key.char == 's'): #backward
            OutputCase = 2
        elif(key.char == 'd'): #right
            OutputCase = 3
        elif(key.char == 'a'): # left
            OutputCase = 4
        elif(key.char == 'q'): #Balance
            OutputCase = 0
        elif(key.char == 'p'): #Increase desired angle
            DESIRED_ANGLE += 0.1
        elif(key.char == 'l'): #Decrease desired angle
            DESIRED_ANGLE -= 0.1

        send_ble_command()
    except AttributeError:
        pass

async def run_ble():
    global client, ble_loop, angles, time_stamps, robot_x_array, robot_y_array, robot_yaw_array, distance_sensed_x_array, distance_sensed_y_array, robot_x_received, robot_y_received
    ble_loop = asyncio.get_running_loop()
    devices = await BleakScanner.discover()
    arduino_device = next((device for device in devices if device.name == arduino_device_name), None)
    
    if not arduino_device:
        print(f"Could not find device with name: {arduino_device_name}")
        return

    print(f"Connecting to {arduino_device_name} ({arduino_device.address})")
    async with BleakClient(arduino_device.address) as client:
        print(f"Connected to {arduino_device_name}\n")
        while True:
            try:
                
                dataReceived = await client.read_gatt_char(laptop_master_characteristic_data_receive_uuid)
                # sonarReceived = await client.read_gatt_char(laptop_master_characteristic_sonar_uuid, 10)
                dataReceived = dataReceived.decode('utf-8').strip()
                # sonarReceived = sonarReceived.decode('utf-8').strip() 

                # angleReceived = float(dataReceived)
                # sonarDistance = float(dataReceived)
                # stepperAngle = float(dataReceived)

                angleReceivedStr, robotXStr, robotYStr, distance_sensed_x_Str, distance_sensed_y_Str = dataReceived.split(" ")
                angleReceived = float(angleReceivedStr)
                # robot_yaw_received = float(robotYawStr)
                robot_x_received = float(robotXStr)
                robot_y_received = float(robotYStr)
                distance_sensed_x_received = float(distance_sensed_x_Str)
                distance_sensed_y_received = float(distance_sensed_y_Str)

                print(f"x: {distance_sensed_x_received} y: {distance_sensed_y_received}")

                # Update angle data for plotting
                current_time = time.time()
                angles.append(float(angleReceived))
                # robot_yaw_array.append(float(robot_yaw_received))
                robot_x_array.append(float(robot_x_received))
                robot_y_array.append(float(robot_y_received))
                distance_sensed_x_array.append(float(distance_sensed_x_received))
                distance_sensed_y_array.append(float(distance_sensed_y_received))
                time_stamps.append(current_time)

                # Limit the number of points in the plot to avoid memory issues
                if len(angles) > 100:
                    angles.pop(0)
                    time_stamps.pop(0)
                # if len(robot_yaw_array) > 100:
                #     robot_yaw_array.pop(0)
                if len(robot_x_array) > 100:
                    robot_x_array.pop(0)
                if len(robot_y_array) > 100:
                    robot_y_array.pop(0)


                update_plots()

                # Update the angle label
                update_angle_label()
                # update_robot_yaw_label()
                update_robot_x_label()
                update_robot_y_label()

                await asyncio.sleep(0.0001)
            except Exception as e:
                print(f"Error: {e}")
                break

# Create and update plot
def update_plots():
    global angles, time_stamps, robot_x_array, robot_y_array, distance_sensed_x_array, distance_sensed_y_array, robot_x_received, robot_y_received

    # Update the tilt angle plot (Graph 1) - Left Plot
    ax1.clear()
    ax1.plot(time_stamps, angles, label="Tilt Angle (Degrees)", color="#DE0000")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Angle (°)")
    ax1.get_xaxis().get_major_formatter().set_scientific(False)
    ax1.set_title("Tilt Angle Over Time")

    # Update the robot coordinates plot (Graph 2) as a scatter plot - Right Plot
    ax2.clear()
    ax2.scatter(distance_sensed_x_array, distance_sensed_y_array, label="Robot Map", color="#0000A0", s=20)  # s=20 sets the size of the points
    ax2.scatter(robot_x_received, robot_y_received, color='red', s=100)

    ax2.set_xlabel("X Coordinate (cm)")
    ax2.set_ylabel("Y Coordinate (cm)")
    ax2.set_title("Robot Path")

    # Set fixed axis limits to 400 cm by 400 cm
    ax2.set_xlim(-100, 100)
    ax2.set_ylim(-100, 100)

    ax2.set_aspect('equal', adjustable='datalim')


    # Adjust layout to place the plots side by side
    plt.tight_layout()

    # Draw both plots
    canvas.draw()



# Run BLE communication in a separate thread
def start_ble_loop():
    asyncio.run(run_ble())

ble_thread = threading.Thread(target=start_ble_loop, daemon=True)
ble_thread.start()

# Start listening to keyboard inputs
listener = keyboard.Listener(on_press=on_press)
listener.start()

# Create the CustomTkinter GUI
ctk.set_appearance_mode("light")
ctk.set_default_color_theme("blue")

root = ctk.CTk()
root.geometry("1366x768")
root.title("C5 Balancing Robot Controller")

# Set the root window background to white
root.configure(bg="white")

# Add text above the angle plot
angleplot_label = ctk.CTkLabel(root, text="Robot Angle", font=("Roboto", 24))
angleplot_label.place(relx=0.64, rely=0.153, anchor="center")

# Create a frame for plotting the angle
plot_frame = ctk.CTkFrame(root, width=500, height=100)
plot_frame.place(relx=0.77, rely=0.47, anchor="center")

# Create a Matplotlib figure with two subplots: one for tilt angles, one for robot path
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6.25, 9))

# Set the figure's border color to dark red
fig.patch.set_edgecolor('#9A0000')  # Dark red color
fig.patch.set_linewidth(3)  # Set border thickness

# Set titles and labels for each plot
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Angle (°)")
ax1.set_title("Tilt Angle Over Time")
ax2.set_xlabel("X Coordinate (cm)")
ax2.set_ylabel("Y Coordinate (cm)")
ax2.set_title("Robot Path")

# Embed Matplotlib plot into CustomTkinter window
canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.get_tk_widget().pack(fill="both", expand=True)


label = ctk.CTkLabel(root, text="C5-Robot", font=("Roboto", 40))
label.place(relx=0.1, rely=0.1, anchor="center")

# # Create a terminal display with custom border color and width
# terminal_frame = ctk.CTkFrame(root, width=500, height=200, border_color="#9A0000", border_width=2)
# terminal_frame.place(relx=0.74, rely=0.8, anchor="center")

# # Remove default border from the terminal text box by setting border width to 0
# terminal = ctk.CTkTextbox(terminal_frame, width=480, height=180, wrap="word", state="disabled", font=("Courier", 12), border_width=0)
# terminal.pack(padx=10, pady=10)

# # Add text above the terminal
# terminal_label = ctk.CTkLabel(root, text="Terminal", font=("Roboto", 16))
# terminal_label.place(relx=0.628, rely=0.685, anchor="center")

# Create a forward and backward slider
fbslider = ctk.CTkSlider(root, from_=-4, to=4, orientation="vertical", button_color="#CD0000")
fbslider.set(0)  # Set default position to 0°
fbslider.pack(pady=20)
fbslider.place(relx= 0.05, rely=0.75, anchor="center")

# Create a left and right slider
lrslider = ctk.CTkSlider(root, from_=-2, to=2, orientation="vertical", button_color="#CD0000")
lrslider.set(0)  # Set default position to 0°
lrslider.pack(pady=20)
lrslider.place(relx= 0.10, rely=0.75, anchor="center")

# Add text above the move forward and backward slider 
fb_label_text = ctk.CTkLabel(root, text="FB", font=("Roboto", 24))
fb_label_text.place(relx=0.05, rely=0.6, anchor="center")

# Add text above the move left and right slider 
lr_label_text = ctk.CTkLabel(root, text="LR", font=("Roboto", 24))
lr_label_text.place(relx=0.10, rely=0.6, anchor="center")

# Add Angle Offset buttons
AOP = ctk.CTkButton(root, 
                               text="Angle OS+", 
                               width=60, 
                               fg_color="#CD0000", 
                               hover_color="#9C0000", 
                               text_color="white")
AOP.place(relx=0.05, rely=0.55, anchor="center")

AOM = ctk.CTkButton(root, 
                            text="Angle OS-", 
                            width=60, 
                            fg_color="#CD0000", 
                            hover_color="#9C0000", 
                            text_color="white")
AOM.place(relx=0.1, rely=0.55, anchor="center")

# # Redirect print to the terminal
# class TerminalRedirector(io.StringIO):
#     def __init__(self, textbox):
#         super().__init__()
#         self.textbox = textbox
    
#     def write(self, text):
#         # self.textbox.configure(state="normal")
#         self.textbox.insert("end", text)
#         self.textbox.yview("end")  # Auto-scroll
#         self.textbox.configure(state="disabled")

# # Set up terminal redirection
# sys.stdout = TerminalRedirector(terminal)


# Create a white background frame for the angle label
angle_frame = ctk.CTkFrame(root, width=200, height=50, fg_color="white", border_width=2, border_color="#9A0000")
angle_frame.place(relx=0.08, rely=0.43, anchor="center")

# Create the angle label inside the frame
angle_label = ctk.CTkLabel(angle_frame, text="Angle: 0°", font=("Roboto", 24))
angle_label.place(relx=0.5, rely=0.5, anchor="center")  # Center the label inside the frame

# robot_yaw_frame = ctk.CTkFrame(root, width=200, height=50, fg_color="white", border_width=2, border_color="#9A0000")
# robot_yaw_frame.place(relx=0.08, rely=0.50, anchor="center")

# robot_yaw_label = ctk.CTkLabel(robot_yaw_frame, text="Yaw: 0°", font=("Roboto", 24))
# robot_yaw_label.place(relx=0.5, rely=0.5, anchor="center")  # Center the label inside the frame

robot_x_frame = ctk.CTkFrame(root, width=200, height=50, fg_color="white", border_width=2, border_color="#9A0000")
robot_x_frame.place(relx=0.08, rely=0.29, anchor="center")

robot_x_label = ctk.CTkLabel(robot_x_frame, text="X: 0 cm", font=("Roboto", 24))
robot_x_label.place(relx=0.5, rely=0.5, anchor="center")  # Center the label inside the frame

robot_y_frame = ctk.CTkFrame(root, width=200, height=50, fg_color="white", border_width=2, border_color="#9A0000")
robot_y_frame.place(relx=0.08, rely=0.36, anchor="center")

robot_y_label = ctk.CTkLabel(robot_y_frame, text="Y: 0 cm", font=("Roboto", 24))
robot_y_label.place(relx=0.5, rely=0.5, anchor="center")  # Center the label inside the frame


# Update the angle label with the latest value
def update_angle_label():
    global angles
    if angles:
        latest_angle = angles[-1]
        angle_label.configure(text=f"Angle: {latest_angle:.1f}°")


# def update_robot_yaw_label():
#     global robot_yaw_array
#     if robot_yaw_array:
#         latest_robot_yaw = robot_yaw_array[-1]
#         robot_yaw_label.configure(text=f"Yaw: {latest_robot_yaw:.2f}°")

def update_robot_x_label():
    global robot_x_array
    if robot_x_array:
        latest_robot_x = robot_x_array[-1]
        robot_x_label.configure(text=f"X: {latest_robot_x:.1f} cm")

def update_robot_y_label():
    global robot_y_array
    if robot_y_array:
        latest_robot_y = robot_y_array[-1]
        robot_y_label.configure(text=f"Y: {latest_robot_y:.1f} cm")

root.mainloop()