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
laptop_master_characteristic_pidoutput_uuid = "00000003-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_kpoutput_uuid = "00000004-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_kioutput_uuid = "00000005-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_kdoutput_uuid = "00000006-5EC4-4083-81CD-A10B8D5CF6EC"



# The name of the Arduino device
arduino_device_name = "C5-BLE"  # Replace with your Arduino device name

global client, ble_loop
client = None
ble_loop = None

# Speed variables
Kp = 2.95 #1.7
Ki = 32.8 #16.0
Kd = 0.1045 #0.085

#LMF = 0.830
#LMB = 0.640
#RMF = 0.830
#RMB = 0.645

LMF = 0.056
LMB = 0.051
RMF = 0.054
RMB = 0.051

# Throttle BLE messages
last_sent_time = 0
send_interval = 0.0001  # Minimum time between BLE sends (1ms)

# Angle Data for Live Plotting
angles = []  # List to store received angles
PIDouts = [] # List to store PID outputs
KpOuts = []
KiOuts = []
KdOuts = []
time_stamps = []  # List to store time of each angle reading

data_per_second = 0
t0 = 0

def fb_move(value):
    print(f"angle desired: {value}")
    # label.configure(text=f"Angle: {value:.1f}°")

def send_ble_command():
    global last_sent_time
    current_time = time.time()
    if current_time - last_sent_time >= send_interval:
        last_sent_time = current_time
        if client and ble_loop:
            command = f"Kp:{Kp:.2f} Ki:{Ki:.2f} Kd:{Kd:.2f} RI:{1} {LMF:.3f} {LMB:.3f} {RMF:.3f} {RMB:.3f}"
            test_str_bytes = bytearray(command, encoding="utf-8")
            asyncio.run_coroutine_threadsafe(
                client.write_gatt_char(laptop_master_send_characteristic_uuid, test_str_bytes, response=True),
                ble_loop
            )
            print(f"Kp:{Kp:.4f} Ki:{Ki:.4f} Kd:{Kd:.4f} RI:{1} {LMF:.3f} {LMB:.3f} {RMF:.3f} {RMB:.3f}")

def on_press(key):
    global Kp, Ki, Kd, LMF, LMB, RMF, RMB
    try:
        if key.char == 'q':
            Kp += 0.05
        elif key.char == 'a':
            Kp -= 0.05
        elif key.char == 'w':
            Ki += 0.1
        elif key.char == 's':
            Ki -= 0.1
        elif key.char == 'e':
            Kd += 0.005
        elif key.char == 'd':
            Kd -= 0.005
        elif key.char == 'r':
            LMF += 0.001
        elif key.char == 'f':
            LMF -= 0.001
        elif key.char == 't':
            LMB += 0.001
        elif key.char == 'g':
            LMB -= 0.001
        elif key.char == 'y':
            RMF += 0.001
        elif key.char == 'h':
            RMF -= 0.001
        elif key.char == 'u':
            RMB += 0.1
        elif key.char == 'j':
            RMB -= 0.1
            
        send_ble_command()
    except AttributeError:
        pass

def on_button_click(input):
    global Kp, Ki, Kd, LMF, LMB, RMF, RMB
    if input == "KpPlus":
        Kp += 0.1
    elif input == "KpMinus":
        Kp -= 0.1
    elif input == "KiPlus":
        Ki += 0.05
    elif input == "KiMinus":
        Ki -= 0.05
    elif input == "KdPlus":
        Kd += 0.005
    elif input == "KdMinus":
        Kd -= 0.005
    elif input == "LMFPlus":
        LMF += 0.005
    elif input == "LMFMinus":
        LMF -= 0.005
    elif input == "LMBPlus":
        LMB += 0.005
    elif input == "LMBMinus":
        LMB -= 0.005
    elif input == "RMFPlus":
        RMF += 0.005
    elif input == "RMFMinus":
        RMF -= 0.005
    elif input == "RMBPlus":
        RMB += 0.005
    elif input == "RMBMinus":
        RMB -= 0.005


    send_ble_command()

async def run_ble():
    global client, ble_loop, angles, time_stamps, PIDouts, KpOuts, KiOuts, KdOuts
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
                dataReceived = dataReceived.decode('utf-8').strip()

                angleReceivedStr, PIDOutputStr, KpOutputStr, KiOutputStr, KdOutputStr = dataReceived.split(" ")
                angleReceived = float(angleReceivedStr)
                pidOutput = float(PIDOutputStr)
                KpReceived = float(KpOutputStr)
                KiReceived = float(KiOutputStr)
                KdReceived = float(KdOutputStr)
                # pidOutput = await client.read_gatt_char(laptop_master_characteristic_pidoutput_uuid)
                # pidOutput = pidOutput.decode('utf-8')
                # KpReceived = await client.read_gatt_char(laptop_master_characteristic_kpoutput_uuid)
                # KpReceived = KpReceived.decode('utf-8')
                # KiReceived = await client.read_gatt_char(laptop_master_characteristic_kioutput_uuid)
                # KiReceived = KiReceived.decode('utf-8')
                # KdReceived = await client.read_gatt_char(laptop_master_characteristic_kdoutput_uuid)
                # KdReceived = KdReceived.decode('utf-8')
                #print(f"Tilt: {angleReceived}° PID Output: {pidOutput}")
                # print(f"Tilt: {angleReceived}°")

                # Update angle data for plotting
                current_time = time.time()
                angles.append(float(angleReceived))
                PIDouts.append(float(pidOutput))
                KpOuts.append(float(KpReceived))
                KiOuts.append(float(KiReceived))
                KdOuts.append(float(KdReceived))
                time_stamps.append(current_time)

                # Limit the number of points in the plot to avoid memory issues
                if len(angles) > 100:
                    angles.pop(0)
                    time_stamps.pop(0)
                if len(PIDouts) > 100:
                    PIDouts.pop(0)
                if len(KpOuts) > 100:
                    KpOuts.pop(0)
                if len(KiOuts) > 100:
                    KiOuts.pop(0)
                if len(KdOuts) > 100:
                    KdOuts.pop(0)


                update_plot()

                # Update the angle label
                update_angle_label()
                update_pidoutput_label()
                update_kpoutput_label()
                update_kioutput_label()
                update_kdoutput_label()

                await asyncio.sleep(0.0001)
            except Exception as e:
                print(f"Error: {e}")
                break

# Create and update plot
def update_plot():
    global angles, time_stamps
    ax.clear()
    
    # Plot the angle with a red line
    ax.plot(time_stamps, angles, label="Angle (Degrees)", color="#DE0000")
    
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (°)")

    # Remove the scientific notation for the x-axis
    ax.get_xaxis().get_major_formatter().set_scientific(False)

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
plot_frame.place(relx=0.77, rely=0.4, anchor="center")

# Create a Matplotlib figure with a dark-red border
fig, ax = plt.subplots(figsize=(6.25, 4.5))

# Set the figure's border color to dark red
fig.patch.set_edgecolor('#9A0000')  # Dark red color
fig.patch.set_linewidth(3)  # Set border thickness

ax.set_xlabel("Time (s)")
ax.set_ylabel("Angle (°)")

# Embed Matplotlib plot into CustomTkinter window
canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.get_tk_widget().pack(fill="both", expand=True)

# Add control buttons
KpPlus = ctk.CTkButton(root, 
                               text="Kp+", 
                               width=60, 
                               fg_color="#CD0000", 
                               hover_color="#9C0000", 
                               text_color="white", 
                               command=lambda: on_button_click("KpPlus"))
KpPlus.place(relx=0.05, rely=0.575, anchor="center")

KpMinus = ctk.CTkButton(root, 
                            text="Kp-", 
                            width=60, 
                            fg_color="#CD0000", 
                            hover_color="#9C0000", 
                            text_color="white", 
                            command=lambda: on_button_click("KpMinus"))
KpMinus.place(relx=0.05, rely=0.625, anchor="center")

KiPlus = ctk.CTkButton(root, 
                            text="Ki+", 
                            width=60, 
                            fg_color="#CD0000", 
                            hover_color="#9C0000", 
                            text_color="white", 
                            command=lambda: on_button_click("KiPlus"))
KiPlus.place(relx=0.1, rely=0.575, anchor="center")

KiMinus = ctk.CTkButton(root, 
                             text="Ki-", 
                             width=60, 
                             fg_color="#CD0000", 
                             hover_color="#9C0000", 
                             text_color="white", 
                             command=lambda: on_button_click("KiMinus"))
KiMinus.place(relx=0.1, rely=0.625, anchor="center")

KdPlus = ctk.CTkButton(root, 
                            text="Kd+", 
                            width=60, 
                            fg_color="#CD0000", 
                            hover_color="#9C0000", 
                            text_color="white", 
                            command=lambda: on_button_click("KdPlus"))
KdPlus.place(relx=0.15, rely=0.575, anchor="center")

KiMinus = ctk.CTkButton(root, 
                             text="Kd-", 
                             width=60, 
                             fg_color="#CD0000", 
                             hover_color="#9C0000", 
                             text_color="white", 
                             command=lambda: on_button_click("KdMinus"))
KiMinus.place(relx=0.15, rely=0.625, anchor="center")

LMotorForwardPlus = ctk.CTkButton(root, 
                            text="LMF+", 
                            width=60, 
                            fg_color="#CD0000", 
                            hover_color="#9C0000", 
                            text_color="white", 
                            command=lambda: on_button_click("LMFPlus"))
LMotorForwardPlus.place(relx=0.2, rely=0.575, anchor="center")

LMotorForwardMinus = ctk.CTkButton(root, 
                             text="LMF-", 
                             width=60, 
                             fg_color="#CD0000", 
                             hover_color="#9C0000", 
                             text_color="white", 
                             command=lambda: on_button_click("LMFMinus"))
LMotorForwardMinus.place(relx=0.2, rely=0.625, anchor="center")

LMotorBackwardPlus = ctk.CTkButton(root, 
                            text="LMB+", 
                            width=60, 
                            fg_color="#CD0000", 
                            hover_color="#9C0000", 
                            text_color="white", 
                            command=lambda: on_button_click("LMBPlus"))
LMotorBackwardPlus.place(relx=0.25, rely=0.575, anchor="center")

LMotorBackwardMinus = ctk.CTkButton(root, 
                             text="LMB-", 
                             width=60, 
                             fg_color="#CD0000", 
                             hover_color="#9C0000", 
                             text_color="white", 
                             command=lambda: on_button_click("LMBMinus"))
LMotorBackwardMinus.place(relx=0.25, rely=0.625, anchor="center")

RMotorForwardPlus = ctk.CTkButton(root, 
                            text="RMF+", 
                            width=60, 
                            fg_color="#CD0000", 
                            hover_color="#9C0000", 
                            text_color="white", 
                            command=lambda: on_button_click("RMFPlus"))
RMotorForwardPlus.place(relx=0.3, rely=0.575, anchor="center")

RMotorForwardMinus = ctk.CTkButton(root, 
                             text="RMF-", 
                             width=60, 
                             fg_color="#CD0000", 
                             hover_color="#9C0000", 
                             text_color="white", 
                             command=lambda: on_button_click("RMFMinus"))
RMotorForwardMinus.place(relx=0.3, rely=0.625, anchor="center")

RMotorBackwardPlus = ctk.CTkButton(root, 
                            text="RMB+", 
                            width=60, 
                            fg_color="#CD0000", 
                            hover_color="#9C0000", 
                            text_color="white", 
                            command=lambda: on_button_click("RMBPlus"))
RMotorBackwardPlus.place(relx=0.35, rely=0.575, anchor="center")

RMotorBackwardMinus = ctk.CTkButton(root, 
                             text="RMB-", 
                             width=60, 
                             fg_color="#CD0000", 
                             hover_color="#9C0000", 
                             text_color="white", 
                             command=lambda: on_button_click("RMBMinus"))
RMotorBackwardMinus.place(relx=0.35, rely=0.625, anchor="center")

label = ctk.CTkLabel(root, text="C5-Robot", font=("Roboto", 40))
label.place(relx=0.1, rely=0.1, anchor="center")

# Create a terminal display with custom border color and width
terminal_frame = ctk.CTkFrame(root, width=500, height=200, border_color="#9A0000", border_width=2)
terminal_frame.place(relx=0.74, rely=0.8, anchor="center")

# Remove default border from the terminal text box by setting border width to 0
terminal = ctk.CTkTextbox(terminal_frame, width=480, height=180, wrap="word", state="disabled", font=("Courier", 12), border_width=0)
terminal.pack(padx=10, pady=10)

# Add text above the terminal
terminal_label = ctk.CTkLabel(root, text="Terminal", font=("Roboto", 16))
terminal_label.place(relx=0.628, rely=0.685, anchor="center")

# Redirect print to the terminal
class TerminalRedirector(io.StringIO):
    def __init__(self, textbox):
        super().__init__()
        self.textbox = textbox
    
    def write(self, text):
        self.textbox.configure(state="normal")
        self.textbox.insert("end", text)
        self.textbox.yview("end")  # Auto-scroll
        self.textbox.configure(state="disabled")

# Set up terminal redirection
sys.stdout = TerminalRedirector(terminal)


# Create a white background frame for the angle label
angle_frame = ctk.CTkFrame(root, width=160, height=50, fg_color="white", border_width=2, border_color="#9A0000")
angle_frame.place(relx=0.08, rely=0.5, anchor="center")

# Create the angle label inside the frame
angle_label = ctk.CTkLabel(angle_frame, text="Angle: 0°", font=("Roboto", 24))
angle_label.place(relx=0.5, rely=0.5, anchor="center")  # Center the label inside the frame

#Create a white background frame for the pidoutput label
pidoutput_frame = ctk.CTkFrame(root, width=240, height=50, fg_color="white", border_width=2, border_color="#9A0000")
pidoutput_frame.place(relx=0.3, rely=0.29, anchor="center")

# Create the pidoutput label inside the frame
pidoutput_label = ctk.CTkLabel(pidoutput_frame, text="PID: ", font=("Roboto", 24))
pidoutput_label.place(relx=0.5, rely=0.5, anchor="center")  # Center the label inside the frame

# Create a white background frame for the pidoutput label
kpoutput_frame = ctk.CTkFrame(root, width=240, height=50, fg_color="white", border_width=2, border_color="#9A0000")
kpoutput_frame.place(relx=0.3, rely=0.36, anchor="center")

# Create the pidoutput label inside the frame
kpoutput_label = ctk.CTkLabel(kpoutput_frame, text="Kp: ", font=("Roboto", 24))
kpoutput_label.place(relx=0.5, rely=0.5, anchor="center")  # Center the label inside the frame

kioutput_frame = ctk.CTkFrame(root, width=240, height=50, fg_color="white", border_width=2, border_color="#9A0000")
kioutput_frame.place(relx=0.3, rely=0.43, anchor="center")

# Create the pidoutput label inside the frame
kioutput_label = ctk.CTkLabel(kioutput_frame, text="Ki: ", font=("Roboto", 24))
kioutput_label.place(relx=0.5, rely=0.5, anchor="center")  # Center the label inside the frame

kdoutput_frame = ctk.CTkFrame(root, width=240, height=50, fg_color="white", border_width=2, border_color="#9A0000")
kdoutput_frame.place(relx=0.3, rely=0.5, anchor="center")

# Create the pidoutput label inside the frame
kdoutput_label = ctk.CTkLabel(kdoutput_frame, text="Kd: ", font=("Roboto", 24))
kdoutput_label.place(relx=0.5, rely=0.5, anchor="center")  # Center the label inside the frame


# Update the angle label with the latest value
def update_angle_label():
    global angles
    if angles:
        latest_angle = angles[-1]
        angle_label.configure(text=f"Angle: {latest_angle:.2f}°")

# Update the pidoutput label with the latest value
def update_pidoutput_label():
    global PIDouts
    if PIDouts:
        latest_PID = PIDouts[-1]
        pidoutput_label.configure(text=f"PID: {latest_PID:.2f}")

def update_kpoutput_label():
    global KpOuts
    if KpOuts:
        latest_Kp = KpOuts[-1]
        kpoutput_label.configure(text=f"Kp: {latest_Kp:.2f}")

def update_kioutput_label():
    global KiOuts
    if KiOuts:
        latest_Ki = KiOuts[-1]
        kioutput_label.configure(text=f"Ki: {latest_Ki:.2f}")

def update_kdoutput_label():
    global KdOuts
    if KdOuts:
        latest_Kd = KdOuts[-1]
        kdoutput_label.configure(text=f"Kd: {latest_Kd:.2f}")


root.mainloop()