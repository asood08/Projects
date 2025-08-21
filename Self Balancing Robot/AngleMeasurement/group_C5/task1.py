import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time  # Import time module

class GyroAxisPlotter:
    def __init__(self, port="COM7", baud_rate=9600):
        self.ser = serial.Serial(port, baud_rate, timeout=1)
        self.xData, self.yData, self.zData, self.tData = [], [], [], []
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.xLine, = self.ax.plot([], [], label="X-axis", color="blue")
        self.yLine, = self.ax.plot([], [], label="Y-axis", color="green")
        self.zLine, = self.ax.plot([], [], label="Z-axis", color="red")
        self.xText = self.ax.text(0.02, 0.95, "", transform=self.ax.transAxes, fontsize=10, verticalalignment='top')
        self.yText = self.ax.text(0.02, 0.90, "", transform=self.ax.transAxes, fontsize=10, verticalalignment='top')
        self.zText = self.ax.text(0.02, 0.85, "", transform=self.ax.transAxes, fontsize=10, verticalalignment='top')
        self.start_time = time.time()  # Initialize start time to track real-time elapsed
        self.axesSetup()

    def axesSetup(self):
        self.ax.set_title("Gyroscope XYZ Data")
        self.ax.set_xlabel("Time(s)")  # Change x-axis label to seconds
        self.ax.set_ylabel("Angular Speed (deg/s)")
        self.ax.legend()

    def serialRead(self):
        try:
            while self.ser.in_waiting:  # while helps clear the buffer out compared to if
                line = self.ser.readline().decode('ascii').strip() 
            values = line.split(',')
            if len(values) == 3:
                return [float(v) for v in values]
        except Exception: # ignore any errors in reading data
            pass
        return None

    def update(self, frame):
        axData = self.serialRead()
        if axData:
            # Append real elapsed time in seconds to tData
            self.tData.append(time.time() - self.start_time)
            self.xData.append(axData[0])
            self.yData.append(axData[1])
            self.zData.append(axData[2])

            self.xLine.set_data(self.tData, self.xData)
            self.yLine.set_data(self.tData, self.yData)
            self.zLine.set_data(self.tData, self.zData)

            self.xText.set_text(f"X: {axData[0]:.2f} deg/s")
            self.yText.set_text(f"Y: {axData[1]:.2f} deg/s")
            self.zText.set_text(f"Z: {axData[2]:.2f} deg/s")

            # Set x-axis limits based on real elapsed time, not the number of data points
            # Ensure the x-axis shows a reasonable window (last 10 seconds)
            self.ax.set_xlim(max(0, self.tData[-1] - 10), self.tData[-1])

            self.ax.relim()
            self.ax.autoscale_view()  # Rescale the view for y-axis

        return self.xLine, self.yLine, self.zLine, self.xText, self.yText, self.zText

    def run(self):
        ani = FuncAnimation(self.fig, self.update, interval=10)  # Update every 10ms
        try:
            plt.tight_layout()
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.ser.close()

if __name__ == "__main__":
    plotter = GyroAxisPlotter()
    plotter.run()
