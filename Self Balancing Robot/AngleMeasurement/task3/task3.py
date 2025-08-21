import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time  # Import time module

class GyroAccumulatedAnglePlotter:
    def __init__(self, port="COM7", baud_rate=9600):
        self.ser = serial.Serial(port, baud_rate, timeout=1)
        self.qData, self.tData = [], []
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.qLine, = self.ax.plot([], [], label="Angle", color="purple")
        self.qText = self.ax.text(0.02, 0.95, "", transform=self.ax.transAxes, fontsize=10, verticalalignment='top')
        self.start_time = time.time()  # Initialize start time to track real-time elapsed
        self.axesSetup()

    def axesSetup(self):
        self.ax.set_title("Gyro Angle vs Time")
        self.ax.set_xlabel("Time (s)")  # Update x-axis label to seconds
        self.ax.set_ylabel("Angle (deg)")
        self.ax.set_ylim(-100, 100)
        self.ax.legend()

    def serialRead(self):
        try:
            while self.ser.in_waiting:  # while helps clear the buffer out compared to if
                line = self.ser.readline().decode('ascii').strip()
            if line:
                return float(line)
        except Exception:  # ignore any errors in reading data
            pass
        return None

    def update(self, frame):
        qValue = self.serialRead()
        if qValue is not None:
            # Append real elapsed time in seconds to tData
            self.tData.append(time.time() - self.start_time)
            self.qData.append(qValue)

            self.qLine.set_data(self.tData, self.qData)
            self.qText.set_text(f"Gyro Angle (deg): {qValue:.4f}")

            # Set x-axis limits based on real elapsed time
            # Ensure the x-axis shows a reasonable window (last 10 seconds)
            self.ax.set_xlim(max(0, self.tData[-1] - 10), self.tData[-1])

            self.ax.relim()
            self.ax.autoscale_view()

        return self.qLine, self.qText

    def run(self):
        ani = FuncAnimation(self.fig, self.update, interval=50)  # Update every 50ms
        try:
            plt.tight_layout()
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.ser.close()

if __name__ == "__main__":
    plotter = GyroAccumulatedAnglePlotter()
    plotter.run()
