import tkinter
import tkintermapview
from pymbtiles import MBtiles
from queue import Queue
import serial
import matplotlib.pyplot as plt
import numpy as np


class MapAndView():
    def __init__(self, track_len=1, mbtiles_file=r"C:\Users\Kadir\Desktop\Arduinokoder_båt\Ålesund_mbtiles_uid_05d25f23-bea3-4647-80f7-66552cceac93\Ålesund.mbtiles"):
        self.track_len = track_len
        self.gps_track = Queue(maxsize=self.track_len)
        self.desired_speed = 0  # Variable to hold the desired speed input

        # Initialize tkinter window and map view
        self.root_tk = tkinter.Tk()
        self.root_tk.geometry(f"{800}x{600}")
        self.root_tk.title("Combined Map and Data Visualization")

        self.map_data = MBtiles(mbtiles_file)
        self.map_widget = tkintermapview.TkinterMapView(self.root_tk, width=800, height=600, corner_radius=30)
        self.map_widget.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER)

        self.ntnu_lat_pos = 62.4720646
        self.ntnu_lon_pos = 6.2354073
        self.map_widget.set_position(self.ntnu_lat_pos, self.ntnu_lon_pos)  # aalesund, norway
        self.map_widget.set_zoom(15)
        self.path_1 = self.map_widget.set_path([(self.ntnu_lat_pos, self.ntnu_lon_pos), (self.ntnu_lat_pos, self.ntnu_lon_pos)])

        # Initialize serial connection
        self.ser = serial.Serial("COM6", 115200, timeout=0.1)  # Adjust timeout for faster response

        # Create user input window
        self.create_input_window()

        # Start updating GUI and map
        self.update_data()
        self.root_tk.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root_tk.mainloop()

    def create_input_window(self):
        self.input_window = tkinter.Toplevel(self.root_tk)
        self.input_window.title("User Input")
        self.input_window.geometry("300x150")

        self.input_label = tkinter.Label(self.input_window, text="Enter target speed,angle:")
        self.input_label.pack()

        self.input_entry = tkinter.Entry(self.input_window, width=30)
        self.input_entry.pack()

        self.previous_speed_angle_label = tkinter.Label(self.input_window, text="Previous speed_angle: None")
        self.previous_speed_angle_label.pack()

        self.input_entry.bind("<Return>", self.update_target_speed_and_angle)

    def update_target_speed_and_angle(self, event):
        try:
            self.target_speed_and_angle = str(self.input_entry.get())
            print("Target Speed and angle set to:", self.target_speed_and_angle)
            self.previous_speed_angle_label.config(text=f"Previous speed_angle: {self.target_speed_and_angle}")
            self.send_speed_and_angle_to_arduino(self.target_speed_and_angle)  # Send desired speed to Arduino
        except ValueError:
            print("Invalid input. Please enter a valid number.")

        self.input_entry.delete(0, tkinter.END)  # Clear the input field after sending

    def send_speed_and_angle_to_arduino(self, speed_and_angle):
        if self.ser.isOpen():
            try:
                self.ser.write(speed_and_angle.encode())  # Send desired speed as bytes to Arduino
                print("Target speed and angle sent to Arduino:", speed_and_angle)
            except Exception as e:
                print("Error:", e)

    def update_data(self):
        try:
            if self.ser.isOpen():
                input_data = self.ser.readline().strip().decode("utf-8")
                if input_data:  # Check if data is received
                    print(input_data)  # Print received message
                    gps_speed, imu_angle, gps_lat, gps_lon = self.parse_message(input_data)
                    if gps_speed is not None and imu_angle is not None and gps_lat is not None and gps_lon is not None:
                        lat_dd = self.ddm2dd(gps_lat, type='ddmm')
                        lon_dd = self.ddm2dd(gps_lon, type='dmm')

                        if self.gps_track.full():
                            self.gps_track.get()
                        self.gps_track.put((lat_dd, lon_dd))

                        self.path_1.set_position_list(list(self.gps_track.queue))
                        self.map_widget.set_position(lat_dd, lon_dd)

                        self.root_tk.after_idle(self.update_gui, gps_speed, imu_angle)  # Update GUI from the main thread
        except Exception as ex:
            print("Error updating data:", ex)
        self.root_tk.after(100, self.update_data)  # Adjust update interval if needed

    def update_gui(self, gps_speed=0, imu_angle=0):
        try:
            plt.clf()  # Clear the previous plot
            self.plot_gps_speed(gps_speed)
            self.plot_compass(imu_angle)
            plt.pause(0.001)  # Pause to update the plot
        except Exception as ex:
            print("Error updating GUI:", ex)

    def parse_message(self, message):
        parts = message.split(',')
        try:
            gps_speed = float(parts[2])
            imu_angle = float(parts[4])
            gps_lat = float(parts[0])
            gps_lon = float(parts[1])
            return gps_speed, imu_angle, gps_lat, gps_lon
        except (IndexError, ValueError):
            return None, None, None, None

    def ddm2dd(self, val, type='ddmm'):
        val_str = str(val)
        if type == 'ddmm':
            int_degs = int(val_str[:2])
            dec_degs = float(val_str[2:]) / 60.0
        elif type == 'dmm':
            int_degs = int(val_str[:1])  # Assuming longitude degrees always have three digits
            dec_degs = float(val_str[1:]) / 60.0
        else:
            raise ValueError("Invalid type parameter")
        return int_degs + dec_degs

    def plot_gps_speed(self, speed):
        ax1 = plt.subplot(1, 3, 1)
        ax1.set_title('GPS Speed')
        ax1.text(0.5, 0.5, f'Speed: {speed}', fontsize=20, ha='center')
        ax1.axis('off')

    def plot_compass(self, angle):
        ax2 = plt.subplot(1, 3, 2, polar=True)

        # Adjust the size of the subplot
        ax2.set_position([0.45, 0.1, 0.5, 0.8])

        theta = np.deg2rad(-(angle - 90))
        ax2.plot(theta, 1, 'ro')  # Keep radius as 1 for the compass
        ax2.plot([0, theta], [0, 1], 'r-')
        ax2.text(0, 1.1, 'E', fontsize=12, ha='center')
        ax2.text(np.deg2rad(-90), 1.1, 'S', fontsize=12, ha='center')
        ax2.text(np.deg2rad(-180), 1.1, 'W', fontsize=12, ha='center')
        ax2.text(np.deg2rad(-270), 1.1, 'N', fontsize=12, ha='center')

        text_radius = 0.5  # Adjust the text radius
        text_x = text_radius * np.cos(theta)
        text_y = text_radius * np.sin(theta)
        ax2.text(text_x, text_y, f'{angle}°', fontsize=18, ha='center')
        ax2.set_yticklabels([])
        ax2.set_xticklabels([])

    def on_close(self):
        try:
            if self.ser.isOpen():
                self.ser.close()
            self.root_tk.destroy()
        except Exception as ex:
            print("Error closing application:", ex)

if __name__ == "__main__":
    mv = MapAndView(track_len=10)
