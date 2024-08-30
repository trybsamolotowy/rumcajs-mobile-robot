import tkinter as tk
from tkinter import PhotoImage
import cv2
from PIL import Image, ImageTk
import os
import sys
import time
from odrive.enums import (AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL)
import argparse
import sys
import time
import odrive
from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL,
                          AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION,
                          AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
                          AXIS_STATE_IDLE, AXIS_STATE_MOTOR_CALIBRATION,
                          CONTROL_MODE_POSITION_CONTROL, ENCODER_MODE_HALL)


class HBMotorConfig:
    """Class for configuring an Odrive axis for a Hoverboard motor.

    Only works with one Odrive at a time.
    """

    # Hoverboard Kv
    HOVERBOARD_KV = 16.0

    # Min/Max phase inductance of motor
    MIN_PHASE_INDUCTANCE = 0
    MAX_PHASE_INDUCTANCE = 0.001

    # Min/Max phase resistance of motor
    MIN_PHASE_RESISTANCE = 0
    MAX_PHASE_RESISTANCE = 2

    # Tolerance for encoder offset float
    ENCODER_OFFSET_FLOAT_TOLERANCE = 0.05

    def __init__(
        self, axis_num: int, anticogging_cal: bool, erase_config: bool
    ) -> None:
        """Init HBMotorConfig object."""
        self.axis_num = axis_num
        self.anticogging_cal = anticogging_cal
        self.erase_config = erase_config

              

    def _find_odrive(self) -> None:
        # connect to Odrive
        # print("Looking for ODrive...")
        self.odrv = odrive.find_any()
        self.odrv_axis = getattr(self.odrv, "axis{}".format(self.axis_num))
        # print("Found ODrive.")

    def configure(self) -> None:
        """Configure the odrive device for a hoverboard motor."""
        if self.erase_config:
            # Erase pre-exsisting configuration
            print("Erasing pre-exsisting configuration...")
            try:
                self.odrv.erase_configuration()
            except Exception:
                pass

        self._find_odrive()

        # Set this to True if using a brake resistor
        self.odrv.config.enable_brake_resistor = True

        # This is the resistance of the brake resistor. You can leave this
        # at the default setting if you are not using a brake resistor. Note
        # that there may be some extra resistance in your wiring and in the
        # screw terminals, so if you are getting issues while braking you may
        # want to increase this parameter by around 0.05 ohm.
        # self.odrv_axis.config.brake_resistance  = 2.0

        # This is the amount of current allowed to flow back into the power supply.
        # The convention is that it is negative. By default, it is set to a
        # conservative value of 10mA. If you are using a brake resistor and getting
        # DC_BUS_OVER_REGEN_CURRENT errors, raise it slightly. If you are not using
        # a brake resistor and you intend to send braking current back to the power
        # supply, set this to a safe level for your power source. Note that in that
        # case, it should be higher than your motor current limit + current limit
        # margin.
        # self.odrv_axis.config.dc_max_negative_current  = -0.01

        # Standard 6.5 inch hoverboard hub motors have 30 permanent magnet
        # poles, and thus 15 pole pairs
        self.odrv_axis.motor.config.pole_pairs = 15

        # Hoverboard hub motors are quite high resistance compared to the hobby
        # aircraft motors, so we want to use a bit higher voltage for the motor
        # calibration, and set up the current sense gain to be more sensitive.
        # The motors are also fairly high inductance, so we need to reduce the
        # bandwidth of the current controller from the default to keep it
        # stable.
        self.odrv_axis.motor.config.resistance_calib_max_voltage = 10 #4
        self.odrv_axis.motor.config.requested_current_range = 25
        self.odrv_axis.motor.config.current_control_bandwidth = 100

        # Estimated KV but should be measured using the "drill test", which can
        # be found here:
        # https://discourse.odriverobotics.com/t/project-hoverarm/441
        self.odrv_axis.motor.config.torque_constant = 8.27 / self.HOVERBOARD_KV

        # Hoverboard motors contain hall effect sensors instead of incremental
        # encorders
        self.odrv_axis.encoder.config.mode = ENCODER_MODE_HALL

        # The hall feedback has 6 states for every pole pair in the motor. Since
        # we have 15 pole pairs, we set the cpr to 15*6 = 90.
        self.odrv_axis.encoder.config.cpr = 90

        # Since hall sensors are low resolution feedback, we also bump up the
        # offset calibration displacement to get better calibration accuracy.
        self.odrv_axis.encoder.config.calib_scan_distance = 150

        # Since the hall feedback only has 90 counts per revolution, we want to
        # reduce the velocity tracking bandwidth to get smoother velocity
        # estimates. We can also set these fairly modest gains that will be a
        # bit sloppy but shouldn’t shake your rig apart if it’s built poorly.
        # Make sure to tune the gains up when you have everything else working
        # to a stiffness that is applicable to your application.
        self.odrv_axis.encoder.config.bandwidth = 100
        self.odrv_axis.controller.config.pos_gain = 6
        self.odrv_axis.controller.config.vel_gain = (
            0.02
            * self.odrv_axis.motor.config.torque_constant
            * self.odrv_axis.encoder.config.cpr
        )
        self.odrv_axis.controller.config.vel_integrator_gain = (
            0.1
            * self.odrv_axis.motor.config.torque_constant
            * self.odrv_axis.encoder.config.cpr
        )
        self.odrv_axis.controller.config.vel_limit = 10

        # Set in position control mode so we can control the position of the
        # wheel
        self.odrv_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        # In the next step we are going to start powering the motor and so we
        # want to make sure that some of the above settings that require a
        # reboot are applied first.

        # Motors must be in IDLE mode before saving
        self.odrv_axis.requested_state = AXIS_STATE_IDLE
        try:
            print("Saving manual configuration and rebooting...")
            is_saved = self.odrv.save_configuration()
            if not is_saved:
                print("Error: Configuration not saved. Are all motors in IDLE state?")
            else:
                print("Calibration configuration saved.")

            print("Manual configuration saved.")
        except Exception:
            pass

        self._find_odrive() 

        # input("Make sure the motor is free to move, then press enter...")

        print("Calibrating Odrive for hoverboard motor (you should hear a " "beep)...")

        self.odrv_axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION

        # Wait for calibration to take place
        time.sleep(10)

        if self.odrv_axis.motor.error != 0:
            print(
                "Error: Odrive reported an error of {} while in the state "
                "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
                "debug:\n{}".format(self.odrv_axis.motor.error, self.odrv_axis.motor)
            )

            sys.exit(1)

        if (
            self.odrv_axis.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE
            or self.odrv_axis.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE
        ):
            print(
                "Error: After odrive motor calibration, the phase inductance "
                "is at {}, which is outside of the expected range. Either widen the "
                "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
                "is between {} and {} respectively) or debug/fix your setup. Printing "
                "out Odrive motor data for debug:\n{}".format(
                    self.odrv_axis.motor.config.phase_inductance,
                    self.MIN_PHASE_INDUCTANCE,
                    self.MAX_PHASE_INDUCTANCE,
                    self.odrv_axis.motor,
                )
            )

            sys.exit(1)

        if (
            self.odrv_axis.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE
            or self.odrv_axis.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE
        ):
            print(
                "Error: After odrive motor calibration, the phase resistance "
                "is at {}, which is outside of the expected range. Either raise the "
                "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
                "debug/fix your setup. Printing out Odrive motor data for "
                "debug:\n{}".format(
                    self.odrv_axis.motor.config.phase_resistance,
                    self.MIN_PHASE_RESISTANCE,
                    self.MAX_PHASE_RESISTANCE,
                    self.odrv_axis.motor,
                )
            )

            sys.exit(1)

        # If all looks good, then lets tell ODrive that saving this calibration
        # to persistent memory is OK
        self.odrv_axis.motor.config.pre_calibrated = True

        # Check the alignment between the motor and the hall sensor. Because of
        # this step you are allowed to plug the motor phases in random order and
        # also the hall signals can be random. Just don’t change it after
        # calibration.
        print("Calibrating Odrive for hall encoder...")
        self.odrv_axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION

        # Wait for calibration to take place
        time.sleep(15)

        if self.odrv_axis.encoder.error != 0:
            print(
                "Error: Odrive reported an error of {} while in the state "
                "AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION. Printing out Odrive "
                "encoder data for debug:\n{}".format(
                    self.odrv_axis.encoder.error, self.odrv_axis.encoder
                )
            )

            sys.exit(1)

        print("Calibrating Odrive for encoder offset...")
        self.odrv_axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        # Wait for calibration to take place
        time.sleep(30)

        if self.odrv_axis.encoder.error != 0:
            print(
                "Error: Odrive reported an error of {} while in the state "
                "AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
                "data for debug:\n{}".format(
                    self.odrv_axis.encoder.error, self.odrv_axis.encoder
                )
            )

            sys.exit(1)

        # If all looks good, then lets tell ODrive that saving this calibration
        # to persistent memory is OK
        self.odrv_axis.encoder.config.pre_calibrated = True

        if self.anticogging_cal:
            print("Calibrating Odrive for anticogging...")
            self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            self.odrv_axis.controller.start_anticogging_calibration()

            while self.odrv_axis.controller.config.anticogging.calib_anticogging:
                time.sleep(15)
                print("Still calibrating anticogging...")

            if self.odrv_axis.controller.error != 0:
                print(
                    "Error: Odrive reported an error of {} while performing "
                    "start_anticogging_calibration(). Printing out Odrive controller "
                    "data for debug:\n{}".format(
                        self.odrv_axis.controller.error, self.odrv_axis.controller
                    )
                )

                sys.exit(1)

            # If all looks good, then lets tell ODrive that saving this calibration
            # to persistent memory is OK
            self.odrv_axis.controller.config.anticogging.pre_calibrated = True

        # Motors must be in IDLE mode before saving
        self.odrv_axis.requested_state = AXIS_STATE_IDLE
        try:
            print("Saving calibration configuration and rebooting...")
            self.odrv.save_configuration()
            if not is_saved:
                print("Error: Configuration not saved. Are all motors in IDLE state?")
            else:
                print("Calibration configuration saved.")
        except Exception:
            pass

        self._find_odrive()

        print("Odrive configuration finished.")

    def mode_idle(self) -> None:
        """Put the motor in idle (i.e. can move freely)."""
        self.odrv_axis.requested_state = AXIS_STATE_IDLE

    def mode_close_loop_control(self) -> None:
        """Put the motor in closed loop control."""
        self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def move_input_pos(self, angle: float) -> None:
        """Put the motor at a certain angle."""
        self.odrv_axis.controller.input_pos = angle / 360.0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hoverboard Motor Calibration")

    # Argument for axis_num
    parser.add_argument(
        "--axis_num",
        type=int,
        choices=[0, 1],  # Only allow 0 or 1
        required=True,
        help="Motor axis number to perform calibration on (only 0 or 1).",
    )

    # Argument for erase_config
    parser.add_argument(
        "--erase_config",
        action="store_true",  # If present, set to True. If absent, set to False.
        help="Erase ODrive board's current configuration.",
    )

    # Argument to perform anticogging calibration
    parser.add_argument(
        "--anticogging_cal",
        action="store_true",  # If present, set to True. If absent, set to False.
        help="Calibrate for anti-cogging.",
    )

    # Argument to conduct motor test (make sure motor can move freely)
    parser.add_argument(
        "--motor_test",
        action="store_true",  # If present, set to True. If absent, set to False.
        help="Test conducted at end of clibration to move motor in 30 deg increments.",
    )

    args = parser.parse_args()

    hb_motor_config = HBMotorConfig(
        axis_num=args.axis_num,
        anticogging_cal=args.anticogging_cal,
        erase_config=args.erase_config,
    )
    # hb_motor_config.configure()

    if args.motor_test:
        print("Placing motor in close loop. If you move motor, motor will resist you.")
        hb_motor_config.mode_close_loop_control()

        print("CONDUCTING MOTOR TEST")

        # Go from 0 to 360 degrees in increments of 30 degrees
        for angle in range(0, 450, 90):
            print("Setting motor to {} degrees.".format(angle))
            hb_motor_config.move_input_pos(angle)
            time.sleep(30)

        print("Placing motor in idle. If you move motor, motor will move freely")
        hb_motor_config.mode_idle()

        if self.odrv_axis.motor.error != 0:
            print(f"Motor Error: {self.odrv_axis.motor.error}")
            print(f"Axis Error: {self.odrv_axis.error}")
            sys.exit(1)



# GUI


root = tk.Tk()
root.title("Rumcajs")
root.configure(bg="#ffffff")

# Ustawienia pełnoekranowe
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
root.geometry(f"{screen_width}x{screen_height}")

cap = cv2.VideoCapture(2)  # ZED

# Ustawienia folderów
photos_dir = "photos_captured"
left_folder_photos = os.path.join(photos_dir, "L")
right_folder_photos = os.path.join(photos_dir, "R")
def create_folder_if_not_exists(folder):
    if not os.path.exists(folder):
        os.makedirs(folder)

create_folder_if_not_exists(photos_dir)
create_folder_if_not_exists(left_folder_photos)
create_folder_if_not_exists(right_folder_photos)

films_dir = "films_captured"
left_folder_films = os.path.join(films_dir, "L")
right_folder_films = os.path.join(films_dir, "R")

create_folder_if_not_exists(films_dir)
create_folder_if_not_exists(left_folder_films)
create_folder_if_not_exists(right_folder_films)

# Funkcje do zarządzania plikami
def get_next_filename(folder, prefix, extension):
    files = [f for f in os.listdir(folder) if f.endswith(extension)]
    if files:
        max_num = max([int(f.split('_')[0]) for f in files])
        return f"{max_num + 1}_{prefix}.{extension}"
    else:
        return f"1_{prefix}.{extension}"

# Klasa zarządzająca kalibracją i sterowaniem silnikiem
class MotorController:
    def __init__(self, axis_num=0, anticogging_cal=False, erase_config=False):
        self.hb_motor_config = HBMotorConfig(axis_num, anticogging_cal, erase_config)
        # self.hb_motor_config.configure()
    
    def calibrate_motor(self):
        self.hb_motor_config.configure()

    def move_motor(self, angle):
        self.hb_motor_config.move_input_pos(angle)

    def start_motor(self):
        self.hb_motor_config.mode_close_loop_control()

    def stop_motor(self):
        self.hb_motor_config.mode_idle()

motor_controller = MotorController()

# Funkcje do sterowania robotem i zapisywania plików
def turn_right():
    pass

def turn_left():
    pass

move_task = None  # Global variable to track the movement task
current_position = 0  # Global variable to track the current motor position

def go_straight():
    motor_controller.start_motor()

    def continuous_move():
        global current_position
        global move_task
        current_position += 30  # Increase position for continuous movement
        motor_controller.move_motor(current_position)  # Update motor position
        move_task = root.after(10, continuous_move)  # Repeat every 10 ms

    continuous_move()

def stop_motor(event=None):
    global move_task
    if move_task is not None:
        root.after_cancel(move_task)
        move_task = None
    motor_controller.stop_motor()

def go_back():
    motor_controller.start_motor()

    def continuous_move():
        global current_position
        global move_task
        current_position -= 30  # Increase position for continuous movement
        motor_controller.move_motor(current_position)  # Update motor position
        move_task = root.after(10, continuous_move)  # Repeat every 10 ms

    continuous_move()


def save_image():
    ret, frame = cap.read()
    if ret:
        height, width = frame.shape[:2]
        mid = width // 2
        left_frame = frame[:, :mid]
        right_frame = frame[:, mid:]
        
        # Zapis obrazów z podzielonego kadru
        filename_left = get_next_filename(left_folder_photos, "L", "jpg")
        filename_right = get_next_filename(right_folder_photos, "R", "jpg")
        cv2.imwrite(os.path.join(left_folder_photos, filename_left), left_frame)
        cv2.imwrite(os.path.join(right_folder_photos, filename_right), right_frame)
        print(f"Saved {filename_left} and {filename_right}")

def start_filming():
    global out_left, out_right, filming
    ret, frame = cap.read()
    if ret:
        height, width = frame.shape[:2]
        mid = width // 2

        # Przygotowanie do nagrywania
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        filename_left = get_next_filename(os.path.join(films_dir, "L"), "L", "avi")
        filename_right = get_next_filename(os.path.join(films_dir, "R"), "R", "avi")
        out_left = cv2.VideoWriter(os.path.join(films_dir, "L", filename_left), fourcc, 20.0, (mid, height))
        out_right = cv2.VideoWriter(os.path.join(films_dir, "R", filename_right), fourcc, 20.0, (mid, height))

        filming = True
        print(f"Started filming {filename_left} and {filename_right}")

def end_filming():
    global filming
    if filming:
        filming = False
        out_left.release()
        out_right.release()
        print("Stopped filming")


arrow_left = PhotoImage(file="arrow_left.png")  
arrow_right = PhotoImage(file="arrow_right.png")  
arrow_up = PhotoImage(file="arrow_up.png")  
arrow_down = PhotoImage(file="arrow_down.png")  

left_camera_label = tk.Label(root, bg="#4682B4")
left_camera_label.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")

right_camera_label = tk.Label(root, bg="#4682B4")
right_camera_label.grid(row=0, column=2, columnspan=2, padx=5, pady=5, sticky="nsew")

start_filming_btn = tk.Button(root, text="START FILMING", command=start_filming, bg="#4682B4", fg="black", font=("Arial", 22))
start_filming_btn.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")


stop_filming_btn = tk.Button(root, text="STOP FILMING", command=end_filming, bg="#4682B4", fg="black", font=("Arial", 22))
stop_filming_btn.grid(row=1, column=1, padx=5, pady=5, sticky="nsew")


save_photo_btn = tk.Button(root, text="SAVE A PHOTO", command=save_image, bg="#4682B4", fg="black", font=("Arial", 22))
save_photo_btn.grid(row=1, column=2, padx=5, pady=5, sticky="nsew")


calibrate_btn = tk.Button(root, text="CALIBRATE MOTOR", command=motor_controller.calibrate_motor, bg="#4682B4", fg="black", font=("Arial", 22))
calibrate_btn.grid(row=1, column=3, padx=5, pady=5, sticky="nsew")


btn_left = tk.Button(root, image=arrow_left, command=turn_left, bg="#FFFFFF")
btn_left.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")

btn_down = tk.Button(root, image=arrow_down, bg="#FFFFFF") #command=go_back, bg="#FFFFFF")
btn_down.grid(row=2, column=1, padx=5, pady=5, sticky="nsew")

btn_up = tk.Button(root, image=arrow_up, bg="#FFFFFF")
btn_up.grid(row=2, column=2, padx=5, pady=5, sticky="nsew")

btn_right = tk.Button(root, image=arrow_right, command=turn_right, bg="#FFFFFF")
btn_right.grid(row=2, column=3, padx=5, pady=5, sticky="nsew")

btn_up.bind("<ButtonPress-1>", lambda event: go_straight())
btn_up.bind("<ButtonRelease>", stop_motor)

btn_down.bind("<ButtonPress-1>", lambda event: go_back())
btn_down.bind("<ButtonRelease>", stop_motor)

# Funkcja zmieniająca rozmiar komponentów w zależności od rozmiaru okna
def resize_components(event):
    global scale_factor
    new_width = root.winfo_width()
    new_height = root.winfo_height()

    scale_factor = min(new_width / screen_width, new_height / screen_height)

    root.grid_rowconfigure(0, weight=1)
    root.grid_rowconfigure(1, weight=1)
    root.grid_rowconfigure(2, weight=2)
    root.grid_columnconfigure(0, weight=1)
    root.grid_columnconfigure(1, weight=1)
    root.grid_columnconfigure(2, weight=1)
    root.grid_columnconfigure(3, weight=1)

root.bind('<Configure>', resize_components)

# Funkcja do aktualizacji obrazu z kamer
def update_frames():
    global filming
    ret, frame = cap.read()
    if ret:
        height, width = frame.shape[:2]
        mid = width // 2

        # Podział obrazu na lewą i prawą część
        left_frame = frame[:, :mid]
        right_frame = frame[:, mid:]
        
        # Aktualizacja lewego obrazu
        img_left = Image.fromarray(cv2.cvtColor(left_frame, cv2.COLOR_BGR2RGB))
        imgtk_left = ImageTk.PhotoImage(image=img_left)
        left_camera_label.imgtk = imgtk_left
        left_camera_label.configure(image=imgtk_left)

        # Aktualizacja prawego obrazu
        img_right = Image.fromarray(cv2.cvtColor(right_frame, cv2.COLOR_BGR2RGB))
        imgtk_right = ImageTk.PhotoImage(image=img_right)
        right_camera_label.imgtk = imgtk_right
        right_camera_label.configure(image=imgtk_right)

        # Zapis do plików wideo
        if filming:
            out_left.write(left_frame)
            out_right.write(right_frame)

    # Aktualizacja po 10 ms
    root.after(10, update_frames)

# Zmienna do kontrolowania stanu nagrywania
filming = False

# Inicjalne ustawienie rozmiaru komponentów
resize_components(None)

# Rozpoczęcie aktualizacji obrazu z kamer
update_frames()

root.mainloop()

# Zwolnienie zasobów kamery po zamknięciu aplikacji
cap.release()
cv2.destroyAllWindows()
