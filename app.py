from flask import Flask, render_template, request, Response, redirect, url_for
import odrive
import time
import cv2
import os
import sys
from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL,
                          AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION,
                          AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
                          AXIS_STATE_IDLE, AXIS_STATE_MOTOR_CALIBRATION,
                          CONTROL_MODE_POSITION_CONTROL, ENCODER_MODE_HALL)

app = Flask(__name__)

# ODrive setup and motor configuration
class HBMotorConfig:
    HOVERBOARD_KV = 16.0
    MIN_PHASE_INDUCTANCE = 0
    MAX_PHASE_INDUCTANCE = 0.001
    MIN_PHASE_RESISTANCE = 0
    MAX_PHASE_RESISTANCE = 2
    ENCODER_OFFSET_FLOAT_TOLERANCE = 0.05

    def __init__(self, axis_num: int, anticogging_cal: bool, erase_config: bool) -> None:
        self.axis_num = axis_num
        self.anticogging_cal = anticogging_cal
        self.erase_config = erase_config
        self._find_odrive()
        
    def _find_odrive(self) -> None:
        self.odrv = odrive.find_any()
        self.odrv_axis = getattr(self.odrv, f"axis{self.axis_num}")
        print("Odrive found")

    def configure(self) -> None:
        if self.erase_config:
            print("Erasing pre-existing configuration...")
            try:
                self.odrv.erase_configuration()
            except Exception:
                pass

        self._find_odrive()
        self.odrv.config.enable_brake_resistor = True
        self.odrv_axis.motor.config.pole_pairs = 15
        self.odrv_axis.motor.config.resistance_calib_max_voltage = 10
        self.odrv_axis.motor.config.requested_current_range = 25
        self.odrv_axis.motor.config.current_control_bandwidth = 100
        self.odrv_axis.motor.config.torque_constant = 8.27 / self.HOVERBOARD_KV
        self.odrv_axis.encoder.config.mode = ENCODER_MODE_HALL
        self.odrv_axis.encoder.config.cpr = 90
        self.odrv_axis.encoder.config.calib_scan_distance = 150
        self.odrv_axis.encoder.config.bandwidth = 100
        self.odrv_axis.controller.config.pos_gain = 6
        self.odrv_axis.controller.config.vel_gain = (
            0.02 * self.odrv_axis.motor.config.torque_constant * self.odrv_axis.encoder.config.cpr
        )
        self.odrv_axis.controller.config.vel_integrator_gain = (
            0.1 * self.odrv_axis.motor.config.torque_constant * self.odrv_axis.encoder.config.cpr
        )
        self.odrv_axis.controller.config.vel_limit = 10
        self.odrv_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

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

        print("Calibrating Odrive (you should hear a beep)...")

        self.odrv_axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        time.sleep(10)

        if self.odrv_axis.motor.error != 0:
            print(f"Error: Odrive reported an error of {self.odrv_axis.motor.error} while in the state AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for debug:\n{self.odrv_axis.motor}")
            sys.exit(1)

        if (self.odrv_axis.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE
            or self.odrv_axis.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE):
            print(f"Error: After odrive motor calibration, the phase inductance is at {self.odrv_axis.motor.config.phase_inductance}, which is outside of the expected range. Printing out Odrive motor data for debug:\n{self.odrv_axis.motor}")
            sys.exit(1)

        if (self.odrv_axis.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE
            or self.odrv_axis.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE):
            print(f"Error: After odrive motor calibration, the phase resistance is at {self.odrv_axis.motor.config.phase_resistance}, which is outside of the expected range. Printing out Odrive motor data for debug:\n{self.odrv_axis.motor}")
            sys.exit(1)

        self.odrv_axis.motor.config.pre_calibrated = True

        print("Calibrating Odrive for hall encoder...")
        self.odrv_axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
        time.sleep(15)

        if self.odrv_axis.encoder.error != 0:
            print(f"Error: Odrive reported an error of {self.odrv_axis.encoder.error} while in the state AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION. Printing out Odrive encoder data for debug:\n{self.odrv_axis.encoder}")
            sys.exit(1)

        print("Calibrating Odrive for encoder offset...")
        self.odrv_axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        time.sleep(30)

        if self.odrv_axis.encoder.error != 0:
            print(f"Error: Odrive reported an error of {self.odrv_axis.encoder.error} while in the state AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder data for debug:\n{self.odrv_axis.encoder}")
            sys.exit(1)

        self.odrv_axis.encoder.config.pre_calibrated = True

        if self.anticogging_cal:
            print("Calibrating Odrive for anticogging...")
            self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            self.odrv_axis.controller.start_anticogging_calibration()

            while self.odrv_axis.controller.config.anticogging.calib_anticogging:
                time.sleep(15)
                print("Still calibrating anticogging...")

            if self.odrv_axis.controller.error != 0:
                print(f"Error: Odrive reported an error of {self.odrv_axis.controller.error} while performing start_anticogging_calibration(). Printing out Odrive controller data for debug:\n{self.odrv_axis.controller}")
                sys.exit(1)

            self.odrv_axis.controller.config.anticogging.pre_calibrated = True

        self.odrv_axis.requested_state = AXIS_STATE_IDLE
        try:
            print("Saving calibration configuration and rebooting...")
            self.odrv.save_configuration()
        except Exception:
            pass

        self._find_odrive()
        print("Odrive configuration finished.")

    def mode_idle(self) -> None:
        self.odrv_axis.requested_state = AXIS_STATE_IDLE

    def mode_close_loop_control(self) -> None:
        self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def move_input_pos(self, angle: float) -> None:
        self.odrv_axis.controller.input_pos = angle / 360.0


# Initialize motor controller
motor_controller = HBMotorConfig(axis_num=0, anticogging_cal=False, erase_config=False)

# Camera setup
cap = cv2.VideoCapture(0)  # 0 for default webcam
photos_dir = "photos_captured"
films_dir = "films_captured"
left_folder_photos = os.path.join(photos_dir, "L")
right_folder_photos = os.path.join(photos_dir, "R")
left_folder_films = os.path.join(films_dir, "L")
right_folder_films = os.path.join(films_dir, "R")

def create_folder_if_not_exists(folder):
    if not os.path.exists(folder):
        os.makedirs(folder)

create_folder_if_not_exists(photos_dir)
create_folder_if_not_exists(left_folder_photos)
create_folder_if_not_exists(right_folder_photos)
create_folder_if_not_exists(films_dir)
create_folder_if_not_exists(left_folder_films)
create_folder_if_not_exists(right_folder_films)

out_left = None
out_right = None
filming = False
current_position = 0

# Motor control wrapper
class MotorController:
    def __init__(self):
        self.hb_motor_config = motor_controller
    
    def calibrate_motor(self):
        self.hb_motor_config.configure()

    def move_motor(self, angle):
        self.hb_motor_config.move_input_pos(angle)

    def start_motor(self):
        self.hb_motor_config.mode_close_loop_control()

    def stop_motor(self):
        self.hb_motor_config.mode_idle()

motor_controller = MotorController()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/calibrate_motor')
def calibrate_motor():
    motor_controller.calibrate_motor()
    return redirect(url_for('index'))

@app.route('/move', methods=['POST'])
def move():
    global current_position
    action = request.form['action']

    if action == 'start_forward':
        motor_controller.start_motor()
        while True:
            current_position += 30  # Adjust this value as needed
            motor_controller.move_motor(current_position)
            # time.sleep(0.1)  # Adjust speed as needed
    if action == 'start_backward':
        motor_controller.start_motor()
        while True:
            current_position -= 30  # Adjust this value as needed
            motor_controller.move_motor(current_position)
            # time.sleep(0.1)  # Adjust speed as needed
    if action == 'stop':
        motor_controller.stop_motor()
        
    return '', 204

@app.route('/turn_left')
def turn_left():
    return '', 204

@app.route('/turn_right')
def turn_right():
    return '', 204

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def gen_frames():
    while True:
        success, frame = cap.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/save_image')
def save_image():
    success, frame = cap.read()
    if success:
        height, width = frame.shape[:2]
        mid = width // 2
        left_frame = frame[:, :mid]
        right_frame = frame[:, mid:]
        
        filename_left = f"{len(os.listdir(left_folder_photos)) + 1}_L.jpg"
        filename_right = f"{len(os.listdir(right_folder_photos)) + 1}_R.jpg"
        
        cv2.imwrite(os.path.join(left_folder_photos, filename_left), left_frame)
        cv2.imwrite(os.path.join(right_folder_photos, filename_right), right_frame)
        
    return redirect(url_for('index'))

@app.route('/start_filming')
def start_filming():
    global out_left, out_right, filming
    ret, frame = cap.read()
    if ret:
        height, width = frame.shape[:2]
        mid = width // 2

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        filename_left = f"{len(os.listdir(left_folder_films)) + 1}_L.avi"
        filename_right = f"{len(os.listdir(right_folder_films)) + 1}_R.avi"
        
        out_left = cv2.VideoWriter(os.path.join(left_folder_films, filename_left), fourcc, 20.0, (mid, height))
        out_right = cv2.VideoWriter(os.path.join(right_folder_films, filename_right), fourcc, 20.0, (mid, height))
        
        filming = True
        print(f"Started filming {filename_left} and {filename_right}")
        
    return redirect(url_for('index'))

@app.route('/stop_filming')
def stop_filming():
    global filming
    if filming:
        filming = False
        out_left.release()
        out_right.release()
        print("Stopped filming")
        
    return redirect(url_for('index'))

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
