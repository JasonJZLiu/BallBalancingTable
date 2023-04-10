import cv2
import imutils
import time
import signal
import threading
import keyboard
import numpy as np
import RPi.GPIO as GPIO


# blue lego ball HSV mask
HSV_LOWER = (94, 124, 84)
HSV_UPPER = (146, 182, 166)

# default control loop variable
SET_POINT = [170, 180]
DEFAULT_DT = 0.0165

# PID Gains
KP_X = 0.21*1.5 
KD_X = 0.2 * 40 * DEFAULT_DT
KI_X = 0.001
KP_Y = KP_X
KD_Y = KD_X
KI_Y = KI_X

# low pass filter parameter
BETA = 0.5

# FSM
TRACK_MODE = False

# tracking mode
ELAPSED_TIME = 0
REF_CALC_OFFSET = 0.001


####################################################################
#-------------------------- SERVO  UTILS --------------------------#
####################################################################

GPIO.setmode(GPIO.BOARD)
class Servo:
    def __init__(self, pin_num, zero_angle, angle_range, min_pulse_width, max_pulse_width, pwm_freq=50):
        """
        Args:
            pin_num (int): pin number on the Raspberry Pi.
            zero_angle (int): Servo angle that corresponds to the zeroth position
            angle_range (int): Maximum allowable angle magnitude from the zeroth position
            min_pulse_width (float): The minimum pulse width in seconds.
            max_pulse_width (float): The maximum pulse width in seconds.
            pwm_freq (float): The frequency of the PWM, defaulted to 50 Hz. 
        """
        self.angle_range = angle_range
        self.zero_angle = zero_angle
        self.duty_cycle_min = min_pulse_width * pwm_freq * 100
        self.duty_cycle_max = max_pulse_width * pwm_freq * 100
        self.duty_cycle_range = self.duty_cycle_max - self.duty_cycle_min

        GPIO.setup(pin_num, GPIO.OUT)
        self.pwm = GPIO.PWM(pin_num, pwm_freq)
        self.pwm.start(0)

    def send_servo_command(self, angle):
        """
        Args:
            angle (float): Centered at 0 with a range of [-self.angle_range, self.angle_range].
        """
        # clamp the angles to stay within the allowable range
        if angle > self.angle_range:
            angle = self.angle_range
        elif angle < -self.angle_range:
            angle = -self.angle_range
        # convert the angle to the range of 0 to 180
        servo_angle = -1*angle + self.zero_angle
        # compute the corresponding duty cycle
        duty_cycle = (servo_angle / 180) * self.duty_cycle_range + self.duty_cycle_min
        self.pwm.ChangeDutyCycle(duty_cycle)


####################################################################
#---------------------------- FSM  ISR ----------------------------#
####################################################################

def FSM_change_state(sig, frame):
    global SET_POINT
    global BETA
    global TRACK_MODE

    print("\nPlease Type in the Next State. Options: [0, 1, 2, 3, 4, C, rc, 8, r8]")
    next_state = input()

    if next_state == "0":
        TRACK_MODE = False
        SET_POINT = [170, 180]
    elif next_state == '1':
        TRACK_MODE = False
        SET_POINT = [100, 260]
    elif next_state == '2':
        SET_POINT = [250, 250]
    elif next_state == '3':
        SET_POINT = [250, 90]
    elif next_state == '4':
        SET_POINT = [90, 90]
    elif next_state == 'c':
        TRACK_MODE = 'c'
    elif next_state == 'rc':
        TRACK_MODE = 'rc'
    elif next_state == '8':
        TRACK_MODE = '8'
    elif next_state == 'r8':
        TRACK_MODE = 'r8'
    else:
        print("Invalid input!")
 
def on_down_arrow():
    global SET_POINT
    SET_POINT[1] += 5
    if SET_POINT[1] > 250:
        SET_POINT[1] = 250

def on_up_arrow():
    global SET_POINT    
    SET_POINT[1] -= 5
    if SET_POINT[1] < 90:
        SET_POINT[1] = 90

def on_left_arrow():
    global SET_POINT
    SET_POINT[0] -= 5
    if SET_POINT[0] < 90:
        SET_POINT[0] = 90

def on_right_arrow():
    global SET_POINT
    SET_POINT[0] += 5
    if SET_POINT[0] > 250:
        SET_POINT[0] = 250


####################################################################
#-------------------------- CONTORL LOOP --------------------------#
####################################################################

def control_loop():
    # set up servos
    servo_1 = Servo(
        pin_num=11, zero_angle=84, angle_range=37, 
        min_pulse_width=0.0005, max_pulse_width=0.0025,
    )
    servo_2 = Servo(
        pin_num=13, zero_angle=84, angle_range=37, 
        min_pulse_width=0.0005, max_pulse_width=0.0025,
    )
    servo_1.send_servo_command(0)
    servo_2.send_servo_command(0)

    # set up camera stream
    vid = cv2.VideoCapture(2)
    vid.set(cv2.CAP_PROP_FPS, 60)

    # initialize variables
    previous_ball_center = SET_POINT
    previous_u_x = 0
    previous_u_y = 0
    error_x_integral = 0
    error_y_integral = 0
    dt = DEFAULT_DT

    # control loop
    while(True):
        # records the start time of the control loop
        start_time = time.time()
        # read from the camera feed
        ret, frame = vid.read()
        # crops the image to where the top plate is
        frame = frame[80:420, 170:510] # [y, x]
        # applies Gaussian blur filters to remove high frequency noise
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        # all colours except for those close to the ball colour are removed (specified by HSV_LOWER and HSV_UPPER)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        ball_mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        # performs erosion and dilation to remove further filter our noise
        ball_mask = cv2.erode(ball_mask, None, iterations=2)
        ball_mask = cv2.dilate(ball_mask, None, iterations=2)
        # find contours in the mask and define each of the contour's centers
        ball_centers = cv2.findContours(ball_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball_centers = imutils.grab_contours(ball_centers)

        ball_center = None
        if len(ball_centers) > 0:
            # find the largest contour in the mask, then compute its minimum enclosing circle and centroid
            c = max(ball_centers, key=cv2.contourArea)
            # compute the largest contour's moments to find its centroid
            M = cv2.moments(c)
            ball_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # if the system has to track a trajectory, change the setpoint at every iteration
        if TRACK_MODE:
            change_setpoint(dt)

        # if the ball is detected
        if ball_center:
            # compute error signal
            error_x = SET_POINT[0] - ball_center[0]
            error_y = SET_POINT[1] - ball_center[1]
            # compute the integral
            error_x_integral += error_x
            error_y_integral += error_y
            # saturate the integral to prevent windup
            error_x_integral_abs = abs(error_x_integral)
            error_y_integral_abs = abs(error_y_integral)
            if error_x_integral_abs > 2000:
                error_x_integral = (error_x_integral/error_x_integral_abs)*2000
            if error_y_integral_abs > 2000:
                error_y_integral = (error_y_integral/error_y_integral_abs)*2000
            # disable integral if the ball is close to the setpoint or under tracking mode for improved stablity
            if abs(error_x) < 5 or TRACK_MODE:
                error_x_integral = 0
                error_y_integral = 0
            # forward euler numerical differentiation for calculating the ball velocity
            v_x = (ball_center[0] - previous_ball_center[0]) / dt
            v_y = (ball_center[1] - previous_ball_center[1]) / dt
            # compute PID output
            u_x = KP_X * error_x + KD_X * (0 - v_x) + KI_X * error_x_integral
            u_y = KP_Y * error_y + KD_Y * (0 - v_y) + KI_Y * error_y_integral
            # exponential moving average low pass filter
            u_x = BETA * previous_u_x + (1 - BETA) * u_x
            u_y = BETA * previous_u_y + (1 - BETA) * u_y
            # send output to servos
            servo_1.send_servo_command(u_x)
            servo_2.send_servo_command(u_y)
            # store values for the next iteration
            previous_ball_center = ball_center
            previous_u_x = u_x
            previous_u_y = u_y
            dt = time.time() - start_time
        else:
            # reset the table if the ball is not detected
            servo_1.send_servo_command(0)
            servo_2.send_servo_command(0)
            # reset variables
            previous_u_x = 0
            previous_u_y = 0
            error_x_integral = 0
            error_y_integral = 0
            dt = DEFAULT_DT
    
def change_setpoint(dt):
    global ELAPSED_TIME
    global SET_POINT

    if TRACK_MODE == 'c':
        ELAPSED_TIME += dt
        SET_POINT[0] = np.cos(4 * (ELAPSED_TIME + REF_CALC_OFFSET)) * 70 + 170
        SET_POINT[1] = np.sin(4 * (ELAPSED_TIME + REF_CALC_OFFSET)) * 70 + 180
    elif TRACK_MODE == 'rc':
        ELAPSED_TIME -= dt
        SET_POINT[0] = np.cos(4 * (ELAPSED_TIME + REF_CALC_OFFSET)) * 70 + 170
        SET_POINT[1] = np.sin(4 * (ELAPSED_TIME + REF_CALC_OFFSET)) * 70 + 180
    elif TRACK_MODE == "8":
        ELAPSED_TIME += dt
        kt = 3*(ELAPSED_TIME + REF_CALC_OFFSET)
        SET_POINT[0] = np.sin(kt) * 90 + 170
        SET_POINT[1] = np.sin(kt) * np.cos(kt) * 90 + 180
    elif TRACK_MODE == "r8":
        ELAPSED_TIME -= dt
        kt = 3*(ELAPSED_TIME + REF_CALC_OFFSET)
        SET_POINT[0] = np.sin(kt) * 90 + 170
        SET_POINT[1] = np.sin(kt) * np.cos(kt) * 90 + 180


 
if __name__ == "__main__":
    # start the control loop in a seperate thread
    control_thread = threading.Thread(target=control_loop)
    control_thread.start()
    
    # register the FSM ISR
    signal.signal(signal.SIGINT, FSM_change_state)
    # register the arrow key ISR
    keyboard.add_hotkey('up', on_up_arrow)
    keyboard.add_hotkey('down', on_down_arrow)
    keyboard.add_hotkey('left', on_left_arrow)
    keyboard.add_hotkey('right', on_right_arrow)

    control_thread.join()
