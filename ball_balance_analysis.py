import cv2
import imutils
import time
import signal
import threading
import numpy as np
import matplotlib.pyplot as plt

from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory

import keyboard

import pickle

# blue wooden ball
# HSV_LOWER = (85, 99, 59)
# HSV_UPPER = (144, 150, 121)


# blue lego ball
# HSV_LOWER = (84, 128, 76)
# HSV_UPPER = (129, 200, 230)

HSV_LOWER = (94, 124, 84)
HSV_UPPER = (146, 182, 166)


X_BUFFER = list()
Y_BUFFER = list()
T_BUFFER = [0]

START_MEASUREMENT = False


SET_POINT = [100, 260]
# SET_POINT = [70, 270]
# SET_POINT = [260, 260]


DEFAULT_DT = 0.0165 #0.034

# kp, ki, kd
KP = 0.21*1.5 
KD = 0.2 * 40 * DEFAULT_DT # 30 # 40
KI = 0.00 # 0.008 for 30 fps

PID_GAINS_X = [KP, KD, KI]
PID_GAINS_Y = [KP, KD, KI]

BETA = 0.5 # 0.35 # 0.7

SERVO1_MAX = 37
SERVO1_MIN = -37

SERVO2_MAX = 37
SERVO2_MIN = -37


TRACK_MODE = False # C, CR
CIRCLE_TIME = 0
CIRCLE_RADIUS = 70
CIRCLE_SPEED_SCALE = 4



def servo_1_send(servo, angle):
    # range is 37
    # servo 1 minimum angle = 31
    # servo 1 zero angle = -6
    # servo 1 maximum angle = -43

    if angle > 37:
        angle = 37
    elif angle < -37:
        angle = -37

    servo.angle = -1*angle - 6


def servo_2_send(servo, angle):
    # range is 37
    # servo 2 minimum angle = 31
    # servo 2 zero angle = -6
    # servo 2 maximum angle = -43

    if angle > 37:
        angle = 37
    elif angle < -37:
        angle = -37

    servo.angle = -1*angle - 6


def change_setpoint(sig, frame):
    global SET_POINT
    global BETA
    global PID_GAINS_X
    global PID_GAINS_Y
    global KP
    global KD
    global KI
    global TRACK_MODE


    print("\nPress 0 for center, 1 to 4 for the corners")
    set_point_select = input()

    if set_point_select == "0":
        TRACK_MODE = False
        BETA = 0.5
        SET_POINT = [170, 180]
        KP = 0.21*1.5
        KD = 0.2 * 40 * DEFAULT_DT # 30 # 40
        KI = 0.00 # 0.008 for 30 fps # 0.004

        PID_GAINS_X = [KP, KD, KI]
        PID_GAINS_Y = [KP, KD, KI]
    elif set_point_select in ['1', '2', '3', '4']:
        TRACK_MODE = False
        if set_point_select == '1':
            SET_POINT = [100, 260]
            BETA = 0.5
            KP = 0.21*1.5
            KD = 0.2 * 40 * DEFAULT_DT # 30 # 40
            KI = 0.00 # 0.008 for 30 fps
        elif set_point_select == '2':
            SET_POINT = [250, 250]
            BETA = 0.5
            KP = 0.21*1.5
            KD = 0.2 * 40 * DEFAULT_DT # 30 # 40
            KI = 0.00 # 0.008 for 30 fps
        elif set_point_select == '3':
            SET_POINT = [250, 90]
            BETA = 0.5
            KP = 0.21*1.5
            KD = 0.2 * 40 * DEFAULT_DT # 30 # 40
            KI = 0.00 # 0.008 for 30 fps
        elif set_point_select == '4':
            SET_POINT = [90, 90]
            BETA = 0.5
            KP = 0.21*1.5
            KD = 0.2 * 40 * DEFAULT_DT # 30 # 40
            KI = 0.00 # 0.008 for 30 fps
        
        PID_GAINS_X = [KP, KD, KI]
        PID_GAINS_Y = [KP, KD, KI]
    elif set_point_select == 'c':
        TRACK_MODE = 'C'
    elif set_point_select == 'rc':
        TRACK_MODE = 'RC'
    elif set_point_select == '8':
        TRACK_MODE = '8'
    elif set_point_select == 'r8':
        TRACK_MODE = 'r8'
    else:
        print("Invalid input!")





 
    





def set_tracking_setpoint(dt):
    global CIRCLE_TIME
    global SET_POINT

    if TRACK_MODE == 'C':
        CIRCLE_TIME += dt
        SET_POINT[0] = np.cos(CIRCLE_SPEED_SCALE*CIRCLE_TIME) * CIRCLE_RADIUS + 170
        SET_POINT[1] = np.sin(CIRCLE_SPEED_SCALE*CIRCLE_TIME) * CIRCLE_RADIUS + 180
    elif TRACK_MODE == 'RC':
        CIRCLE_TIME -= dt
        SET_POINT[0] = np.cos(CIRCLE_SPEED_SCALE*CIRCLE_TIME) * CIRCLE_RADIUS + 170
        SET_POINT[1] = np.sin(CIRCLE_SPEED_SCALE*CIRCLE_TIME) * CIRCLE_RADIUS + 180
    elif TRACK_MODE == "8":
        CIRCLE_TIME += dt
        k=3
        SET_POINT[0] = np.sin(k*CIRCLE_TIME) * CIRCLE_RADIUS * 1.3 + 170
        SET_POINT[1] = np.sin(k*CIRCLE_TIME) * np.cos(k*CIRCLE_TIME) * CIRCLE_RADIUS*1.3 + 180
    elif TRACK_MODE == "r8":
        CIRCLE_TIME -= dt
        k=3
        SET_POINT[0] = np.sin(k*CIRCLE_TIME) * CIRCLE_RADIUS * 1.3 + 170
        SET_POINT[1] = np.sin(k*CIRCLE_TIME) * np.cos(k*CIRCLE_TIME) * CIRCLE_RADIUS*1.3 + 180


    




def control():
    global X_BUFFER
    global Y_BUFFER
    global T_BUFFER

    factory = PiGPIOFactory()

    servo1 = AngularServo(27, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)
    servo2 = AngularServo(17, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)

    servo_1_send(servo1, 0)
    servo_2_send(servo2, 0)

    vid = cv2.VideoCapture(2)
    vid.set(cv2.CAP_PROP_FPS, 60)


    previous_ball_center = SET_POINT
    previous_u_x = 0
    previous_u_y = 0
    error_x_integral = 0
    error_y_integral = 0
    dt = DEFAULT_DT

    while(True):
        start_time = time.time()

        ret, frame = vid.read()
        frame = frame[80:420, 170:510] # y, x

        # print(frame.dtype)

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)

        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # Construct masks for the blue ball 
        ball_mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        ball_mask = cv2.erode(ball_mask, None, iterations=2)
        ball_mask = cv2.dilate(ball_mask, None, iterations=2)


        # Find contours in the mask and define each of the colour's centers
        ball_centers = cv2.findContours(ball_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball_centers = imutils.grab_contours(ball_centers)

        ball_center = None

        if len(ball_centers) > 0:
            # Find the largest contour in the mask, then compute its minimum enclosing circle and centroid
            c = max(ball_centers, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            ball_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # Only assign and draw a center if its radius meets a certain size
            if radius > 8:
                cv2.circle(frame, (int(x), int(y)), int(radius),(0, 0, 255), 1)
                cv2.circle(frame, ball_center, 2, (0, 0, 255), -1)
        

        # cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            



        if TRACK_MODE:
            set_tracking_setpoint(dt)

        


        if ball_center:


            if START_MEASUREMENT:
                X_BUFFER.append(ball_center[0])
                Y_BUFFER.append(ball_center[1])
                T_BUFFER.append(T_BUFFER[-1]+dt)



            error_x = SET_POINT[0] - ball_center[0]
            error_y = SET_POINT[1] - ball_center[1]

    

            error_x_integral += error_x
            error_y_integral += error_y

            # print(error_x, error_y)

            error_x_integral_abs = abs(error_x_integral)
            error_y_integral_abs = abs(error_y_integral)

            if error_x_integral_abs > 2000:
                error_x_integral = (error_x_integral/error_x_integral_abs)*2000
            if error_y_integral_abs > 2000:
                error_y_integral = (error_y_integral/error_y_integral_abs)*2000

            if abs(error_x) < 5 or TRACK_MODE:
                error_x_integral = 0
                error_y_integral = 0

            # print(error_x_integral, error_y_integral)


            v_x = (ball_center[0] - previous_ball_center[0]) / dt
            v_y = (ball_center[1] - previous_ball_center[1]) / dt
            

            u_x = PID_GAINS_X[0] * error_x + PID_GAINS_X[1] * (0 - v_x) + PID_GAINS_X[2] * (error_x_integral) 
            u_y = PID_GAINS_Y[0] * error_y + PID_GAINS_Y[1] * (0 - v_y) + PID_GAINS_Y[2] * (error_y_integral) 

            u_x = BETA * previous_u_x + (1-BETA) * u_x
            u_y = BETA * previous_u_y + (1-BETA) * u_y

            # if abs(error_x) < 5:
            #     u_x = 0
            # if abs(error_y) < 5:
            #     u_y = 0


            servo_1_send(servo1, u_x)
            servo_2_send(servo2, u_y)

            previous_ball_center = ball_center
            previous_u_x = u_x
            previous_u_y = u_y


            dt = time.time() - start_time
            # print(dt)
        
        else:
            servo_1_send(servo1, 0)
            servo_2_send(servo2, 0)
            previous_u_x = 0
            previous_u_y = 0
            error_x_integral = 0
            error_y_integral = 0

            dt = DEFAULT_DT
    
    vid.release()
    cv2.destroyAllWindows()

    servo_1_send(servo1, 0)
    servo_2_send(servo2, 0)
        




def on_down_arrow():
    global SET_POINT
    global CIRCLE_RADIUS
    
    SET_POINT[1] += 5
    if SET_POINT[1] > 250:
        SET_POINT[1] = 250


def on_up_arrow():
    global SET_POINT
    global CIRCLE_RADIUS
    
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


 


if __name__ == "__main__":
    # global SET_POINT
    # global BETA
    # global PID_GAINS_X
    # global PID_GAINS_Y
    # global KP
    # global KD
    # global KI
    # global TRACK_MODE

    # control_thread = threading.Thread(target=control)
    # signal.signal(signal.SIGINT, change_setpoint)

    control_thread = threading.Thread(target=control)

    control_thread.start()


    

    keyboard.add_hotkey('up', on_up_arrow)
    keyboard.add_hotkey('down', on_down_arrow)
    keyboard.add_hotkey('left', on_left_arrow)
    keyboard.add_hotkey('right', on_right_arrow)


    signal.signal(signal.SIGINT, change_setpoint)

    input("READY")

    # change set point to corner 1
    SET_POINT = [100, 260]
    BETA = 0.5
    KP = 0.21*1.5
    KD = 0.2 * 40 * DEFAULT_DT # 30 # 40
    KI = 0.00 # 0.008 for 30 fps
    PID_GAINS_X = [KP, KD, KI]
    PID_GAINS_Y = [KP, KD, KI]

    time.sleep(2)

    # change set point to corner 0
    START_MEASUREMENT = True
    time.sleep(5)

    BETA = 0.5
    SET_POINT = [170, 180]
    KP = 0.21*1.5*0.4
    KD = 0.2 * 40 * DEFAULT_DT # 30 # 40
    KI = 0.001 # 0.008 for 30 fps # 0.004

    PID_GAINS_X = [KP, KD, KI]
    PID_GAINS_Y = [KP, KD, KI]

    time.sleep(10)
    START_MEASUREMENT = False
    print("Done Measurement")


    # print(X_BUFFER)
    # print(len(X_BUFFER))

    # print(len(T_BUFFER))
    # print(T_BUFFER)

    t_buffer = np.asarray(T_BUFFER[0:-1])
    x_buffer = np.asarray(X_BUFFER)
    y_buffer = np.asarray(Y_BUFFER)


    file = open('bbt_analysis', 'wb')
    pickle.dump([t_buffer, x_buffer, y_buffer], file)
    file.close()


    # y_buffer = (y_buffer - 180) * (-1) + 180
    
    x_buffer = (x_buffer - 100)/(170-100)

    y_buffer = (y_buffer - 260)/(180-260)


    


    plt.plot(t_buffer, x_buffer)
    plt.plot(t_buffer, y_buffer)

    plt.show()



















    # Wait for the control thread to finish
    control_thread.join()
