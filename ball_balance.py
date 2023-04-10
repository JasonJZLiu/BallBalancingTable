import cv2
import imutils
import time

from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory


# blue wooden ball
# HSV_LOWER = (85, 99, 59)
# HSV_UPPER = (144, 150, 121)


# blue lego ball
HSV_LOWER = (84, 128, 76)
HSV_UPPER = (129, 170, 143)


# # yellow nerf ball
# HSV_LOWER = (3, 120, 121)
# HSV_UPPER = (34, 194, 244)




SET_POINT = (170, 170)
# SET_POINT = (75, 70)


DEFAULT_DT = 0.0165 #0.034

# kp, ki, kd
KP = 0.21*1.5
KD = 0.2 *40 * DEFAULT_DT # 30 # 40
KI = 0.004 # 0.008 for 30 fps

PID_GAINS_X = [KP, KD, KI]
PID_GAINS_Y = [KP, KD, KI]

BETA = 0.5 # 0.35 # 0.7

SERVO1_MAX = 37
SERVO1_MIN = -37

SERVO2_MAX = 37
SERVO2_MIN = -37



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
    


    if ball_center:

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

        if abs(error_x) < 5:
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