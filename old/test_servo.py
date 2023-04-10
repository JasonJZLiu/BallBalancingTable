# import RPi.GPIO as GPIO

# import time
# from time import sleep


# GPIO.setmode(GPIO.BOARD)


# GPIO.setup(11, GPIO.OUT)
# pwm = GPIO.PWM(11, 50)


# pwm.start(0)


# while(True):
#     pwm.ChangeDutyCycle(10)
#     sleep(1)

#     pwm.ChangeDutyCycle(4.5)
#     sleep(1)


# pwm.stop()



from gpiozero import AngularServo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()

servo2 = AngularServo(17, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)
servo1 = AngularServo(27, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)

# servo1 = AngularServo(17, min_angle=-43, max_angle=43, pin_factory=factory)


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



while True:

    servo_1_send(servo1, 0)
    servo_2_send(servo2, 0)
    sleep(1)

    servo_1_send(servo1, -37)
    servo_2_send(servo2, -37)
    sleep(1)

    servo_1_send(servo1, 37)
    servo_2_send(servo2, 37)
    sleep(1)

    