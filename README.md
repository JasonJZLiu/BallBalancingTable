# Vision-Based Ball Balancing Table
A course project for MIE438: Microprocessors and Embedded Microcontrollers at the University of Toronto.

Video Demo: https://www.youtube.com/watch?v=9Cm7HZLUdoY
CAD Files: https://grabcad.com/library/ball-balance-table-1

To run this code:
```
sudo pigpiod
sudo python3 ball_balancing_table.py
```

Control Loop:
- RGB Image captured by a USB webcam at 60 Hz
- Performs HSV Masking, Erosion + Dilation, Contour Finding, and Moments Computation to acquire the ball position using OpenCV
- PID controller to compute the desired position for each servo

Hardware:
- Raspberry Pi 4
- Intel RealSense D405 (or any USB webcam)
- CRAFTSMAN Universal Joint Socket (3/8-Inch Drive)
- 2× MG996R Servos
- 2× 60mm Servo Pull Rod Linkages
