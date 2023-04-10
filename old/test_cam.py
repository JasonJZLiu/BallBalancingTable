# import pyrealsense2 as rs
import numpy as np
import cv2


# class RealSenseCapture:
#     def __init__(self):
#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.color, 320, 240, rs.format.bgr8, 30)
#         profile = self.pipeline.start(config)
    
#     def read(self):
#         frames = self.pipeline.wait_for_frames()
#         color_frame = frames.get_color_frame()
#         color_image = np.asanyarray(color_frame.get_data())
#         return np.flip(color_image, -1).copy()



# if __name__ == '__main__':
#     capture = RealSenseCapture()
#     while True:
#         img = capture.read()
#         # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
#         cv2.imshow("2d", img)
#         cv2.waitKey(1)



# import the opencv library
import cv2
  
  
# define a video capture object
vid = cv2.VideoCapture(2)
  
while(True):
      
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
    # print(frame.shape)
  
    # Display the resulting frame
    cv2.imshow('frame', frame)
      
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
