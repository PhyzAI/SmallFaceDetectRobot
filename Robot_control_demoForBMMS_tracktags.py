# Control mini robot, based on face detection
# RKD - 2024.12.04

# TODO:
# X Remove non type-1 face_detector stuff


import cv2
import dlib
from picamera2 import Picamera2
import libcamera
import math
import time
from RpiMotorLib import RpiMotorLib
import concurrent.futures
import Ultrasonic_distance as ud
import pygame
from pupil_apriltags import Detector


from pprint import *

debug = True  # Show extra info
enable_MC = True  # enable Motor Controls
enable_fwd_back = True  # enable moving forwards/backwards (otherwise just turn)

# Camera Image Parameters
image_scale = 1.0
image_dim = (640,480)

FOV=70 # field of view of the camera


################### Motor stuff ##############

GpioPins_left = [18, 23, 24, 25]
GpioPins_right = [4, 5, 6, 13]

motor_left = RpiMotorLib.BYJMotor("MyMotorOne", "28BYJ")
motor_right = RpiMotorLib.BYJMotor("MyMotorTwo", "28BYJ")
motor_delay = 0.001
motor_mode = "half"
motor_steps_per_rev = 512   # half mode, 4096 steps per revolution????

def turn_left(steps=25, motor_slow_factor=1):
    with concurrent.futures.ThreadPoolExecutor() as executor:
        executor.submit(motor_left.motor_run, GpioPins_left, motor_delay*motor_slow_factor, steps, False, False, motor_mode, .05)
        executor.submit(motor_right.motor_run, GpioPins_right, motor_delay*motor_slow_factor, steps, False, False, motor_mode, .05)

def turn_right(steps=25, motor_slow_factor=1):
    with concurrent.futures.ThreadPoolExecutor() as executor:
        executor.submit(motor_left.motor_run, GpioPins_left, motor_delay*motor_slow_factor, steps, True, False, motor_mode, .05)
        executor.submit(motor_right.motor_run, GpioPins_right, motor_delay*motor_slow_factor, steps, True, False, motor_mode, .05)

def move_forward(steps=50, motor_slow_factor=2):
    with concurrent.futures.ThreadPoolExecutor() as executor:
        executor.submit(motor_left.motor_run, GpioPins_left, motor_delay*motor_slow_factor, steps, False, False, motor_mode, .05)
        executor.submit(motor_right.motor_run, GpioPins_right, motor_delay*motor_slow_factor, steps, True, False, motor_mode, .05)
    
def move_backward(steps=50, motor_slow_factor=1):
    with concurrent.futures.ThreadPoolExecutor() as executor:
        executor.submit(motor_left.motor_run, GpioPins_left, motor_delay*motor_slow_factor, steps, True, False, motor_mode, .05)
        executor.submit(motor_right.motor_run, GpioPins_right, motor_delay*motor_slow_factor, steps, False, False, motor_mode, .05)
 


######### Face Detect ################

def find_closest_face(faces, setpoint_x, setpoint_y):
    face_num = -1
    face_dist = 9999
    current_face = -1
    for rect in faces:
        current_face += 1
        face_mid_x = (rect.left() + rect.right()) // 2
        face_mid_y = (rect.top() + rect.bottom()) // 2
        this_face_dist = math.sqrt((face_mid_x - setpoint_x)**2 +
                         (face_mid_y - setpoint_y)**2)
        if this_face_dist < face_dist:
            face_dist = int(this_face_dist)
            face_num = current_face
    return face_num
    
def check_for_faces(image, face_detector):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    faces = face_detector(image_rgb, 0)
    return faces
    

########################### MAIN #######################
    
def run_robot():
        
    # Set up the PiCamera
    camera = Picamera2()
    preview_config = camera.create_preview_configuration()
    preview_config['main']={"format": 'RGB888', "size": image_dim} # 1296, 927; (1280, 960)}  #1296,972
    camera.configure(preview_config)
    camera.start(show_preview=False)
    
    # Point to move desired tag to
    setpoint_x = image_dim[0]//2
    setpoint_y = image_dim[1]//2
    
    # Search parameters when looking for a tag
    scan_speed = 3
    scan_amount = 25
    
    TAG_DET_TTL = 15  # Number of frames to wait before starting to look for a new tag
    tag_time_to_live = 0  # counter for current face
    
    # Create Tag detector
    detector = Detector(families = "tag36h11") #, quad_decimate=2.0)
    
    clock = pygame.time.Clock()
    
    while True:
        
        clock.tick(30)  # Set frame rate of 30 fps
        
        # get an image from the camera
        image = camera.capture_array()
        image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        
        # Look for tags
        tags = detector.detect(image_gray)
        
        # Display the tags found
        for tag in tags:
            corners = [(int(pt[0]), int(pt[1])) for pt in tag.corners]
            for i in range(4):
                cv2.line(image, corners[i], corners[(i + 1) % 4], (0, 255, 0), 2)

        # Set frame counter in case tag image lost for a moment
        if len(tags) > 0:
            tag_time_to_live = TAG_DET_TTL
        else:
            tag_time_to_live -= 1

       
        if (tag_time_to_live <= 0) and enable_MC:  # No Tag found, search
            turn_left(scan_amount,scan_speed)
                
        elif (len(tags) > 0) and enable_MC: # Turn/move towards the face (or away, if too close)
            tag = tags[0]  # Just use first tag. 
            tag_mid_x = int(tag.center[0])
            tag_mid_y = int(tag.center[1])
                   
            diff_deg = (tag_mid_x - setpoint_x)/setpoint_x * (FOV/2)
            steps = -int((motor_steps_per_rev/360)*diff_deg)
            if debug: print(f"steps: {steps}")
            
            x_points = [int(pt[0]) for pt in tag.corners]
            tag_width = max(x_points) - min(x_points)
            if debug: print("tag width: ", tag_width)
            
            # define how close we can get to the desired face
            tag_width_thresh = 300 #int((235/240)*setpoint_x)   # Determine experimentally
            
            min_steps = 2  # Don't move if close enough
            if steps > min_steps: # Turn to the left
                if debug: print("left: ", steps)
                turn_left(abs(steps))
            elif steps < -min_steps:  # Turn to the right
                if debug: print("right: ", steps)
                turn_right(abs(steps))
            elif tag_width > 1.05*tag_width_thresh: # Move Backwards
                if debug: print("Backwards")
                if enable_fwd_back: move_backward(50,3)
            elif tag_width < tag_width_thresh: # Move Forwards
                if debug: print("Forwards")
                if enable_fwd_back: move_forward(200,1)
            else:
                pass
                    
        # Display the frame
        image = cv2.flip(image, 0) # Can't flip at top of loop for some odd reason
        cv2.imshow("Camera", image)                

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()



if __name__ == "__main__":
    pygame.init()
    run_robot()

