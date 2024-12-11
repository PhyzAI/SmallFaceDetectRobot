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
from pyapriltags import Detector


from pprint import *

debug = True  # Show extra info
enable_MC = False  # enable Motor Controls
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

def check_for_faces(image, face_detector):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    faces = face_detector(image_rgb, 0)
    return faces
    

def find_closest_face(faces, image_mid_x, image_mid_y):
    face_num = -1
    face_dist = 9999
    current_face = -1
    for rect in faces:
        current_face += 1
        face_mid_x = (rect.left() + rect.right()) // 2
        face_mid_y = (rect.top() + rect.bottom()) // 2
        this_face_dist = math.sqrt((face_mid_x - image_mid_x)**2 +
                         (face_mid_y - image_mid_y)**2)
        if this_face_dist < face_dist:
            face_dist = int(this_face_dist)
            face_num = current_face
    return face_num
    

########################### MAIN #######################
    
def run_robot():
    
    face_detector = dlib.get_frontal_face_detector()
        
    # Set up the PiCamera
    camera = Picamera2()
    #pprint(camera.sensor_modes)
    preview_config = camera.create_preview_configuration()
    preview_config['transform'] = libcamera.Transform(vflip=1, hflip=0)
    preview_config['main']={"format": 'RGB888', "size": image_dim} # 1296, 927; (1280, 960)}  #1296,972
    camera.configure(preview_config)
    camera.start(show_preview=False)
    
    # Point to move desired face to
    image_mid_x = image_dim[0]//2
    image_mid_y = image_dim[1]//2
    
    # Search parameters when looking for a face
    scan_speed = 3
    scan_amount = 10
    num_search_steps = 20
    last_search_step = 0  # keep track of where we are in the search

    FACE_DET_TTL = 15  # Number of frames to wait before starting to look for a new face
    face_time_to_live = 0  # counter for current face
    
    clock = pygame.time.Clock()
    
    while True:
        
        clock.tick(30)  # Set frame rate of 30 fps
        
        # Use ultrasonic face_detector to check for distance to the ground
        #cliff_detect_distance = ud.distance()
        cliff_detect_distance = 3
    
        # get an image from the camera
        image = camera.capture_array()
        #image = cv2.resize(image, dsize=None, fx=image_scale, fy=image_scale)

        faces = check_for_faces (image, face_detector)
        if len(faces) > 0:
            face_time_to_live = FACE_DET_TTL
        else:
            face_time_to_live -= 1
       
        if (face_time_to_live <= 0) and enable_MC:  # Start Searching left and right
            print(f"last direction: {last_search_step}")
            if (last_search_step < num_search_steps//4) or (last_search_step >= 3*num_search_steps//4): 
                turn_left(scan_amount,scan_speed)
                if debug: print("searching left")
            else:
                turn_right(scan_amount,scan_speed)
                if debug: print("searching right")
            last_search_step = (last_search_step + 1) % num_search_steps               
                
        elif (len(faces) > 0) and enable_MC: # Turn/move towards the face (or away, if too close)
            closest_face = find_closest_face(faces, image_mid_x, image_mid_y)
            rect = faces[closest_face]
            face_mid_x = (rect.left() + rect.right()) // 2
            face_mid_y = (rect.top() + rect.bottom()) // 2
            face_width = rect.right() - rect.left()
                   
            diff_deg = (face_mid_x - image_mid_x)/image_mid_x * (FOV/2)
            steps = -int((motor_steps_per_rev/360)*diff_deg)
            if debug: print(f"steps: {steps}")
            
            # define how close we can get to the desired face
            face_width_thresh = (150/240)*image_mid_x   # Determine experimentally
            
            min_steps = 2  # Don't move if close enough
            if steps > min_steps: # Turn to the left
                if debug: print("left: ", steps)
                turn_left(abs(steps))
            elif steps < -min_steps:  # Turn to the right
                if debug: print("right: ", steps)
                turn_right(abs(steps))
            elif face_width > face_width_thresh:
                if debug: print("Backwards")
                if enable_fwd_back: move_backward(75,2)
            elif cliff_detect_distance < 10.0 and face_width < face_width_thresh:
                if debug: print("Forwards")
                if enable_fwd_back: move_forward(25,2)
            else:
                pass

        # Draw face rectangles
        for rect in faces:
            cv2.rectangle(image,
                (rect.left(),rect.top()),
                (rect.right(), rect.bottom()),
                (255, 0, 0), 2)
                    
                # Display the frame                
        cv2.imshow("Camera", image)                

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()



if __name__ == "__main__":
    pygame.init()
    run_robot()

