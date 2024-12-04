# Control mini robot, based on face detection
# RKD - 2024.12.04

# TODO:
# * Remove non type-1 detector stuff


import cv2
import dlib
from picamera2 import Picamera2
import libcamera
import math
import time
from RpiMotorLib import RpiMotorLib
import concurrent.futures
import Ultrasonic_distance as ud
import threading
import pygame

from pprint import *

debug = True  # Show extra info
enable_gui = True  # Display window of camera view + found faces
enable_MC = True  # enable Motor Controls

# Camera Image Parameters
image_scale = 1.0
image_dim = (640,480)

FOV=75 # field of view of the camera


################ Face Timer #####

face_detected = False
face_timer = []

def clear_face_detected():
    global face_detected
    face_detected = False
    if debug: print("Clearing Face Detected")
    
def start_face_timer():
    global face_detected
    global face_timer
    face_detected=True
    if debug: print("Starting Face Detected Timer")
    try:
        face_timer.cancel()
    except:
        pass
    face_timer=threading.Timer(3, clear_face_detected)
    face_timer.start()


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
    
def check_for_faces(image, detector):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    faces = detector(image_rgb, 0)
    return faces
    

########################### MAIN #######################
    
def run_robot():
    global face_detected  # Controlled by the Timer process
    
    last_direction = 0

    detector = dlib.get_frontal_face_detector()
        
    camera = Picamera2()
    #pprint(camera.sensor_modes)
    
    # Set up the PiCamera and preview
    preview_config = camera.create_preview_configuration()
    preview_config['transform'] = libcamera.Transform(vflip=1, hflip=0)
    preview_config['main']={"format": 'RGB888', "size": image_dim} # 1296, 927; (1280, 960)}  #1296,972
    camera.configure(preview_config)
    camera.start(show_preview=False)
    
    image = camera.capture_array()
    #image = cv2.resize(image, dsize=None, fx=image_scale, fy=image_scale)
    image_mid_x = image.shape[1]//2
    image_mid_y = image.shape[0]//2
    
    # Search parameters when looking for a face
    scan_speed = 3
    scan_amount = 10
    num_search_steps = 20
    
    clock = pygame.time.Clock()
    
    face_time_to_live = 0
    FACE_DET_TTL = 15  # Number of frames to wait before starting to look for a new face
    
    while True:
        
        clock.tick(30)  # Set frame rate of 30 fps
        
        # Use ultrasonic detector to check for distance to the ground.  Part of cliff-detection.
        dist = ud.distance()
    
        # get an image from the camera
        image = camera.capture_array()
        #image = cv2.resize(image, dsize=None, fx=image_scale, fy=image_scale)

        faces = check_for_faces (image, detector)
        if len(faces) > 0:
            face_time_to_live = FACE_DET_TTL
        else:
            face_time_to_live -= 1
       
        if False and (face_time_to_live <= 0) and enable_MC:  # Start Searching left and right
            print(f"last direction: {last_direction}")
            if (last_direction < num_search_steps//4) or (last_direction >= 3*num_search_steps//4): 
                turn_left(scan_amount,scan_speed)
                if debug: print("searching left")
            else:
                turn_right(scan_amount,scan_speed)
                if debug: print("searching right")
            last_direction = (last_direction + 1) % num_search_steps               
                
                
            
                
        elif (len(faces) > 0) and enable_MC:  #FIXME: using face_detected flag causes error.  Race condition?
            closest_face = find_closest_face(faces, image_mid_x, image_mid_y)
            rect = faces[closest_face]
            face_mid_x = (rect.left() + rect.right()) // 2
            face_mid_y = (rect.top() + rect.bottom()) // 2
            face_width = rect.right() - rect.left()
                   
            diff_deg = (face_mid_x - image_mid_x)/image_mid_x * (FOV/2)
            steps = -int((motor_steps_per_rev/360)*diff_deg)
            if debug: print(f"steps: {steps}")
            
            # Check face size (closeness) to see if we should stop moving forward, or move backward
            face_width_thresh = (150/240)*image_mid_x
            #if debug: print(f"face_width: {face_width}, threshold: {face_width_thresh}")
            
            min_steps = 2  # Don't move if close enough
            if steps > min_steps: # Turn to the left
                if debug: print("left: ", steps)
                turn_left(abs(steps))
            elif steps < -min_steps:  # Turn to the right
                if debug: print("right: ", steps)
                turn_right(abs(steps))
            elif face_width > face_width_thresh:
                if debug: print("Backwards")
                move_backward(75,2)
            elif dist < 10.0 and face_width < face_width_thresh:
                if debug: print("Forwards")
                #move_forward(25,2)
            else:
                pass

        if enable_gui:
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

