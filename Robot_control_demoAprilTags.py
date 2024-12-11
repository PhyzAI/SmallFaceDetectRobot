# Detect April Tags
# RKD - 2024.12.04


import cv2
from picamera2 import Picamera2
import libcamera
#from pyapriltags import Detector
from pupil_apriltags import Detector


debug = True  # Show extra info

# Camera Image Parameters
image_scale = 1.0
image_dim = (640,480)


########################### MAIN #######################
    
def run_robot():
            
    # Set up the PiCamera
    camera = Picamera2()
    #pprint(camera.sensor_modes)
    preview_config = camera.create_preview_configuration()
    #preview_config['transform'] = libcamera.Transform(vflip=1, hflip=0)
    preview_config['main']={"format": 'RGB888', "size": image_dim} # 1296, 927; (1280, 960)}  #1296,972
    camera.configure(preview_config)
    camera.start(show_preview=False)
    
    
    detector = Detector(families = "tag36h11") #, quad_decimate=2.0)
    
    while True:
        
        # get an image from the camera
        image = camera.capture_array()
    
        image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        
        tags = detector.detect(image_gray)
        if len(tags) > 0:
            print("# tags: ", len(tags))
             
        
        for tag in tags:
            
            corners = [(int(pt[0]), int(pt[1])) for pt in tag.corners]
            for i in range(4):
                cv2.line(image, corners[i], corners[(i + 1) % 4], (0, 255, 0), 2)

            # Draw the tag ID near the center
            center = (int(tag.center[0]), int(tag.center[1]))
            cv2.putText(image, f"ID: {tag.tag_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Display the frame
        image = cv2.flip(image, 0) # Can't flip at top of loop for some odd reason
        cv2.imshow("Camera", image)                

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()



if __name__ == "__main__":
    run_robot()

