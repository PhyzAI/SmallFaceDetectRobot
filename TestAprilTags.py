from picamera2 import Picamera2
from pupil_apriltags import Detector
import cv2

def main():
    # Initialize the Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    # Initialize the AprilTag detector for the tag36h11 family
    detector = Detector(families="tag36h11", nthreads=2, quad_decimate=2.0, quad_sigma=0.0)

    print("Press Ctrl+C to quit.")

    try:
        while True:
            # Capture a frame from the Pi Camera
            frame = picam2.capture_array()
            #frame = cv2.flip(frame, 0)

            # Convert the frame to grayscale (required by the detector)
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            # Detect AprilTags in the frame
            tags = detector.detect(gray)

            # Draw detections on the frame
            for tag in tags:
                corners = [(int(pt[0]), int(pt[1])) for pt in tag.corners]
                for i in range(4):
                    cv2.line(frame, corners[i], corners[(i + 1) % 4], (0, 255, 0), 2)

                # Draw the tag ID at the center
                center = (int(tag.center[0]), int(tag.center[1]))
                cv2.putText(frame, f"ID: {tag.tag_id}", (center[0] - 20, center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Display the frame
            cv2.imshow("AprilTag Detection - 36h11", frame)

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Release resources
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()