from time import sleep
from LEDcontrol import openLED, closeLED, blink
import cv2
from TFLite_detection_webcam import ObjectDetector, VideoStream
import time
from queue import Queue
from threading import Thread
import RPi.GPIO as GPIO
import pygame

LED1_PIN = 12
LED2_PIN = 22

GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED1_PIN, GPIO.OUT)
GPIO.setup(LED2_PIN, GPIO.OUT)
v = 346
TRIGGER_PIN = 16
ECHO_PIN = 18
GPIO.setup(TRIGGER_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Initialize pygame for keyboard input
pygame.init()
screen = pygame.display.set_mode((100, 100))  # Create a small window to capture keyboard events


# turn on sweep mode (open LED 1)
def sweep(LED1_PIN = LED1_PIN, LED2_PIN = LED2_PIN) -> None:
    openLED(1, LED1_PIN, LED2_PIN)
    sleep(1)
    closeLED(1, LED1_PIN, LED2_PIN)

# turn on mop mode (open LED 2)
def mop(LED1_PIN = LED1_PIN, LED2_PIN = LED2_PIN) -> None:
    openLED(2, LED1_PIN, LED2_PIN)
    sleep(1)
    closeLED(2, LED1_PIN, LED2_PIN)

def objectDetection(detection_queue):
    detector = ObjectDetector(modeldir='custom_model_lite', threshold=0.5)
    videostream = VideoStream(resolution=(detector.imW, detector.imH), framerate=30).start()
    time.sleep(1)
    while True:
        # Read a frame from the video stream
        frame = videostream.read()
        
        # Detect objects in the frame
        detections = detector.detect_objects(frame)
        
        # Put the detection results into the queue
        detection_queue.put(detections)
        
        # Process and print the detections
        for detection in detections:
            print(f"Detected {detection['name']} with confidence {detection['score']}")
            xmin, ymin, xmax, ymax = detection['box']
            name = detection['name']
            # Determine mode (sweep / mop / none)
            if name == 'Plate':
                sweep(LED1_PIN, LED2_PIN)
            elif name == 'Flat Tile Round':
                mop(LED1_PIN, LED2_PIN)
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)
            label = f"{detection['name']}: {int(detection['score'] * 100)}%"
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            label_ymin = max(ymin, labelSize[1] + 10)
            cv2.rectangle(frame, (xmin, label_ymin - labelSize[1] - 10), (xmin + labelSize[0], label_ymin + baseLine - 10), (255, 255, 255), cv2.FILLED)
            cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        cv2.imshow('Object detector', frame)
        
        if cv2.waitKey(1) == ord('q'):
            break
    
    cv2.destroyAllWindows()
    videostream.stop()

if __name__ == "__main__":
    # Initialize the detection queue
    detection_queue = Queue()

    # Start the object detection thread
    detection_thread = Thread(target=objectDetection, args=(detection_queue,))
    detection_thread.start()
    
    # Blink the LED to indicate start
    blink()
    
    running = True
    try: 
        while running:
            # Check for 'q' key press to exit
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        running = False

            # Measure the distance against the wall to prevent hit
            # if the robot is too close to the wall, turn right                                                                
    except KeyboardInterrupt:
        print("KeyboardInterrupt by user")

    # End
    blink()
    GPIO.cleanup()
    detection_thread.join()
    pygame.quit()