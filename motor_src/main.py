from time import sleep
import cv2
import RPi.GPIO as GPIO
from measureDistance import measure
from move import forward, turn_right

v = 346
TRIGGER_PIN = 16
ECHO_PIN = 19
GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIGGER_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

if __name__ == '__main__':
    try:
        while True:
            distance = measure()
            print(f"Distance: {distance} cm")
            
            if distance < 10:
                sleep(1)
                print("Approaching the wall")
                turn_right(200)
            else:
                forward(200)
            
            sleep(1)
            
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Program terminated by user")

