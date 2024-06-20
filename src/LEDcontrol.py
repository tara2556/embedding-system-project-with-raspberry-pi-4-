import RPi.GPIO as GPIO
from time import sleep

def openLED(id: int, LED1_PIN=12, LED2_PIN=22) -> None:
    if id == 1:
        GPIO.output(LED1_PIN, GPIO.HIGH)
    elif id == 2:
        GPIO.output(LED2_PIN, GPIO.HIGH)

def closeLED(id: int, LED1_PIN=12, LED2_PIN=22) -> None:
    if id == 1:
        GPIO.output(LED1_PIN, GPIO.LOW)
    elif id == 2:
        GPIO.output(LED2_PIN, GPIO.LOW)

def blink() -> None:
    openLED(1)
    openLED(2)
    sleep(0.1414)
    closeLED(1)
    closeLED(2)
    sleep(0.1414)
    openLED(1)
    openLED(2)
    sleep(0.1414)
    closeLED(1)
    closeLED(2)

if __name__ == "__main__":
    try:
        while True:
            print("LED is on")
            openLED(1)
            sleep(1)
            openLED(2)
            sleep(1)
            
            print("LED is off")
            closeLED(1)
            sleep(1)
            closeLED(2)
            sleep(1)
            
    except KeyboardInterrupt:
        print("Exception: KeyboardInterrupt")
        
    finally:
        GPIO.output(LED1_PIN, GPIO.LOW)
        GPIO.output(LED2_PIN, GPIO.LOW)
        GPIO.cleanup()
