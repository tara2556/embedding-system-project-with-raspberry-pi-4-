from HR8825 import HR8825
import RPi.GPIO as GPIO
from time import sleep

FORWARD = 1
BACKWARD = 0

LEFT_DIR_PIN = 33
LEFT_STEP_PIN = 35
RIGHT_DIR_PIN = 18
RIGHT_STEP_PIN = 12

#Motor1 = HR8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
#Motor2 = HR8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))
Motor1 = HR8825(dir_pin=33, step_pin=35, enable_pin=32, mode_pins=(36, 11, 38))
Motor2 = HR8825(dir_pin=18, step_pin=12, enable_pin=7, mode_pins=(40, 15, 13))

Motor1.SetMicroStep('softward', 'fullstep')
Motor2.SetMicroStep('softward', 'fullstep')

def move_steps(steps, left_direction, right_direction, delay):
    """
    Control two stepper motors to rotate a specified number of steps.
    
    :param steps: Number of steps
    :param left_direction: Direction of the left wheel (FORWARD or BACKWARD)
    :param right_direction: Direction of the right wheel (FORWARD or BACKWARD)
    :param delay: Delay between each step (controls speed)
    """
    
    if left_direction == FORWARD:
        Motor1.digital_write(Motor1.enable_pin, 1)
        Motor1.digital_write(Motor1.dir_pin, 1)
    elif left_direction == BACKWARD:
        Motor1.digital_write(Motor1.enable_pin, 1)
        Motor1.digital_write(Motor1.dir_pin, 0)
    else:
        Motor1.digital_write(Motor1.enable_pin, 0)
        return
    
    if right_direction == FORWARD:
        Motor2.digital_write(Motor2.enable_pin, 1)
        Motor2.digital_write(Motor2.dir_pin, 0)
    elif right_direction == BACKWARD:
        Motor2.digital_write(Motor2.enable_pin, 1)
        Motor2.digital_write(Motor2.dir_pin, 1)
    else:
        Motor2.digital_write(Motor2.enable_pin, 0)
        return
    
    for _ in range(steps):
        GPIO.output(LEFT_STEP_PIN, GPIO.HIGH)
        GPIO.output(RIGHT_STEP_PIN, GPIO.HIGH)
        sleep(delay)
        GPIO.output(LEFT_STEP_PIN, GPIO.LOW)
        GPIO.output(RIGHT_STEP_PIN, GPIO.LOW)
        sleep(delay)

def forward(steps, delay=0.001):
    """
    Control both stepper motors to move forward a specified number of steps.
    
    :param steps: Number of steps
    :param delay: Delay between each step (controls speed)
    """
    move_steps(steps, FORWARD, FORWARD, delay)

def turn_left(steps, delay=0.001):
    """
    Control both stepper motors to turn left a specified number of steps.
    
    :param steps: Number of steps
    :param delay: Delay between each step (controls speed)
    """
    move_steps(steps, BACKWARD, FORWARD, delay)

def turn_right(steps, delay=0.001):
    """
    Control both stepper motors to turn right a specified number of steps.
    
    :param steps: Number of steps
    :param delay: Delay between each step (controls speed)
    """
    move_steps(steps, FORWARD, BACKWARD, delay)

if __name__ == '__main__':
    try:
        while True:
            # Example: move forward 200 steps
            forward(200)
            sleep(1)

            # Example: turn right 200 steps
            turn_right(200)
            sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
