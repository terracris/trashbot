import Jetson.GPIO as GPIO
import time

# Set the GPIO mode to BCM numbering
GPIO.setmode(GPIO.BOARD)

input_1 = 15
input_2 = 23
input_3 = 33
input_4 = 40

test_pin = input_4
# Set up GPIO pin 18 as an input
GPIO.setup(test_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    while True:
        # Read the state of the GPIO pin
        input_state = GPIO.input(test_pin)
        print("GPIO pin ", test_pin, " state: ", input_state)
        time.sleep(0.1)  # Wait for a short time to avoid flooding the console

except KeyboardInterrupt:
    # Clean up GPIO on keyboard interrupt
    GPIO.cleanup()

