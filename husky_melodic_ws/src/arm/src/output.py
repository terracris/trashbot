import Jetson.GPIO as GPIO

# set the GPIO mode to BCM
GPIO.setmode(GPIO.BOARD)

pin_1 = 37

state = GPIO.LOW

# set up GPIO pin 8, 10 as output
GPIO.setup(pin_1, GPIO.OUT)

GPIO.output(pin_1, state)

try:
    while True:
        print(pin_1, ": ", state)
except KeyboardInterrupt:
    GPIO.cleanup()
