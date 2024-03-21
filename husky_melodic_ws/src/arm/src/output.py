import Jetson.GPIO as GPIO

# set the GPIO mode to BCM
GPIO.setmode(GPIO.BOARD)

pin_1 = 13
pin_2 = 21
state = GPIO.HIGH

# set up GPIO pin 8, 10 as output
GPIO.setup(pin_1, GPIO.OUT)
GPIO.setup(pin_2, GPIO.OUT)

GPIO.output(pin_1, state)
GPIO.output(pin_2, state)

try:
    while True:
        print(pin_1, ": ", state)
        print(pin_2, ": ", state)
except KeyboardInterrupt:
    GPIO.cleanup()
