# Import necessary libraries
import Jetson.GPIO as GPIO
import smbus
import time

# Define PCA9685 constants
PCA9685_I2C_ADDR = 0x40
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

# Initialize the I2C bus and PCA9685
bus = smbus.SMBus(1)  # Use bus 1 for Jetson TX2
bus.write_byte_data(PCA9685_I2C_ADDR, MODE1, 0x00)  # Reset PCA9685

# Function to set PWM duty cycle
def set_pwm(channel, on, off):
    channel_reg = LED0_ON_L + 4 * channel
    bus.write_i2c_block_data(PCA9685_I2C_ADDR, channel_reg, [on & 0xFF, on >> 8, off & 0xFF, off >> 8])

# Set PWM frequency
def set_pwm_freq(freq):
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    prescale = int(prescaleval + 0.5)
    old_mode = bus.read_byte_data(PCA9685_I2C_ADDR, MODE1)
    new_mode = (old_mode & 0x7F) | 0x10
    bus.write_byte_data(PCA9685_I2C_ADDR, MODE1, new_mode)
    bus.write_byte_data(PCA9685_I2C_ADDR, PRESCALE, prescale)
    bus.write_byte_data(PCA9685_I2C_ADDR, MODE1, old_mode)
    time.sleep(0.005)
    bus.write_byte_data(PCA9685_I2C_ADDR, MODE1, old_mode | 0x80)


# Cleanup GPIO

if __name__ == '__main__':
    # Set PWM frequency to 60Hz
    set_pwm_freq(60)

    # Set PWM duty cycle for channel zero to 100%

    while True:
        set_pwm(0, 0, 0xFFF)


    GPIO.cleanup()

