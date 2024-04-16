from stepper import Stepper
import time

""" this is the Gripper class. it inherits from the Stepper class. It does not make full use of the
    Stepper class, since it does not have a homing pin. All we care about is opening and closing the gripper"""

class Gripper(Stepper):

    def __init__(self, pulse_pin, dir_pin, enable_pin):
        homing_pin = None  # no homing pin (we should add one in the future)
        steps_per_rev = 100  # TODO review specs for this motor
        gear_ratio = 1
        max_speed = 100  # max speed in pulses per second
        max_joint_positive_angle = 90
        max_joint_negative_angle = 90
        home_count = 0
        # this device is not homeable. by default is starts open.
        homing_direction = None
        super().__init__(pulse_pin, dir_pin, enable_pin, homing_pin, steps_per_rev, gear_ratio,
                         max_speed, max_joint_positive_angle, max_joint_negative_angle, home_count, homing_direction, debug=True, has_homed=True)
        
       
        """
        I need to test these two methods to make sure they work properly and that the counts are correct.
        """
    def open(self):
        self.direction = Stepper.CW
        open_position = 0 # pulse position 0 is open
        self.move_absolute_pid(0)  # moves stepper to position 0

    def close(self):
        self.direction = Stepper.CCW
        close_position = 1000 # pulse position 100 is closed
        self.move_absolute_pid(close_position)  # moves stepper to position 100


if __name__ == '__main__':
    grippy_pulse_pin = 12 # check this is actually the correct pin
    grippy_dir_pin = 18
    grippy_enable_pin = None
    grippy = Gripper(grippy_pulse_pin, grippy_dir_pin, grippy_enable_pin)

    try:
        test_case = input("Enter 'open' or 'close': ")
        
        while test_case != "exit":
            if test_case == "open":
                grippy.open()
            elif test_case == "close":
                grippy.close()
            else:
                grippy.move_clockwise_simple()
            test_case = input("Enter 'open' or 'close': ")
    except KeyboardInterrupt:
    # Clean up GPIO on keyboard interrupt
        grippy.cleanup()
