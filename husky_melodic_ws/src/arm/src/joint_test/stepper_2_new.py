import time
import ctypes
import numpy as np
from math import sqrt
import Jetson.GPIO as GPIO
from collections import deque

"""
Default motor rotation direction is CCW. However, you can set the motor to be inverted
"""
GPIO.setmode(GPIO.BOARD)

class Stepper:
    CCW = 1
    CW = 0
    libc = ctypes.CDLL("libc.so.6") # Load the C library
    MILLISECONDS_IN_SECOND = 1000    # number of milliseconds in one second

    """
    All motors use CCW as positive direction for rotationi as default but you can change it.
    max_speed comes in as pulses per second.
    """
    def __init__(self, pulse_pin, dir_pin, enable_pin, homing_pin, steps_per_rev, gear_ratio, max_speed, max_joint_positive_angle, max_joint_negative_angle, home_count, homing_direction ,inverted=False, kp=0.005, kd=0.003, has_homed = False, debug=False,stepper_id=0):
        self.id = stepper_id
        self.pulse_pin = pulse_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self.homing_pin = homing_pin
        self.inverted = inverted # changes the positive direction for rotation
        self.homing_direction = homing_direction  # default homing direction is CCW
        self.positive_direction = Stepper.CW if inverted else Stepper.CCW # defualt positive direction is CCW
        self.negative_direction = Stepper.CCW if inverted else Stepper.CW # default negative direction is CW

        self.direction = self.positive_direction if has_homed else homing_direction # current direction motor is spinning
        self.current_pos = 0
        self.target_pos = 0
        self.current_angle = 0

        # The current motor speed in steps per second --> Positive means counter-clockwise
        self.speed = 0
        self.max_speed = max_speed / Stepper.MILLISECONDS_IN_SECOND # our code operates in pulses per millisecond for increased accuracy
        self.acceleration = 0.0 # default to zero until set
        self.min_pulse_width = 2.5 # the minimum pulse width in microseconds (based on stepper driver)
        self.min_enable_setup = 0.2 # the minimum enable time in seconds (200 ms)
        self.min_dir_width = 5     # the minimum pulse width in microseconds (based on stepper driver)
        self.kp = kp
        self.kd = kd

        # gear ratio
        self.gear_ratio = gear_ratio
        self.steps_per_rev = steps_per_rev
        self.step_angle = 360 / steps_per_rev
        self.max_joint_limit_positive = max_joint_positive_angle
        self.max_joint_limit_negative = max_joint_negative_angle
        self.set_output_pins() # set up pins --> direction, pulse, enable
        self.has_homed = has_homed
        self.home_count = home_count
        self.debug = debug
        # self.home()

   
    # write angles
    # sets the direction
    def write(self, angle):
        # check if the angle is within the limits for the joint
        # if it is, calculate the angles number of steps to get there
        
        if not self.in_limits(angle):
            print(self.id, " angle out of limit: ", angle)
            return
        
        
        # absolute step position for arm
        absolute_steps = self.calculate_steps(angle)

        if self.debug: 
            print("the desired joint angle is ", angle)
            # print("steps to travel to ", absolute_steps)
        
        if absolute_steps > 0:
            self.direction = self.positive_direction # again positive steps are CCW
        else:
            self.direction = self.negative_direction
        
        self.move_absolute_pid(absolute_steps)
        
        # calculates our angle based on our current position
        # this accounts for angle errors
        self.current_angle = self.update_angle()

        return self.current_angle

    
    def in_limits(self, angle):
        # returns True if angle is in limit.
        return angle >= self.max_joint_limit_negative and angle <= self.max_joint_limit_positive
    
    """
    converts pulse position to angle in degrees.
    """
    def update_angle(self):
        return (self.current_pos * self.step_angle) / self.gear_ratio

    """
    Converts desired angle to steps to control the stepper motor.
    angle: angle from -jointLimitCW to JointLimitCCW

    negative goal steps indicates move CW. positive goal steps are CCW
    """
    def calculate_steps(self, angle):
        
        # rounds the number of steps required and then turns it into an int (just in case)
        goal_steps = int(round(angle * self.gear_ratio / self.step_angle))

        return goal_steps


    def move_absolute_pid(self, absolute):
        # if we are not there set the target position
        if self.current_pos != absolute:
            self.target_pos = absolute
        else:
            return
        
        Kp = self.kp
        Kd = self.kd

        prev_error = 0  # Initialize prev_error outside the loop
        start_time = Stepper.get_time()
        max_acceleration = 0.20 # max pulses per ms per ms
        prev_v_t = 0
        TOLERANCE = 3

        while abs(self.get_distance_to_target()) >= TOLERANCE :
            error = self.get_distance_to_target()
            error_der = error - prev_error
            # print("error derivative: ", error_der)

            #if error < 0:
                # print("switching direction")
                # self.direction = Stepper.CCW if self.direction == Stepper.CW else Stepper.CW

            v_t = abs(Kp * error + Kd * error_der)  # constrain maximum velocity
            v_t =  min(self.max_speed, v_t)
            v_t = max(v_t, 0.1) 
            
            self.step_interval = 1/ v_t  # [ ms period between each pulse ]

            if Stepper.get_time() - start_time >= self.step_interval:
                if self.debug:
                    pass
                    print(self.current_pos, self.direction)
                
                self.step()
                start_time = Stepper.get_time()
                prev_error = error
                prev_v_t = v_t

        # print("at position")


    def get_distance_to_target(self):
        return self.target_pos - self.current_pos
    
    
    def step(self):
        self.set_direction_pins()
        GPIO.output(self.pulse_pin, GPIO.HIGH)
        Stepper.usleep(self.min_pulse_width)
        GPIO.output(self.pulse_pin, GPIO.LOW)
        Stepper.usleep(self.min_pulse_width)
        
        if self.has_homed:
            self.update_position()

    def update_position(self):
        if self.direction == self.positive_direction:
            self.current_pos += 1
        else:
            self.current_pos -= 1
    
    def set_direction_pins(self):
        if self.direction == Stepper.CCW:
            if self.debug:
                pass
                #print("ccw")
            GPIO.output(self.dir_pin, GPIO.HIGH) # When direction pin is HIGH, the motor will spin CCW
        else:
            if self.debug:
                pass
                #print("cw")
            GPIO.output(self.dir_pin, GPIO.LOW) # when direction pin is LOW, the motor will spin CW

        Stepper.usleep(2*self.min_dir_width)


   # returns time in microseconds since epoch 1970
    @staticmethod 
    def get_time():
        return int(time.time() * Stepper.MILLISECONDS_IN_SECOND)
   
    def set_output_pins(self):
        
        GPIO.setup(self.pulse_pin, GPIO.OUT)   # output pin 
        GPIO.setup(self.dir_pin, GPIO.OUT)     # output pin
        
        if self.homing_pin != None:
            GPIO.setup(self.homing_pin, GPIO.IN) # input pin 
        
        if self.enable_pin != None: 
            GPIO.setup(self.enable_pin, GPIO.OUT)  # output pin
            GPIO.output(self.enable_pin, GPIO.LOW) # turn motors on
        
        if self.homing_direction == Stepper.CCW:
            GPIO.output(self.dir_pin, GPIO.HIGH)    # HIGH is ccw --> default direction
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)    # HIGH is ccw --> default direction
        GPIO.output(self.pulse_pin, GPIO.LOW)  # no pulse
        time.sleep(self.min_enable_setup) # minimum enable time is 200 ms 
    
    
    def home(self):
        
        is_home = GPIO.input(self.homing_pin)  # reads from homing pin to determine if we are home yet

        self.direction = self.homing_direction # all motors home in the CCW direction
        
        while not is_home:
            # if we are not home, we must rotate in our negative direction
            # until the limit switch has been hit
            
            is_home = GPIO.input(self.homing_pin)
            if is_home:
                break

            if self.debug:
                pass
                #print("homing direction is: ", self.direction)
            self.step()
            time.sleep(0.01) # sleep 10 ms between pulses --> gives pretty good speed

        
        if self.debug:
            print("I homed")
        self.stop()

        # once we have hit the limit switch, we must go to our home configuration step count
        # unfortunately, I believe this will have to be calculated experimentally. 
        # to minimize the error, we should increase the pulse number
        self.has_homed = True
        time.sleep(0.05) # wait to slow down completely
        home_count = abs(self.home_count) # count to home position
        self.direction = Stepper.CW if self.homing_direction == Stepper.CCW else Stepper.CCW  # we need to move in the opposite to our homing direction
        
        cur_pos = 0
        while cur_pos < home_count:
            self.step()
            time.sleep(0.01)
            if self.debug:
                pass
                #print("current direction is: ", self.direction)
            cur_pos += 1
        
        
        #self.move_absolute_pid(home_count) # move there

        # after all homing is complete, we need to reset our position
        self.reset_position()

    def reset_position(self):
        self.current_pos = 0
        self.target_pos = 0
        self.current_angle = 0

    def move_clockwise(self, angle):
        steps = self.calculate_steps(angle)
        #print("number of steps: ",steps)
        self.direction = Stepper.CW
        #print("direction is: ",self.direction)
        self.set_direction_pins()
        # print("time for PID")
        time.sleep(0.5)
        self.move_absolute_pid(steps)
    
    def move_clockwise_simple(self):
        self.direction = Stepper.CCW
        while True:
            self.step()
            time.sleep(0.05)
    
    def stop(self):
        GPIO.output(self.pulse_pin, GPIO.LOW)
        Stepper.usleep(self.min_pulse_width)
    
    def get_angle(self):
        return self.current_angle
    
    def cleanup(self):
        GPIO.cleanup()

    
    @staticmethod
    def usleep(microseconds):
        """
        Sleep for the given number of microseconds.
        """ 
        Stepper.libc.usleep(int(microseconds))

if __name__ == '__main__':
    pulses_per_rev = 200
    enable_pin = 37
    # joint 2
    pulse_pin_j2 = 19
    dir_pin_j2 = 21
    homing_pin_j2 = 23
    gear_ratio_j2 = 5 * 5.18
    home_count_j2 = -145
    max_speed_j2 = 75
    # gonna need to update kinematics to account for the joint limits:
    # like if it says j2 goes to 30 degrees, need to find clockwise alternative for all joints
    max_ccw_j2 = 115
    max_cw_j2 = -10
    homing_direction_j2 = Stepper.CCW
    try:
        j2 = Stepper(pulse_pin_j2, dir_pin_j2, enable_pin, homing_pin_j2, pulses_per_rev, gear_ratio_j2, max_speed_j2, max_ccw_j2, max_cw_j2, home_count_j2,homing_direction_j2 ,inverted=True, debug=True)
        print("about to home")

        j2.home()
        j2.write(90)
        except KeyboardInterrupt:
            GPIO.cleanup()
