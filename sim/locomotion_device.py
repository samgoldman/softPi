import struct
import socket
from time import sleep
from time import time
from math import sin, cos

class LocomotionDevice:

    def __init__(self, sim_inst):
        self.sim = sim_inst

        self.sim.register_listener("GPIO15_pwmDutycycle", self.motor_updated, self, None)
        self.sim.register_listener("GPIO17_pwmDutycycle", self.motor_updated, self, None)

        self.sim.register_listener("GPIO16_level", self.motor_updated, self, None)
        self.sim.register_listener("GPIO18_level", self.motor_updated, self, None)

        self.sim.register_updater("GPIO14_level", self.get_switch_position, self, None)

        self.sim.register_variable("position")
        self.sim.register_updater("position", self.get_position, self, None)

        self.MAX_ANGULAR_VELOCITY = 2.0     #rad/s
        self.WHEEL_RADIUS         = 0.12    #m
        self.WHEEL_BASE           = 0.05    #m
         
        self.right_motor_speed = 0          #rad/s
        self.left_motor_speed = 0           #rad/s
        self.x  = 25                        #m
        self.y  = 25                        #m
        self.velocity = 0                   #m/s
        self.angle = 0                      #rad
        self.rotational_velocity = 0        #rad/s

        self.last_time = time()             #s


    def motor_updated(self, i):
        self.right_motor_speed = self.MAX_ANGULAR_VELOCITY * (self.sim.get_variable("GPIO15_pwmDutycycle") / 255.0)
        self.left_motor_speed = self.MAX_ANGULAR_VELOCITY * (self.sim.get_variable("GPIO17_pwmDutycycle") / 255.0)

        if 1 == self.sim.get_variable("GPIO16_level"):
            self.right_motor_speed *= -1

        if 1 == self.sim.get_variable("GPIO18_level"):
            self.left_motor_speed *= -1
        self.velocity = (self.right_motor_speed + self.left_motor_speed) / 2.0
        self.rotational_velocity = (self.right_motor_speed - self.left_motor_speed) / self.WHEEL_BASE

    def get_switch_position(self, i):
        if self.x < 0 or self.y < 0 or self.x > 30 or self.y > 30:
            return 1
        return 0
    
    def get_position(self, i):
        self.update_position()
        return (self.x, self.y)

    def update_position(self):
        time_diff = time() - self.last_time
        self.last_time = time()
        
        print("({0:.2f}, {1:.2f}, {2:.2f}, {3:1d})\n".format(self.x, self.y, self.angle, self.sim.get_variable("GPIO14_level")), end="")
        
        self.angle += self.rotational_velocity * time_diff
        self.x +=  self.velocity * cos(self.angle) * time_diff
        self.y +=  self.velocity * sin(self.angle) * time_diff

        
