import threading
import struct
import socket
from time import sleep

NUMBER_GPIO_PINS = 32

class GpioDevice:

    def __init__(self, sim_inst):
        self.sim = sim_inst
        self.listeners = {}
        self.updaters = {}

        for i in range(0, NUMBER_GPIO_PINS):
            self.sim.register_variable("GPIO{}_level".format(i))
            self.sim.register_variable("GPIO{}_pwmDutycycle".format(i))
            self.listeners["GPIO{}_level".format(i)] = 0
            self.updaters["GPIO{}_level".format(i)] = 0            
            self.updaters["GPIO{}_pwmDutycycle".format(i)] = 0            
        
        threading.Thread(target=self.rec_server).start()
#        threading.Thread(target=self.snd_server).start()

    def rec_server(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('127.0.0.1', 42421))
        
        while True:
            raw_data = s.recv(NUMBER_GPIO_PINS * 4 * 3)
            struct_string = 'I'*(32*3)
            data = struct.unpack(struct_string, raw_data)
            data = list(data)
            self.modes = data[32:64]
            self.dutycycles = data[64:]
            self.levels = [0] * NUMBER_GPIO_PINS
            for i in range(0, NUMBER_GPIO_PINS):
                if 1 == self.modes[i]:
                    self.levels[i] = data[i]
            self.update_registrations()        
            sleep(.05)
            unpacked_data = self.levels + self.modes + self.dutycycles
            struct_string = '{}I'.format(32*3)
            raw_data = struct.pack(struct_string, *unpacked_data)
            s.send(raw_data)
            sleep(.05)
    
    def update_registrations(self):
        for i in range(0, NUMBER_GPIO_PINS):
            if self.modes[i] == 0:
                if self.updaters["GPIO{}_level".format(i)] != 0:
                    self.sim.delete_registration(self.updaters["GPIO{}_level".format(i)])
                if self.updaters["GPIO{}_pwmDutycycle".format(i)] != 0:
                    self.sim.delete_registration(self.updaters["GPIO{}_pwmDutycycle".format(i)])
                if self.listeners["GPIO{}_level".format(i)] == 0:
                    self.listeners["GPIO{}_level".format(i)] = self.sim.register_listener("GPIO{}_level".format(i), self.gpio_updated, self, i)
            if self.modes[i] == 1:
                if self.listeners["GPIO{}_level".format(i)] != 0:
                    self.sim.delete_registration(self.listeners["GPIO{}_level".format(i)])
                if self.updaters["GPIO{}_level".format(i)] == 0:
                    self.updaters["GPIO{}_level".format(i)] = self.sim.register_updater("GPIO{}_level".format(i), self.gpio_get_new_value, self, i)
                if self.updaters["GPIO{}_pwmDutycycle".format(i)] == 0:
                    self.updaters["GPIO{}_pwmDutycycle".format(i)] = self.sim.register_updater("GPIO{}_pwmDutycycle".format(i), self.gpio_get_new_pwm_value, self, i)


    def gpio_updated(self, i):
        if self.modes[i] == 0:
            self.levels[i] = self.sim.get_variable("GPIO{}_level".format(i))

    def gpio_get_new_value(self, i):
        if self.modes[i] == 1:
            return self.levels[i] 

    def gpio_get_new_pwm_value(self, i):
        if self.modes[i] == 1:
            return self.dutycycles[i] 
