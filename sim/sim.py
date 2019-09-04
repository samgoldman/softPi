import gpio_device, locomotion_device
from time import sleep

class Simulation:
    def __init__(self):
        # variable_name: value
        self.simulation_variables = {}

        # variable_name: (id, function, device instance, parameter)
        #    Function takes 1 variable: new value of variable
        self.listeners = {}

        # variable_name: (id, function, device_instance)
        #    Function returns 1 variable: value of of the variable
        self.updaters = {}

        self.next_id = 1

    def get_variable(self, variable_name):
        return self.simulation_variables[variable_name]


    def register_variable(self, variable_name):
        if variable_name not in self.simulation_variables:
            self.simulation_variables[variable_name] = 0
            self.listeners[variable_name] = []
            self.updaters[variable_name] = []

    def register_listener(self, variable_name, function, device_instance, parameter):
        if variable_name in self.simulation_variables:
            self.listeners[variable_name].append((self.next_id, function, device_instance, parameter))
            self.next_id += 1
            return self.next_id - 1
        return 0


    def register_updater(self, variable_name, function, device_instance, parameter):
        if variable_name in self.simulation_variables:
            self.updaters[variable_name].append((self.next_id, function, device_instance, parameter))
            self.next_id += 1
            return self.next_id - 1
        return 0


    def delete_registration(self, registration_id):
        for key in self.listeners.keys():
            for j in range(len(self.listeners[key])):
                function_id, _, _, _ = self.listeners[key][j]
                if function_id == registration_id:
                    del self.listeners[key][j]

        for key in self.updaters.keys():
            for j in range(len(self.updaters[key])):
                function_id, _, _, _ = self.updaters[key][j]
                if function_id == registration_id:
                    del self.updaters[key][j] 


def main():
    sim = Simulation()
    ###### Load Devices #######
    pi = gpio_device.GpioDevice(sim)
    bot = locomotion_device.LocomotionDevice(sim)
    #### End Load Devices #####
    
    while True:
        for var, val in sim.simulation_variables.items():
            updated = False
            for _, function, device_instance, parameter in sim.updaters[var]:
                new_val = function(parameter)
                if new_val != val:
                    sim.simulation_variables[var] = new_val
                    updated = True
                    break
            for _, function, device_instance, parameter in sim.listeners[var]:
                function(parameter)

if __name__ == '__main__':
    main()
