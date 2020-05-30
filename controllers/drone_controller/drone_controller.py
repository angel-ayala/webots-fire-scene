#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 29 23:16:03 2020

@author: Angel Ayala <angel4ayala [at] gmail.com>
"""

import struct
from controller import Robot
from flight_control import FlightControl


# Drone Robot
class DroneController(Robot):
    """DroneController is the main class to manage the drone's action.
    
    This class manage the Emitter and Receiver nodes to send and get the states
    and actions of the drone to the Remote Control.
    This use the FlightControl to controlate the motors velocity of the drone.
    """
    
    def __init__(self):
        super(DroneController, self).__init__()
        self.timestep = int(self.getBasicTimeStep())

        # Initialize Flight Control
        print('Initiating Drone...', end='')
        self.fc = FlightControl(self.timestep)
        self.fc.init_devices(self.timestep)
        self.fc.init_motors()

        # Initialize comms
        self.state = self.getEmitter('StateEmitter') # channel 4
        self.action = self.getReceiver('ActionReceiver') # channel 6
        self.action.enable(self.timestep)
        print('OK')

    def run(self):
        """Run controller's main loop.
        
        Send the variations of the altitude and the roll, pitch, and yaw 
        angles to the drone. Send the current image captured by the drone's 
        camera and get the actions sended by the Remote Control, once the 
        actions (variations of the angles an altitude) is received, calculate 
        the velocity with the FlightControl, and apply the velocity for the 3 
        different angles and the altitude.
        """
        # control loop
        while self.step(self.timestep) != -1:
            # values change
            roll_disturb = 0.
            pitch_disturb = 0.
            yaw_disturb = 0. #drone.target_yaw
            target_disturb = 0. #drone.target_altitude

            # send state
            img_buffer, img_height, img_width, channels = self.fc.get_image()
            fmt = "3i{}s".format(channels*img_width*img_height)
            msg = struct.pack(fmt, img_height, img_width, channels, img_buffer)
            self.state.send(msg) # image buffer

            # Receiver's action
            if self.action.getQueueLength() > 0:
                msg = self.action.getData()
                actions = struct.unpack('4d', msg)
                roll_disturb = actions[0]
                pitch_disturb = actions[1]
                yaw_disturb = actions[2]
                target_disturb = actions[3]
                self.action.nextPacket()

            # actuate Drone's motors
            roll, pitch, yaw, thrust = self.fc.calculate_velocity(roll_disturb,
                                        pitch_disturb, yaw_disturb, target_disturb)
            self.fc.apply_velocity(roll, pitch, yaw, thrust)

if __name__ == '__main__':
    # run controller
    controller = DroneController()
    controller.run()
