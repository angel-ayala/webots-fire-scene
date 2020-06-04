#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 29 23:16:03 2020

@author: Angel Ayala <angel4ayala [at] gmail.com>
"""

import struct
from controller import Robot
from drone import Drone


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
        print('Initializing Drone...', end=' ')
        self.drone = Drone(self.timestep)
        self.drone.init_devices(self, self.timestep)
        self.drone.init_motors()

        # Initialize comms
        self.state = self.getEmitter('StateEmitter') # channel 4
        self.action = self.getReceiver('ActionReceiver') # channel 6
        self.action.enable(self.timestep)
        self.sync() # devices sync
        print('OK')
        
    def _step(self):
        """Do a Robot.step()."""
        return self.step(self.timestep)
        
    def sync(self):
        """Synchronize device info with remote control."""
        height, width, channels = self.drone.get_camera_metadata()
        data = struct.pack('3i', height, width, channels)
        self.state.send(data)
        self.cam_info = [height, width, channels]

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
        print('Running...')
        while self._step() != -1:
            # values change
            roll_disturb = 0.
            pitch_disturb = 0.
            yaw_disturb = 0. #drone.target_yaw
            target_disturb = 0. #drone.target_altitude

            # send state
            img_buffer = self.drone.get_image()
            buffer_size = 1
            for elm in self.cam_info:
                buffer_size *= elm
            fmt = "{}s".format(buffer_size)
            msg = struct.pack(fmt, img_buffer)
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
            self.drone.control(roll_disturb, pitch_disturb, yaw_disturb, 
                               target_disturb)

if __name__ == '__main__':
    # run controller
    controller = DroneController()
    controller.run()
