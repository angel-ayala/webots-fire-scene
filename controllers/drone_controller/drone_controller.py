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
        # local variables
        self.timestep = int(self.getBasicTimeStep())

        # Initialize Flight Control
        print('Initializing Drone Control...', end=' ')
        self.drone = Drone(start_alt=1.5)
        self.drone.init_devices(self, self.timestep)
        self.drone.init_sensors(self, self.timestep)
        self.drone.init_motors()

        # Initialize comms
        self.state = self.getEmitter('StateEmitter')  # channel 4
        self.action = self.getReceiver('ActionReceiver')  # channel 6
        self.action.enable(self.timestep)
        # self.sync()
        print('OK')

    def _step(self):
        """Do a Robot.step()."""
        return self.step(self.timestep)

    def sync(self):
        """Synchronize device info with remote control."""
        # retrieve data from drone
        self.len_sensors_data = len(self.drone.get_sensors_info())
        _, angles_data, _, _ = self.drone.get_odometry()
        self.len_angles_data = len(angles_data)

        # send metadata
        height, width, channels = self.drone.get_camera_metadata()
        self.len_image_data = height * width * channels
        data = struct.pack('5i', height, width, channels,
                           self.len_sensors_data, self.len_angles_data)
        self.state.send(data)
        self._step()  # propagate data

        # wait for initial height if given
        print('Syncing initial altitude')
        while self.action.getQueueLength() == 0:
            print('.', end='')
            self._step()

        if self.action.getQueueLength() > 0:
            target_altitude = struct.unpack('1f', self.action.getData())[0]
            self.drone.target_altitude = target_altitude
            print('target_altitude', target_altitude)
            self.action.nextPacket()

    def run(self):
        """Run controller's main loop.

        Send the variations of the altitude, the roll, pitch, and yaw angles
        to the drone. Send the current image captured by the drone's
        camera and get the actions from the Remote Control, once the action
        (variations of the angles an altitude) is received, the Drone
        calculates the velocity, and apply it to the 3 different angles and
        altitude.
        """
        # devices sync
        self.sync()
        # control loop
        print('Drone control is active')
        while self._step() != -1:
            # values change
            roll_disturb = 0.
            pitch_disturb = 0.
            yaw_disturb = 0.  # drone.target_yaw
            target_disturb = 0.  # drone.target_altitude

            # Receiver's action
            if self.action.getQueueLength() > 0:
                # print(self.action.getDataSize())
                msg = self.action.getData()
                actions = struct.unpack('4d', msg)
                roll_disturb = actions[0]
                pitch_disturb = actions[1]
                yaw_disturb = actions[2]
                target_disturb = actions[3]
                self.action.nextPacket()

            # actuate Drone's motors
            self.drone.control(roll_disturb, pitch_disturb,
                               yaw_disturb, target_disturb)

            # get current state
            img_buffer = self.drone.get_image()
            sensors_data = self.drone.get_sensors_info()
            _, angles_data, _, north_deg = self.drone.get_odometry()
            # send data
            fmt = "{}s{}i{}f1f".format(self.len_image_data,
                                       self.len_sensors_data,
                                       self.len_angles_data)
            msg = struct.pack(fmt, img_buffer, *sensors_data,
                              *angles_data, north_deg)
            self.state.send(msg)  # image buffer


if __name__ == '__main__':
    # run controller
    controller = DroneController()
    controller.run()
