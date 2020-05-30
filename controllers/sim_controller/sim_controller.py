#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 28 19:19:44 2020

@author: Angel Ayala <angel4ayala [at] gmail.com>
"""

import cv2
import random
import numpy as np
import struct
import traceback
from controller import Supervisor


def image_from_bytes(buffer, size=(240,400)):
    """Translate a buffered image Camera() node into an RGB channels array.
    
    :param bytes buffer: the buffer data of the image
    :param tuple size: (height, width) of the image
    """
    array_image = np.frombuffer(buffer, np.uint8).reshape(
        (size[0], size[1], 4)) #RGBA image
    array_image = array_image[:, :, :3] #RGB
    return array_image

def print_control_keys():
    """Display manual control message."""
    print("You can control the drone with your computer keyboard:")
    print("IMPORTANT! The Webots 3D window must be selected to work!")
    print("- 'up': move forward.")
    print("- 'down': move backward.")
    print("- 'right': strafe right.")
    print("- 'left': strafe left.")
    print("- 'w': increase the target altitude.")
    print("- 's': decrease the target altitude.")
    print("- 'd': turn right.")
    print("- 'a': turn left.")
    print("- 'q': exit.")


# Webots environment controller
class SimController(Supervisor):
    """Main class to control the Webots simulation scene.
    
    In order to work this class, a Robot node must be present in the Webots 
    scenario with the supervisor option turned on. For this case the Robot 
    node is considered as the RL-agent configured with the Emitter and 
    Receiver nodes in order to get and send the states and actions, 
    respectively, working as the Remote Control of the drone.
    Additionally, this class is responsible to randomize the fire size and 
    location.
    Also, consider the implementation of a default keyboard control as human 
    interface for testing purpose.
        
    """
    
    def __init__(self):
        super(SimController, self).__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.fire_node = self.getFromDef('FireSmoke')
        self.drone_node = self.getFromDef('Drone')
        # Initialize Communication Nodes
        self.action = self.getEmitter('ActionEmitter')
        self.state = self.getReceiver('StateReceiver')
        self.state.enable(self.timestep)
        # runtime vars
        self._end = False
    
    def run_simulation(self, speed=1):
        """Start the Webots simulation.
                
        Set a respective speed (mode), to run the simulation. This option can
        take a value as the ones defined in the Webots documentation.        
        :param integer speed: Mode to run the simulation:
            1 = The simulation is running as close as possible to the 
                real-time. (SIMULATION_MODE_REAL_TIME)
            2 = The simulation is running as fast as possible with the 
                graphical renderings. (SIMULATION_MODE_RUN)
            3 = The simulation is running as fast as possible without the 
                graphical renderings. (SIMULATION_MODE_FAST)
        """
        if speed == 2:
            mode = self.SIMULATION_MODE_RUN
        elif speed == 3:
            mode = self.SIMULATION_MODE_FAST
        else:
            mode = self.SIMULATION_MODE_REAL_TIME
        # initiate simulation time
        self.simulationSetMode(mode)

    def stop_simulation(self):
        """Stop the Webots simulation.
        
        Set the simulation mode with the constant SIMULATION_MODE_PAUSE as 
        defined in the Webots documentation
        """
        self.simulationSetMode(self.SIMULATION_MODE_PAUSE)
    
    def reload_simulation(self):
        """Restart all the controllers and reload the current Webots world."""
        self.worldReload()

    def restart_environment(self):
        """Restart the entire simulation.
        
        Reset the fire and drone nodes at the starting point, and restart 
        the controllers simulation.
        """
        self.fire_node.restartController()
        self.drone_node.restartController()
        self.simulationReset()

    def get_data(self):
        """Read the data sended by the drone's Emitter node.
        
        Capture and translate the drones sended data with the Receiver node. 
        This data is interpreted as the drone's state
        """
        image = None
        # get drone's state
        if self.state.getQueueLength() > 0:
            data = self.state.getData()
            # get image info data 3*(int=4)bytes => 12 bytes
            img_height, img_width, channels = struct.unpack_from('3i', data)
            fmt = "{}s".format(channels*img_width*img_height)
            # get the image buffer with the 12 bytes offset
            img_buffer = struct.unpack_from(fmt, data, offset=12)
            image = image_from_bytes(img_buffer[0], (img_height, img_width))
            self.state.nextPacket()
        return image

    def _step(self, action):
        """Perform an action step to the drone motors.
        
        Send the desired value of the angles an altitude to the drone to do a 
        variation in the motors speed the 3 angles and the altitude.
        
        :param list action: A 4 float values list with the velocity for the 
            [roll, pitch, yaw, altitude] variations.
        """
        if len(action) == 4:
            roll_angle, pitch_angle, yaw_angle, altitude = action
            if (roll_angle != 0. 
                or pitch_angle != 0. 
                or yaw_angle != 0. 
                or altitude != 0.):
                msg = struct.pack('4d', roll_angle, pitch_angle, 
                                  yaw_angle, altitude)
                self.action.send(msg)
        self.step(self.timestep)

    def randomize_fire_position(self):
        """Randomize the size and position of the FireSmoke node.
        
        The size and position has value in meters.
        The height of the node is [1., 5.] and its radius is [0.2, 1.5]
        The position is directly related to the radius, reducing the 2-axis 
        available space and requiring move up given its height.
        The limits are:
            X = [-9.5 +fire_radius, 9.5 -fire_radius]
            Z = [-10 +fire_radius, -1 -fire_radius] #Z becaus the land-grid in
            webots is in the XZ-axis and the Y offset is:
            Y = 0.5 * fire_height
        """
        # random values
        fire_height = random.uniform(1., 5.)
        fire_radius = random.uniform(0.2, 1.5)
        X = random.uniform(-9.5 +fire_radius, 9.5 -fire_radius)
        Y = 0.5 * fire_height
        Z = random.uniform(-10 +fire_radius, -1 -fire_radius)
        fire_pos = [X, Y, Z]
        # FireSmoke node field
        self.fire_node.getField('fireHeight').setSFFloat(fire_height)
        self.fire_node.getField('fireRadius').setSFFloat(fire_radius)    
        self.fire_node.getField('translation').setSFVec3f(fire_pos)

        return fire_pos, fire_height, fire_radius

    def run(self, show=False):
        """Run controller's main loop.
        
        Capture the keyboard and translate into fixed float values to variate 
        the 3 different angles and the altitude, optionally an image captured 
        from the drone's camera can be presented in a new window.
        The pitch and roll angles are variated in +-pi/12., 
        the yaw angle in +-pi/360. and the altitude in +-5cm.
        The control keys are:
            - ArrowUp:      +pitch
            - ArrowDown:    -pitch
            - ArrowLeft:    -roll
            - ArrowRight:   +roll
            - W:            +altitude
            - S:            -altitude
            - A:            +yaw
            - D:            +yaw
            - Q:            EXIT
            
        :param bool show: Set if show or not the image from the drone's camera.
        """
        # Random FireSmoke position
        self.randomize_fire_position()
        self.simulationResetPhysics()
        
        # keyboard interaction
        print_control_keys()
        kb = self.getKeyboard()
        kb.enable(self.timestep)

        self.run_simulation()

        while (not self._end):# and drone.getTime() < 30):
            # capture control data
            key = kb.getKey()

            roll_angle = 0.
            pitch_angle = 0.
            yaw_angle = 0. #drone.yaw_orientation
            altitude = 0. #drone.target_altitude
            # capture state
            image = self.get_data()
            if image is not None and show:
                cv2.imshow('test', image)
                cv2.waitKey(1)

            while key > 0:
                if key == kb.UP:
                    pitch_angle = -np.pi / 12.
                elif key == kb.DOWN:
                    pitch_angle = np.pi / 12.
                elif key == kb.RIGHT:
                    roll_angle = np.pi / 12.#-1.0
                elif key == kb.LEFT:
                    roll_angle = -np.pi / 12.#1.0

                elif key == ord('A'):
                    yaw_angle = np.pi / 360.
                elif key == ord('D'):
                    yaw_angle = -np.pi / 360.
                elif key == ord('W'):
                    altitude = 0.05
                elif key ==  ord('S'):
                    altitude = -0.05
                elif key ==  ord('Q'):
                    self._end = True
                key = kb.getKey()

            action = [
                roll_angle,
                pitch_angle,
                yaw_angle,
                altitude
            ]
            self._step(action)
            if self._end:
                break
        if show:
            cv2.destroyAllWindows()
        
        self.stop_simulation()
        self.restart_environment()

if __name__ == '__main__':
    # run controller
    try:
        controller = SimController()
        controller.run()
    except Exception as e:
        print(e)
        traceback.print_tb(e.__traceback__)
        controller.stop_simulation()
        controller.restart_environment()