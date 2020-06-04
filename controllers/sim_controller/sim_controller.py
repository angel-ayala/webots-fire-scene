#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 28 19:19:44 2020

@author: Angel Ayala <angel4ayala [at] gmail.com>
"""

import cv2
import struct
import traceback
import numpy as np
from controller import Supervisor


def image_from_bytes(buffer, size=(240,400)):
    """Translate a buffered image Camera() node into an RGB channels array.
    
    :param bytes buffer: the buffer data of the image
    :param tuple size: (height, width) of the image
    """
    array_image = np.frombuffer(buffer, np.uint8).reshape(
        (size[0], size[1], 4)) #BGRA image
    array_image = array_image[:, :, :3] #BGR
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
        self.action = self.getEmitter('ActionEmitter') # channel 6
        self.state = self.getReceiver('StateReceiver') # channel 4
        self.state.enable(self.timestep)
        # actions value boundaries
        self.limits = self.set_limits()
        # runtime vars
        self.seed()
        self.sync()
        self._end = False
        # Fire vars
        self.fire = {
            'height': float('nan'),
            'radius': float('nan'),
            'pos': float('nan'),
            'set_height': self.fire_node.getField('fireHeight').setSFFloat,
            'set_radius': self.fire_node.getField('fireRadius').setSFFloat,
            'set_pos': self.fire_node.getField('translation').setSFVec3f
            }
        # Drone vars
        self.drone = {
            'pos_field': self.drone_node.getField('translation')#.getSFVec3f
            }
    
    @property
    def speed(self):
        """Return the simulation speed mode."""
        return self.simulationGetMode()  
    
    @property
    def is_running(self):
        """Get if the simulation is running."""
        return self.SIMULATION_MODE_PAUSE != self.speed
    
    def sync(self):
        """Synchronize device info with the drone."""
        # run simulation and get the data
        if not self.is_running:
            self.run_simulation()
        self._step() # step to process the signals propagation
        
        # check sync data exists and size
        if (self.state.getQueueLength() > 0 
            and self.state.getDataSize() == 12):
            # read and sync data
            msg = self.state.getData()
            self.cam_info = struct.unpack('3i', msg)
            # release the data and stop simulation
            self.state.nextPacket()
        self.stop_simulation() 
        
    def seed(self, seed=None):
        """Set seed for the numpy.random and WorldInfo node, None default."""
        self.np_random = np.random.RandomState(seed)
        world_node = self.getFromDef('World')
        world_node.getField('randomSeed').setSFInt32(
            0 if seed is None else seed)
        return seed
    
    def set_limits(self):
        """Get the limits to manipulate the angles and altitude."""
        limits = np.array([np.pi / 12.,     # roll
                  np.pi / 12.,              # pitch
                  np.pi / 360.,             # yaw
                  0.5                       # altitude
                  ])
        return np.array([limits*-1, # low limist
                         limits])   # high limits
    
    def set_speed(self, speed):
        """Set the Webots simulation speed mode."""
        speeds = [
            self.SIMULATION_MODE_PAUSE,
            self.SIMULATION_MODE_REAL_TIME,
            self.SIMULATION_MODE_RUN,
            self.SIMULATION_MODE_FAST
        ]
        
        if speed in speeds:
            self.simulationSetMode(speed)
        else:
            interval = "[{}, {}]".format(speeds[0], speeds[1])
            raise ValueError((
                "The speed must be a value " + interval + " as mentioned in "
                "the Webots documentation https://www.cyberbotics.com/doc/"
                "reference/supervisor?tab-language=python"
                "#wb_supervisor_simulation_set_mode"))
    
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
        self.set_speed(speed)
        return speed

    def stop_simulation(self):
        """Stop the Webots simulation.
        
        Set the simulation mode with the constant SIMULATION_MODE_PAUSE as 
        defined in the Webots documentation
        """
        mode = self.SIMULATION_MODE_PAUSE
        self.set_speed(mode)
        return mode
    
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
        self.simulationResetPhysics()
        self._step() # step to process the restart
        
    def _step(self):
        """Do a Robot.step(timestep)."""
        self.step(self.timestep)
        
    def randomize_fire_position(self):
        """Randomize the size and position of the FireSmoke node.
        
        The size and position has value in meters.
        The height of the node is [1., 5.] and it radius is [0.2, 1.5]
        The position is directly related to the radius, reducing the 2-axis 
        available space and requiring move up given its height.
        The limits are:
            X = [fire_radius -9.5, 9.5 -fire_radius]
            Z = [fire_radius -10, -1 -fire_radius] #Z because the land-grid in
            webots is in the XZ-axis and the Y offset is:
            Y = fire_height * 0.5
        """
        # randomize size
        fire_height = self.np_random.uniform(1., 5.)
        fire_radius = self.np_random.uniform(0.2, 1.5)
        # randomize position
        X = self.np_random.uniform(fire_radius -9.5, 9.5 -fire_radius)
        Z = self.np_random.uniform(fire_radius -10, -1 -fire_radius)
        Y = fire_height * 0.5
        fire_pos = [X, Y, Z]
        
        # FireSmoke node fields
        self.fire['set_height'](fire_height)
        self.fire['set_radius'](fire_radius)
        self.fire['set_pos'](fire_pos)
        self.fire['height'] = fire_height
        self.fire['radius'] = fire_radius
        self.fire['pos'] = fire_pos

        return fire_pos, fire_height, fire_radius

    def drone_distance(self, fixed=False):
        """Compute the drone's distance from the fire.
        
        :param bool fixed: Specify if must consider the altitude in distance.
        """
        fire_position = np.array(self.fire['pos'])
        drone_position = np.array(self.drone['pos_field'].getSFVec3f())
        
        # fixed altitude
        if fixed:
            drone_position[1] = fire_position[1] = 0
        else:
            fire_position[1] = 0.5
        
        # Euclidean distance
        delta = drone_position - fire_position
        distance = np.sum(np.square(delta))
        return distance
    
    def compute_reward(self, fixed=True):
        """Compute the reward related to the distance and fire size.
        
        This consider a risk_zone to 4 times the fire height as mentioned in
        Firefighter Safety Zones: A Theoretical Model Based on Radiative Heating,
        Butler, 1998.
        
        :param bool fixed: Indicate if must consider a fixed height in distance.
        """
        distance = self.drone_distance(fixed)
        risk_zone = self.fire['radius'] + self.fire['height'] *4
        reward = risk_zone - distance
        if reward > 0:
            self._end = True
        
        return reward

    def get_state(self):
        """Read the data sended by the drone's Emitter node.
        
        Capture and translate the drones sended data with the Receiver node. 
        This data is interpreted as the drone's state
        """
        image = None
        buffer_size = np.prod(self.cam_info)
        # get drone's state
        if (self.state.getQueueLength() > 0
            and self.state.getDataSize() == buffer_size):
            data = self.state.getData()
            fmt = "{}s".format(buffer_size)
            img_buffer = struct.unpack(fmt, data)
            image = image_from_bytes(img_buffer[0], 
                                     (self.cam_info[0], self.cam_info[1]))
            self.state.nextPacket()
            
        return image

    def take_action(self, action):
        """Perform an action step to the drone motors.
        
        Send the desired value of the angles an altitude to the drone to do a 
        variation in the motors speed the 3 angles and the altitude.
        
        :param list action: A 4 float values list with the velocity for the 
            [roll, pitch, yaw, altitude] variations.
        """
        if len(action) != 4:
            raise ValueError("The action is a list with 4 values with roll, "
                             "pitch, yaw angles and the altitude.")
        else:
            roll_angle, pitch_angle, yaw_angle, altitude = action
            # send only if a value is setted
            if (roll_angle != 0. 
                or pitch_angle != 0. 
                or yaw_angle != 0. 
                or altitude != 0.):
                msg = struct.pack('4d', roll_angle, pitch_angle, 
                                  yaw_angle, altitude)
                self.action.send(msg)

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
        
        # keyboard interaction
        print_control_keys()
        kb = self.getKeyboard()
        kb.enable(self.timestep)

        self.run_simulation()
        print('Running...')
        while (not self._end):# and drone.getTime() < 30):
            # capture control data
            key = kb.getKey()

            roll_angle = 0.
            pitch_angle = 0.
            yaw_angle = 0. #drone.yaw_orientation
            altitude = 0. #drone.target_altitude
            # capture state
            image = self.get_state()
            if image is not None and show:
                cv2.imshow("Drone's live view", image)
                cv2.waitKey(1)

            while key > 0:
                # roll
                if key == kb.LEFT:
                    roll_angle = self.limits[0][0]
                elif key == kb.RIGHT:
                    roll_angle = self.limits[1][0]
                # pitch
                elif key == kb.UP:
                    pitch_angle = self.limits[0][1]
                elif key == kb.DOWN:
                    pitch_angle = self.limits[1][1]
                # yaw
                elif key == ord('D'):
                    yaw_angle = self.limits[0][2]
                elif key == ord('A'):
                    yaw_angle = self.limits[1][2]
                # altitude
                elif key ==  ord('S'):
                    altitude = self.limits[0][3] * 0.1
                elif key == ord('W'):
                    altitude = self.limits[1][3] * 0.1
                # quit
                elif key ==  ord('Q'):
                    print('Terminating...')
                    self._end = True
                key = kb.getKey()

            action = [
                roll_angle,
                pitch_angle,
                yaw_angle,
                altitude
            ]
            self.take_action(action)
            self._step()
            
        if show:
            cv2.destroyAllWindows()
            
        self.restart_environment()
        self.stop_simulation()

if __name__ == '__main__':
    # run controller
    try:
        controller = SimController()
        controller.run(True)
    except Exception as e:
        traceback.print_tb(e.__traceback__)
        print(e)
        controller.stop_simulation()
        controller.restart_environment()