#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 28 19:19:44 2020

@author: Angel Ayala <angel4ayala [at] gmail.com>
"""

import struct
import time
import traceback
import numpy as np
from controller import Supervisor


# utils functions
def bytes2image(buffer, shape=(240, 400, 4)):
    """Translate a buffered image Camera() node into an RGB channels array.

    :param bytes buffer: the buffer data of the image
    :param tuple size: (height, width, channels) of the image
    """
    array_image = np.frombuffer(buffer, np.uint8).reshape(shape)  # BGRA image
    array_image = array_image[:, :, :3]  # BGR
    return array_image


def min_max_norm(x, a=0, b=1, minx=0, maxx=1):
    """Normalize a x integer in the [a, b] range.

    :param integer x: Number to be normalized.
    :param integer a: Low limit. The default is 0.
    :param integer b: upper limit. The default is 1.

    :return float, the number normalized.
    """
    return a + (((x - minx) * (b - a)) / (maxx - minx))


def compute_distance(coord1, coord2):
    """Compute squared Euclidean distance.

    :param np.array coord1: the first coordinates.
    :param np.array coord2: the second coordinates.

    :return np.array: squared difference sum of the coordinates, rounded
        to 4 decimal points.
    """
    return np.sqrt(np.sum(np.square(coord1 - coord2))).round(4)


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

    def __init__(self, init_altitude=12.):
        super(SimController, self).__init__()
        # simulation timestep
        self.timestep = int(self.getBasicTimeStep())
        # actions value boundaries
        self.set_limits()
        # runtime vars
        self.lift_treshold = init_altitude
        self.drone_altitude = init_altitude
        # self.modes_available = [self.SIMULATION_MODE_PAUSE,
        #                         self.SIMULATION_MODE_REAL_TIME,
        #                         self.SIMULATION_MODE_RUN,
        #                         self.SIMULATION_MODE_FAST]
        self.seed()
        # self.sync()
        self.init_nodes()

    @property
    def is_running(self):
        """Get if the simulation is running."""
        return self.SIMULATION_MODE_PAUSE != self.simulationGetMode()

    def pause(self):
        """Pause the Webots's simulation."""
        self.simulationSetMode(self.SIMULATION_MODE_PAUSE)

    def play(self):
        """Start the Webots's simulation in real time mode."""
        self.simulationSetMode(self.SIMULATION_MODE_REAL_TIME)

    def play_fast(self):
        """Start the Webots's simulation in run mode."""
        self.simulationSetMode(self.SIMULATION_MODE_RUN)

    def play_faster(self):
        """Start the Webots's simulation in fast mode."""
        self.simulationSetMode(self.SIMULATION_MODE_FAST)

    # @property
    # def mode(self):
    #     """Get the current Webots simulation's speed mode."""
    #     return self.simulationGetMode()

    # @mode.setter
    # def mode(self, mode):
    #     """Set the Webots simulation's speed mode."""
    #     if mode in self.modes_available:
    #         self.simulationSetMode(mode)
    #     else:
    #         interval = "[{}, {}]".format(self.modes_available[0],
    #                                      self.modes_available[3])
    #         raise ValueError((
    #             "The speed must be a value " + interval + " as mentioned in "
    #             "the Webots documentation https://www.cyberbotics.com/doc/"
    #             "reference/supervisor?tab-language=python"
    #             "#wb_supervisor_simulation_set_mode"))

    @property
    def drone_lifted(self):
        """Get if the drone has reached the initial altitude."""
        curr_pos = self.drone_node.getPosition()
        off_pos = np.subtract(curr_pos,
                              self.drone['init_pos'])

        return off_pos.sum() > self.lift_treshold

    def seed(self, seed=None):
        """Set seed for the numpy.random and WorldInfo node, None default."""
        self.np_random = np.random.RandomState(seed)
        world_node = self.getFromDef('World')
        world_node.getField('randomSeed').setSFInt32(
            0 if seed is None else seed)
        return seed

    def set_limits(self):
        """Get the limits to manipulate the angles and altitude."""
        limits = np.array([np.pi / 12.,      # roll
                           np.pi / 12.,      # pitch
                           np.pi / 360.,     # yaw
                           0.5               # altitude
                           ])
        self.limits = np.array([limits * -1,  # low limist
                                limits])      # high limits
        return self

    def init_comms(self):
        """Initialize the communication nodes."""
        self.action = self.getEmitter('ActionEmitter')  # channel 6
        self.state = self.getReceiver('StateReceiver')  # channel 4
        self.state.enable(self.timestep)
        return self

    def init_nodes(self):
        """Initialize the fire and drone nodes' information."""
        # Flight Area
        area_size = self.getFromDef('FlightArea').getField('size').getSFVec2f()
        area_size = [fs / 2 for fs in area_size]  # size from center
        self.flight_area = [[fs * -1 for fs in area_size], area_size]

        # Forest area
        forest_shape = self.getFromDef('ForestArea').getField('shape')
        self.forest_area = []

        for i in range(forest_shape.getCount()):
            self.forest_area.append(forest_shape.getMFVec2f(i))

        # Fire vars
        self.fire_node = self.getFromDef('FireSmoke')
        self.fire = dict(
                height=self.fire_node.getField('fireHeight').getSFFloat(),
                radius=self.fire_node.getField('fireRadius').getSFFloat(),
                pos=np.array(self.fire_node.getField(
                    'translation').getSFVec3f()),
                set_height=self.fire_node.getField('fireHeight').setSFFloat,
                set_radius=self.fire_node.getField('fireRadius').setSFFloat,
                set_pos=self.fire_node.getField('translation').setSFVec3f
            )
        self.risk_distance = self.fire['radius'] + self.fire['height'] * 4
        # self.risk_distance = round(self.risk_distance**2, 4)

        # Drone vars
        self.drone_node = self.getFromDef('Drone')
        self.drone = dict(
                init_pos=self.drone_node.getField('translation').getSFVec3f()
            )
        return self

    def sync(self, altitude=False):
        """
        Synchronize device info with the drone.

        :param bool altitude: Indicate if must synchronize the initial
            altitude with the drone controller.
        """
        # Initialize Communication Nodes
        self.init_comms()
        if altitude:  # sync initial drone's alt.
            self.action.send(struct.pack('1f', self.lift_treshold))
            self.drone_altitude = self.lift_treshold
        # run simulation time
        self.play()
        # step to process the signals propagation
        self._step()

        # check sync data exists and size
        if (self.state.getQueueLength() > 0
                and self.state.getDataSize() == 20):
            # read and sync metadata
            msg = self.state.getData()
            drone_metadata = struct.unpack('5i', msg)
            self.state_shape = (drone_metadata[0],
                                drone_metadata[1],
                                drone_metadata[2] - 1)  # BGRA2BGR
            self.image_shape = drone_metadata[:3]
            self.len_image = np.prod(self.image_shape)
            self.len_sensors = drone_metadata[3]
            self.len_angles = drone_metadata[4]
            self.len_buffer = self.len_image + ((self.len_sensors
                                                 + self.len_angles) * 4) + 4
            # release the data
            self.state.nextPacket()

            # helper
            self.sensors_limits = [[50, 4000],  # front left
                                   [50, 4000],  # front right
                                   [50, 3200],  # rear top
                                   [50, 3200],  # rear bottom
                                   [50, 1000],  # left side
                                   [50, 1000],  # right side
                                   [50, 2200],  # down front
                                   [50, 2200],  # down back
                                   [10,  800]]  # top
            self.sensors = dict(front=[0, 1],
                                rear=[2, 3],
                                left=[4],
                                right=[5],
                                down=[6, 7],
                                top=[8])
        self.pause()
        return self

    def reset_simulation(self):
        """Reset the Webots simulation.

        Set the simulation mode with the constant SIMULATION_MODE_PAUSE as
        defined in the Webots documentation.
        Reset the fire and drone nodes at the starting point, and restart
        the controllers simulation.
        """
        if self.is_running:
            self.state.disable()  # prevent to receive data
            self.fire_node.restartController()
            self.drone_node.restartController()
            self.simulationReset()
            self.simulationResetPhysics()
            # stop simulation
            self._step()  # step to process the reset
            self.pause()

    def _step(self):
        """Do a Robot.step(timestep)."""
        self.step(self.timestep)

    def set_fire_dim(self, fire_height=2., fire_radius=0.5):
        """
        Set the FireSmoke Node's height and radius.

        :param float fire_height: The fire's height, default is 2.
        :param float fire_radius: The fire's radius, default is 0.5

        :return float, float: the settled height and radius values.
        """
        # FireSmoke node fields
        self.fire['set_height'](fire_height)
        self.fire['set_radius'](fire_radius)
        self.fire['height'] = fire_height
        self.fire['radius'] = fire_radius

        # update Y position and risk_zone
        self.set_fire_position(fire_pos=self.fire['pos'].copy())

        return fire_height, fire_radius

    def set_fire_position(self, fire_pos=None):
        """
        Set the FireSmoke Node's position in the scenario.

        Set a desired node position value or generated a new random one inside
        the scenario's forest area if no input is given.

        :param list pos: The [X, Z] position values where locate the node, if
            no values are given a random one is generated instead.
            Default is None.
        """
        fire_radius = self.fire['radius']
        if fire_pos is None:  # randomize position
            fire_pos = self.fire['pos'].copy()  # current position
            # get forest limits
            X_range = [self.forest_area[3][0], self.forest_area[1][0]]
            Z_range = [self.forest_area[1][1], self.forest_area[3][1]]

            # randomize position
            fire_pos[0] = self.np_random.uniform(fire_radius - abs(X_range[0]),
                                                 X_range[1] - fire_radius)
            fire_pos[2] = self.np_random.uniform(fire_radius - abs(Z_range[0]),
                                                 Z_range[1] - fire_radius)

        fire_pos[1] = self.fire['height'] * 0.5  # update height

        # FireSmoke node fields
        self.fire['set_pos'](list(fire_pos))
        self.fire['pos'] = np.array(fire_pos)
        self.risk_distance = fire_radius + self.fire['height'] * 4
        # self.risk_distance = round(self.risk_distance**2, 4)

        return fire_pos, self.risk_distance

    def randomize_fire_position(self):
        """Randomize the size and position of the FireSmoke node.

        The size and position has value in meters.
        The height of the node is [2., 13.] and it radius is [0.5, 3.]
        The position is directly related to the radius, reducing the 2-axis
        available space and requiring move up given its height.
        The position is delimited by the forest area.
        """
        # randomize dimension
        fire_height, fire_radius = self.set_fire_dim(
            fire_height=self.np_random.uniform(2., 13.),
            fire_radius=self.np_random.uniform(0.5, 3.))

        # avoid to the fire appears near the drone's initial position
        n_random = 0
        while (self.get_goal_distance() <= self.risk_distance
               or n_random == 0):
            # randomize position
            fire_pos, self.risk_distance = self.set_fire_position()
            n_random += 1

        return fire_pos, fire_height, fire_radius, self.risk_distance

    def get_drone_pos(self):
        """Read the current drone position from the node's info."""
        return np.array(self.drone_node.getPosition())

    def get_goal_distance(self):
        """Compute the drone's distance to the fire."""
        fire_position = self.fire['pos'].copy()
        drone_position = self.get_drone_pos()

        # consider only xz coordinates
        fire_position[1] = drone_position[1]

        # Squared Euclidean distance
        distance = compute_distance(drone_position, fire_position)
        return distance

    def check_altitude(self, altitude=[11, 75]):
        """Check if the Drone is outside the altitude limits defined.

        :param list altitude: optional, The altitude's inferior and superior
         limits. The default is [11, 75].

        :return bool: True if the Drone is outside the limits, otherwise False.
        """
        upper = self.drone_altitude >= altitude[1]
        bottom = self.drone_altitude <= altitude[0]
        return [upper, bottom]

    def check_flight_area(self):
        """Check if the Drone is outside the FlightArea."""
        drone_position = self.get_drone_pos()

        # X axis check
        north = drone_position[0] <= self.flight_area[0][0]
        south = drone_position[0] >= self.flight_area[1][0]

        # Z axis check
        east = drone_position[2] <= self.flight_area[0][1]
        west = drone_position[2] >= self.flight_area[1][1]
        return [north, south, east, west]

    def check_flipped(self, angles):
        """Check if the Drone was flipped.

        :param list angles: The Drone's 3D normalized angles.

        :return bool: True if have the propellers down side.
        """
        return angles[0] > 0.3

    def check_near_object(self, sensors, threshold=0.01):
        """Check if the sensors encountered an object near.

        :param list sensors: The Drone's normalized sensors values.

        :return bool: True if the value of any sensor is below the threshold.
        """
        object_near = [sensor < threshold for sensor in sensors]

        return object_near

    def get_state(self):
        """Read the data sended by the drone's Emitter node.

        Capture and translate the drones sended data with the Receiver node.
        This data is interpreted as the drone's state
        """
        image = np.zeros(self.state_shape)  # blank image
        sensors = np.zeros(self.len_sensors)  # blank sensor data
        angles = np.zeros(self.len_angles)  # blank angles data
        north_deg = 180.  # initial north orientation

        # get drone's state
        if (self.state.getQueueLength() > 0
                and self.state.getDataSize() == self.len_buffer):
            data = self.state.getData()
            fmt = "{}s{}i{}f1f".format(self.len_image,
                                       self.len_sensors,
                                       self.len_angles)
            data_buffer = struct.unpack(fmt, data)
            init_idx = 0
            image = bytes2image(data_buffer[init_idx], self.image_shape)

            init_idx += 1
            sensors = list(data_buffer[init_idx:init_idx + self.len_sensors])

            # normalize values
            for i in range(self.len_sensors):
                sensor_limit = self.sensors_limits[i]
                sensors[i] = min_max_norm(sensors[i], minx=sensor_limit[0],
                                          maxx=sensor_limit[1])

            init_idx += self.len_sensors
            angles = list(data_buffer[init_idx:init_idx + self.len_angles])

            # normalize values
            for i in range(self.len_angles):
                angles[i] = min_max_norm(angles[i], a=-1, b=1,
                                         minx=-np.pi,
                                         maxx=np.pi)

            init_idx += self.len_angles
            north_deg = float(data_buffer[init_idx])

            self.state.nextPacket()

        return image, sensors, angles, north_deg

    def take_action(self, action):
        """Perform an action step to the drone motors.

        Send the desired value of the angles an altitude to the drone to do a
        variation in the motors speed the 3 angles and the altitude.

        :param list action: A 4 float values list with the velocity for the
            [roll, pitch, yaw, altitude] variations.
        """
        if len(action) != 4:
            raise ValueError("The action is a list with 4 values with roll, "
                             "pitch, yaw angles and the altitude."
                             "{} was given".format(len(action)))
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
                self.drone_altitude += altitude
            self._step()  # step to process the action

    def __del__(self):
        """Stop simulation when is destroyed."""
        self.reset_simulation()


if __name__ == '__main__':
    import cv2

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

    def run(controller, show=True):
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
        # keyboard interaction
        print_control_keys()
        kb = controller.getKeyboard()
        kb.enable(controller.timestep)

        # Start simulation with random FireSmoke position
        controller.sync(True)
        controller.randomize_fire_position()
        controller.play()
        run_flag = True

        print('Fire scene is running')
        while (run_flag):  # and drone.getTime() < 30):
            # capture control data
            key = kb.getKey()

            roll_angle = 0.
            pitch_angle = 0.
            yaw_angle = 0.  # drone.yaw_orientation
            altitude = 0.  # drone.target_altitude
            # capture state
            image, sensors, angles, north_deg = controller.get_state()
            if show:
                cv2.imshow("Drone's live view", image)
                cv2.waitKey(1)

            while key > 0:
                # roll
                if key == kb.LEFT:
                    roll_angle = controller.limits[0][0]
                elif key == kb.RIGHT:
                    roll_angle = controller.limits[1][0]
                # pitch
                elif key == kb.UP:
                    pitch_angle = controller.limits[1][1]
                elif key == kb.DOWN:
                    pitch_angle = controller.limits[0][1]
                # yaw
                elif key == ord('D'):
                    yaw_angle = controller.limits[0][2]
                elif key == ord('A'):
                    yaw_angle = controller.limits[1][2]
                # altitude
                elif key == ord('S'):
                    altitude = controller.limits[0][3]  # * 0.1
                elif key == ord('W'):
                    altitude = controller.limits[1][3]  # * 0.1
                # quit
                elif key == ord('Q'):
                    print('Terminated')
                    run_flag = False
                key = kb.getKey()

            action = [
                roll_angle,
                pitch_angle,
                yaw_angle,
                altitude
            ]
            controller.take_action(action)
            print("DIST: {:.2f} [{:.2f}]".format(
                controller.get_goal_distance(), controller.risk_distance),
                "(INFO:",
                # "obj_det: {},".format(
                # controller.check_near_object(sensors)),
                # "out_alt: {},".format(
                # controller.check_altitude()),
                # "out_area: {},".format(
                # controller.check_flight_area()),
                # "is_flip: {},".format(
                # controller.check_flipped(angles)),
                "north: {:.2f})".format(
                    north_deg),
                np.array(controller.drone_node.getPosition())
                )

        if show:
            cv2.destroyAllWindows()

    # run controller
    try:
        controller = SimController(init_altitude=20.)
        run(controller)
    except Exception as e:
        traceback.print_tb(e.__traceback__)
        print(e)
        controller.reset_simulation()()
