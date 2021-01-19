#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 27 08:49:45 2020

@author: Angel Ayala <angel4ayala [at] gmail.com>
"""

import numpy as np
from simple_pid import PID


def pi_clip(angle):
    """Ensure the angle will be in a [-pi, pi] loop.

    :param float angle: The angle in radians to clip.
    """
    if angle > 0:
        if angle > np.pi:
            return angle - 2 * np.pi
    else:
        if angle < -np.pi:
            return angle + 2 * np.pi
    return angle


class Drone:
    """The Drone class manage each sensor and actuators of the drone.
     
    It is developed for the Mavic 2 Pro drone, it consists of GPS, IMU, Gyro, 
    Compass, Camera, LED and Motor nodes.
    This drone control unit is designed to stabilize the drone through 4 PID 
    controllers tunned for a 8ms simulation timestep, and the drone's gimbal 
    with a Damping node in the WorldInfo node with values of 0.5 for both 
    angular and linear fields.
    
    :param integer timestep: The simulation timestep, 8ms mus be setted, 
        unexpected behaviour can occur with a different value.
    :param string name: A name for the controller, just for debug purpose.
    :param float start_alt: The initial altitude to be reached.
    """

    def __init__(self, name='Mavic', start_alt=1., start_yaw=np.pi):
        # Time helpers
        self.time_counter = 0

        # Variables
        self.target_altitude = start_alt  # drone's initial position
        self.target_yaw = start_yaw  # drone's initial orientation
        self.roll_correction = np.pi / 2.  # due to drone's initial rotation
        self.lift_thrust = 68.5  # with this thrust, the drone lifts.

    def init_sensors(self, drone, timestep):
        """Initialize each sensor distance of the Mavic 2 Pro.

        :param drone Robot: The instantiated Robot Node class.
        :param integer timestep: The simulation timestep, 8ms mus be setted,
            unexpected behaviour can occur with a different value.
        """
        self.sensors_id = ['front left', 'front right',
                           'rear top', 'rear bottom',
                           'left side', 'right side',
                           'down front', 'down back']
        # instantiate sensors
        self.sensors = [drone.getDistanceSensor("{} dist sonar".format(sid))
                        for sid in self.sensors_id]
        self.sensors_id.append('top dist')
        self.sensors.append(drone.getDistanceSensor("top dist infrared"))
        # activate sensors
        for sensor in self.sensors:
            sensor.enable(timestep)

        return True

    def init_devices(self, drone, timestep):
        """Initialize each device of the Mavic 2 Pro, in a desired timestep.

        In this project the Compass node is not used.
        The camera node is initialized at 33ms timestep to reach ~30fps.

        :param drone Robot: The instantiated Robot Node class.
        :param integer timestep: The simulation timestep, 8ms mus be setted,
            unexpected behaviour can occur with a different value.
        """
        # time
        self.deltaT = timestep / 1000.
        # Drone's Odometry
        # Position coordinates [Y, Z ,X]
        self.gps = drone.getGPS("gps")
        self.gps.enable(timestep)
        # Angles respect global coordinates [roll, pitch, yaw]
        self.imu = drone.getInertialUnit("inertial unit")
        self.imu.enable(timestep)
        # Accelertion angles [roll, pitch, yaw]
        self.gyro = drone.getGyro("gyro")
        self.gyro.enable(timestep)
        # Direction degree with north as reference
        self.compass = drone.getCompass("compass")
        self.compass.enable(timestep)

        # Video acquisition
        fps = 25
        self.camera = drone.getCamera("camera")
        self.camera_rate = 1000 // fps
        self.camera.enable(self.camera_rate)

        self.leds = [
            drone.getLED("front left led"),
            drone.getLED("front right led")
        ]
        # gimbal
        self.camera_roll = drone.getMotor("camera roll")
        self.camera_pitch = drone.getMotor("camera pitch")

        # Motors
        sides = [
            ['front', 'rear'],
            ['left', 'right']
        ]
        self.motors = [drone.getMotor("{} {} propeller".format(part, side))
                       for part in sides[0] for side in sides[1]]

        return True

    def init_motors(self):
        """Initialize the Motor nodes and the PID controllers."""
        # self.maxVelocity = 576# -> 5 m/s
        # self.maxTorque = 30

        # motor init
        for m in self.motors:
            m.setPosition(float('inf'))
            m.setVelocity(1.)

        # Propeller PID control params tunned with Ziegler–Nichols PID
        K_u = 150.
        T_u = 342.857 / 1000.  # ms
        # no overshoot
        params_roll = {'P': K_u / 5., 'I': (2. / 5.) * K_u / T_u,
                       'D': K_u * T_u / 15., 'sp': 0.}
        self.rollPID = PID(params_roll['P'], params_roll['I'],
                           params_roll['D'], setpoint=params_roll['sp'],
                           output_limits=(-2., 2.), sample_time=self.deltaT)

        K_u = 150.
        T_u = 682.66 / 1000.  # ms
        # no overshoot
        params_pitch = {'P': K_u/5.,
                        'I': (2. / 5.) * K_u / T_u,
                        'D': K_u*T_u/15.,
                        'sp': 0.}
        self.pitchPID = PID(params_pitch['P'], params_pitch['I'],
                            params_pitch['D'], setpoint=params_pitch['sp'],
                            output_limits=(-2., 2.), sample_time=self.deltaT)
        K_u = 20.
        T_u = 1621.33 / 1000.  # ms
        # PD
        params_yaw = {'P': 0.8 * K_u,
                      'I': 0.,
                      'D': K_u * T_u / 10.,
                      'sp': self.target_yaw}
        self.yawPID = PID(params_yaw['P'], params_yaw['I'], params_yaw['D'],
                          setpoint=params_yaw['sp'], output_limits=(-2., 2.),
                          sample_time=self.deltaT, error_map=pi_clip)

        K_u = 20.
        T_u = 2668.8 / 1000.  # ms
        # PD
        params_vert = {'P': 0.8 * K_u,
                       'I': 0.,
                       'D': K_u * T_u / 10.,
                       'sp': self.target_altitude}
        self.vertPID = PID(params_vert['P'], params_vert['I'],
                           params_vert['D'], setpoint=params_vert['sp'],
                           output_limits=(-5., 5.), sample_time=self.deltaT)

        return True

    def blink_leds(self):
        """Blink the LED nodes."""
        led_state = int(self.time_counter) % 2
        self.leds[0].set(led_state)
        self.leds[1].set(int(not(led_state)))

    def gimbal_stabilize(self, acceleration):
        """Stabilize camera (gimbal)."""
        self.camera_roll.setPosition(-0.115 * acceleration[0])
        self.camera_pitch.setPosition(-0.1 * acceleration[1])

    def get_odometry(self):
        """Get the drone's current acceleration, angles and position."""
        acceleration = self.gyro.getValues()
        angles = self.imu.getRollPitchYaw()
        position = self.gps.getValues()

        compass = self.compass.getValues()
        north_deg = np.arctan2(compass[0], compass[1])
        north_deg = (north_deg - 1.5708) / np.pi * 180

        if north_deg < 0.:
            north_deg += 360.

        return acceleration, angles, position, north_deg

    def get_image(self):
        """Get the Camera node image with size and channels.

        :return the data buffer with BGRA values
        """
        return self.camera.getImage()

    def get_sensors_info(self):
        """Get the Distance sensors Nodes' info."""
        return [0 if np.isnan(s.getValue()) else int(s.getValue())
                for s in self.sensors]

    def get_camera_metadata(self):
        """Get the camera image dimension and channels."""
        return self.camera.getHeight(), self.camera.getWidth(), 4  # channels

    def control(self, phi=0., theta=0., psi=0., thrust=0.):
        """Control the drone's motor for a given angles and thrust.

        In order to reach the desired angles and altitude, the drone must vary
        the velocity of each motor. In order to achieve this the arguments
        passed are used as setpoint for each PID controller, for the case of
        the altitude and yaw angle, a target value is used and is update by the
        amount of the argument value. If no value is passed the drone will hold
        its posision.

        :param float phi: The phi angle for the roll setpoint.
        :param float theta: The theta angle for the pitch setpoint.
        :param float psi: The psi variation value for the target angle of yaw.
        :param float thrust: The thrust variation value for the target of the
            altitude.
        """
        # compute current state
        acceleration, angles, position, _ = self.get_odometry()
        roll_angle = angles[0] + self.roll_correction
        pitch_angle = angles[1]
        yaw_angle = angles[2]
        altitude_position = position[1]
        # update target values
        self.target_yaw += psi
        self.target_yaw = pi_clip(self.target_yaw)
        self.target_altitude += thrust

        # Compute ouput values
        # Roll phi angle
        self.rollPID.setpoint = phi
        roll = (self.rollPID(roll_angle, dt=self.deltaT) * -1
                + acceleration[0])

        # Pitch theta angle
        self.pitchPID.setpoint = theta * -1  # positive angle to front
        pitch = (self.pitchPID(pitch_angle, dt=self.deltaT) * -1
                 - acceleration[1])

        # Yaw psi angle
        self.yawPID.setpoint = self.target_yaw
        yaw = (self.yawPID(yaw_angle, dt=self.deltaT) * -1
               + acceleration[2])

        # Vertical thrust
        self.vertPID.setpoint = self.target_altitude
        altitude = self.vertPID(altitude_position, dt=self.deltaT)

        # update time
        self.time_counter += self.deltaT
        # leds
        self.blink_leds()
        # camera
        self.gimbal_stabilize(acceleration)

        # Actuate the motors taking into consideration all the computed inputs.
        fl_motor = self.lift_thrust + altitude - roll - pitch + yaw  # front L
        fr_motor = self.lift_thrust + altitude + roll - pitch - yaw  # front R
        rl_motor = self.lift_thrust + altitude - roll + pitch - yaw  # rear L
        rr_motor = self.lift_thrust + altitude + roll + pitch + yaw  # rear R

        # CounterClockWise motor propellers
        fr_motor *= -1  # CCW
        rl_motor *= -1  # CCW

        # actuate over the motors
        if not np.isnan(fl_motor):
            self.motors[0].setVelocity(fl_motor)
            self.motors[1].setVelocity(fr_motor)
            self.motors[2].setVelocity(rl_motor)
            self.motors[3].setVelocity(rr_motor)
