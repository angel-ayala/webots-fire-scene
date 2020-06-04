# Webots Fire Scene
A simulated Webots scene with a first approach of fire simulation through the FireSmoke node.
In this scene the DJI Mavic 2 Pro drone is available to control, variating the roll, pitch and yaw angles, and the target altitude.
The scenes is intended to run automously from the Webots interface using a Robot node with the supervisor option set to TRUE and with the controller set to '\<extern\>'.

The SimController class is configured to run from a terminal, being capable to:

1. Randomize the size and location of the FireSmoke node.
2. Start the simulation.
3. Acquire, and optionally show the image from the drone's camera.
4. Control the drone from keyboard inputs.
5. Stop and restart the simulation.

## The FireSmoke node
This node is implemented as a Robot node, using two Display nodes for the fire and smoke image.
The FireMovement controller manage this nodes using two sprite cheet images to simulate the movement of the fire and the smoke in a low resolution.
This node still need consider more realistic aspect such as the safe distance or propragation. This last aspect is not intented to be covered in the short-time.

- **Update 2020/06/04:** The safe distance was implemented using a first approach as presented by Butler[1] which define the safe distance at 4 times the fire's height.
To achieve this, the fire location is used as base where is added the radius size and 4 times the fire's height, if the drone exceed this point the '_end' var of SimController class is set True.
Additionally, some code improvements has been made to run as [OpenAI Gym](https://github.com/openai/gym) environment called [Gym Webots Fire Scene](https://github.com/angel-ayala/gym-webots-fire).
With this environment a reward can be computed considering the distance of the drone to the boundary the save zone, it start with a under zero value and is increasing while the drone is getting close of the fire location.

**TODO**
- [X] Fire and Smoke movement.
- [X] Safe distance of heat.
- [ ] Smoke cloud.
- [ ] Heat propagation.

## The Drone's controller
The Mavic 2 Pro control is achieved by the DroneController class which have a two-way communication with the SimController using an Emitter and Receiver node at each side.
This setup is intended to simulated a Radio Control (by the SimController) of the drone which send the variation for the angles and thrust values to manipulate the drone's position.
At counter part, the drone send back the image captured by its camera.

### Motors control
All the sensors and actuators of the drone are managed by the FlightControl class, which instantiate the GPS, IMU, Gyro, Camera, and the LED nodes. 
Additionally, control the drone's gimbal and propeller motors.
The velocity of each propeller motor is calculated using four PID controlles, one for the roll, pitch and yaw angles, and another for the altitude.
The controller were tunned using the Ziegler-Nichols PID tunning technique.

**IMPORTANT!** the controllers PID values were tunned for a 8ms simulation timestep, with a configured defaultDamping WorldInfo's field using a Damping node with 0.5 value for the angular and linear field.

### Camera image
The drone's is equipped with a Camera Node (can be modified in the Webots scene) for an 400x240 pixels BGRA channel image over a 3-axis gimbal to smooth the image movement.
The image is processed to remove the Alpha channel an keep the BGR channels to be presented during the execution of the sim_controller.py file.

## Running the scene
In order to execute the SimController from a terminal, must ensure to have configured the WEBOTS_HOME and LD_LIBRARY_PATH environment variables, and add the Webots lib controller to the PYTHONPATH, in order to import the Webots python library.
More info can be found in the [Running Extern Robot Controller section](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers) in the Webots documentation.

### Steps to run the simulation

**IMPORTANT!** This explain to execute the code in a Linux environment with python 3.7
In first place must be ensure that the **Webots simulator is running and loaded the fire.wbt world**, the simulation must be paused and at the begining (second 0).

From the terminal where will be executed the SimController, that is placed in controllers/sim_controller folder, ensure of the OS environment variables are set.
Next is an example of this, the WEBOTS_HOME must vary.
Enviroment variables example:
```
export WEBOTS_HOME=/home/user/webots
export LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller
export PYTHONPATH=$WEBOTS_HOME/lib/controller/python37
```
In order to check:
```
echo $WEBOTS_HOME # must show /home/user/webots
echo $LD_LIBRARY_PATH # must show /home/user/webots/lib/controller
echo $PYTHONPATH # must show  /home/user/webots/lib/controller/python37
```

Finally must execute the sim_controller.py file
```
python sim_controller/sim_controller.py
```

#### References
- [1] Firefighter Safety Zones: A Theoretical Model Based on Radiative Heating, Butler, 1998.
