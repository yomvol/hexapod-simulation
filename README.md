# Hexapod Simulator
A side-project exploring hexapod locomotion in simulation using ROS 2, Gazebo and a custom gait engine with inverse kinematics. The goal was to brush up on theory like Denavit-Hartenberg matrices and to deepen knowledge of legged robotics and inverse kinematics. Hardware and real assembly are out of the scope, so PhantomX AX Mark III was chosen due to its popularity in research community.

## Features
* Tripod gait engine with phase offsets
* Analytical inverse kinematics (IK) solver for a 3-DOF leg
* Integration with ros2_control and gz_ros2_control
* Effort interface with PID control (position control proved unusable for complex trajectories)
* Leg workspace (reach) visualization in RViz

## Future plans
I like this project for its expandability. There's so much space to grow the sandbox: SLAM, navigation, rough terrain adaptability and so on.
* Try out different gait patterns (ripple, wave)
* Tune PID parameters
* Implement directional turning
* Add gamepad support
* Try out a real IK library like KDL or Track-IK
* Experiment with central pattern generators (CPGs) or reinforced learning

## How to build
It's recommended to make use of Docker with [CUDA support](https://github.com/NVIDIA/nvidia-container-toolkit)
Clone the repo and build a runtime container:<br>
`docker build `

## How to run
Inside a container (or locally) source the workspace:<br>
`source install/setup.sh`  
`ros2 launch hexapod_bringup gz_sim_main.launch.py`
And then in a new terminal call the service to transit to *standing* state:<br>
`docker exec`

## Credits
I’d like to gratefully acknowledge the following researchers whose work inspired and informed key aspects of this project:
- [**Kinematic and Dynamic Modeling of the PhantomX AX Metal Hexapod Mark III Robot using Quaternions**](https://www.researchgate.net/publication/356934434_Kinematic_and_Dynamic_Modeling_of_the_PhantomX_AX_Metal_Hexapod_Mark_III_Robot_using_Quaternions)<br>
Arnold Jean Pierre Acosta Chavez, Josmell Henry Alva Alcantara
Though this paper's quaternion-based approach to dynamics is yet to be grasped, DH parameters and leg trajectory pattern generation equations proved useful for the project.

- [**Dynamic Modeling and Control of the Hexapod Robot Using MATLAB SimMechanics**](https://www.researchgate.net/publication/330405549_Dynamic_Modeling_and_Control_of_the_Hexapod_Robot_Using_Matlab_SimMechanics)<br>
Mohamed A. Kamel PhD., Abdelrahman Zaghloul M.Eng., Sameh I. Beaber M.Eng.
This paper actually describes geometrical approach to inverse kinematics in detail (unlike the previous one)
