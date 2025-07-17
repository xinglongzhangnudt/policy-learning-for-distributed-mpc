

# Distrbuted policy learning for DMPC Project


This repository contains material related to the project on `Distributed policy learning for DMPC`.  

## Table of Contents

### Tutorials

The tutorials lead you through implementing the code files uploaded. 

* DLPC_training: Implement the code to verify the convergence condition of actor-critic learning and the closed-stability condition under actor-critic learning in the receding horizon control framework. The code is implemented in Matlab.
* DLPC_xtdrone: Deploy the control policy to control a number of multirotor drones in the Gazebo. This part is based on XTDrone, PX4, and MAVROS, containing materials related to  [XTDrone project](https://github.com/robin-shaun/XTDrone/blob/master). The code is implemented in Python. 
  * DLPC_xtdrone6: Control 6 multirotor drones to realize formation control and transformation.
  * DLPC_xtdrone18: Control 18 multirotor drones to realize formation control and transformation.
* DLPC_solving_one_robot_control: Implement the code to solve the centralized control problem of one robot distributedly and compare it with the centralized version. The code is implemented in Matlab.
* dlpc_online_train_scales_to_10000.py: The Python code for online training of DLPC.
* dlpc_online_train_scales_to_10000.m: The matlab code for online training of DLPC.

## Dependencies

There is no dependency for `DLPC_training` within Matlab. As for `DLPC_xtdrone`, please follow the instructions in [XTDrone project](https://github.com/robin-shaun/XTDrone/blob/master) to complete the environment installation and basic configuration.

## Run DLPC_xtdrone

To run the code in this repository, follow the instructions below.

1. Load worlds and drones.
    ```bash
    roslaunch multi_vehicle.launch
      ```
   
3. Obtain the position information of drones. Replace 6 with the number in the name of the selected file folder.
    ```bash
    python3 get_local_pose.py iris 6
      ```
   
5. Build the communication network among drones.
    ```bash
    multi_vehicle_communication.sh
      ```
   
6. Keyboard control code.
    ```bash
    python3 multirotor_keyboard_control_promotion.py
      ```
    *Use the keyboard to control all drones to take off and press ‘s’ to hover after a desired height. Then press ‘g’ to enter leader control mode.
   
7. Run the DLPC code for formation control.
    ```bash
    run_formation_promotion.sh
      ```
   *Note: When the script is running, and the drones are stationary, switch to the keyboard control terminal to press ‘w’ to give the leader a specified velocity. After the drones achieve the specified formation, press ‘f’ or ‘h’ to turn or press numbers 0-9 to change the formation.

8. run the baseline controller for comparison  in a straight-line formation scenario.
    ```bash
    run_formation_baseline.sh
    ```
9. Run the following script for the figure plot.
    ```bash
    python3 draw_figure.py
    ```
## Reference
Please cite the following reference:

[1] Xinglong Zhang, et al. "Toward Scalable Multirobot Control: Fast Policy Learning in Distributed MPC." IEEE Transactions on Robotics, 41 (2025).
