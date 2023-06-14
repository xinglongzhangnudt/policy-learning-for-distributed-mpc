

# Distributed Safe Learning Control for DMPC


This repository contains material related to the project on `Distributed Safe Learning Control for DMPC'.  

## Table of Contents

### Tutorials

The tutorials lead you through implementing the code files uploaded. 

* DSLC_training: Implement the code to verify the convergence condition of actor-critic learning and the closed-stability condition under actor-critic learning in the receding horizon control framework. The code is implemented in Matlab.
* DSLC_xtdrone: Deploy the control policy to control a number of multirotor drones in the Gazebo. This part is based on XTDrone, PX4, and MAVROS, containing materials related to  [XTDrone project](https://github.com/robin-shaun/XTDrone/blob/master).
  * DSLC_xtdrone6: Control 6 multirotor drones to realize formation control and transformation.
  * DSLC_xtdrone18: Control 18 multirotor drones to realize formation control and transformation.

## Dependencies

There is no dependency for `DSLC_training' in Matlab. As for `DSLC_xtdrone', please follow the instructions in [XTDrone project](https://github.com/robin-shaun/XTDrone/blob/master) to complete the environment installation and basic configuration.

## Run for DSLC_xtdrone

To set up your python environment to run the code in this repository, follow the instructions below.
1. Create (and activate) a new environment with Python 3.6.

	- __Linux__ or __Mac__: 
	```bash
	conda create --name drlnd python=3.6
	source activate drlnd
	```
	- __Windows__: 
	```bash
	conda create --name drlnd python=3.6 
	activate drlnd
	```
	
2. If running in **Windows**, ensure you have the "Build Tools for Visual Studio 2019" installed from this [site](https://visualstudio.microsoft.com/downloads/).  This [article](https://towardsdatascience.com/how-to-install-openai-gym-in-a-windows-environment-338969e24d30) may also be very helpful.  This was confirmed to work in Windows 10 Home.  

3. Follow the instructions in [this repository](https://github.com/openai/gym) to perform a minimal install of OpenAI gym.  
	- Next, install the **classic control** environment group by following the instructions [here](https://github.com/openai/gym#classic-control).
	- Then, install the **box2d** environment group by following the instructions [here](https://github.com/openai/gym#box2d).
	
4. Clone the repository (if you haven't already!), and navigate to the `python/` folder.  Then, install several dependencies.  
    ```bash
    git clone https://github.com/udacity/deep-reinforcement-learning.git
    cd deep-reinforcement-learning/python
    pip install .
    ```

5. Create an [IPython kernel](http://ipython.readthedocs.io/en/stable/install/kernel_install.html) for the `drlnd` environment.    
    ```bash
    python -m ipykernel install --user --name drlnd --display-name "drlnd"
    ```

6. Before running code in a notebook, change the kernel to match the `drlnd` environment by using the drop-down `Kernel` menu. 


## Want to learn more?

<p align="center">Come learn with us in the <a href="https://www.udacity.com/course/deep-reinforcement-learning-nanodegree--nd893">Deep Reinforcement Learning Nanodegree</a> program at Udacity!</p>

<p align="center"><a href="https://www.udacity.com/course/deep-reinforcement-learning-nanodegree--nd893">
 <img width="503" height="133" src="https://user-images.githubusercontent.com/10624937/42135812-1829637e-7d16-11e8-9aa1-88056f23f51e.png"></a>
</p>
