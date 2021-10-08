## Overview

Matlab code for the project: soft RCM (remote center manipulator) teleoperation control. The project is featured as:

1. Master Arm: MTM (Master Tool Manipulator) of dVRK (da Vinci Research Kit).
2. Slave Arm: Flexiv 7 DOF arm + customized instrument servo mechanism
3. Soft RCM control (in other words that we achieve the RCM fixed feature on general 7 DOF arm like PSM of dVRK by software instead of mechanism featured)
4. Both simulation and physical servo are available

## Prerequite

  You need to run the code on Ubuntu with ROS installation. Ubuntu 18.04 and ROS melodic are recommended.

## Install

1. Install dVRK

    You can refer to [dvrk](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild) to install dvrk.


2. Setup Flexiv External Control

    Here we give some detailed steps as recommendation:

    - Open power of the controller box
    - release e-stop button
    - push the controller handle to external box ( there is a swtich on the left upper direction of handle).
    - connect a LAN wire to your computer


    You can test the connection by the following code. Dowload [Flexiv_2_5](https://drive.google.com/file/d/1l8-08NTaTyWZuTghECyAlocHDnokIoPj/view?usp=sharing). Extract it. And in the terminal run 

    ```sh
    cd Flexiv_V2.5/example
    ./buildExample.sh
    ./ExampleClient
    ```
    The connection is ok if the code works fine.


3. Flexiv ROS Communication 

    Download [TeleOp_WS]()


4. Download RCM Control code

    ```sh
    git clone https://github.com/linhongbin-ws/soft-rcm-control.git
    ```

## Run Offline Simulation

We also offer offline simulation to quickly examine the control algorithm. To run offline simulation, you can open a Matlab, change directory to `<your path>/soft-rcm-control/control`, and run the script: `run_MTM_teleop_Flexiv_rcm_test.m`

## Run Online Demo


1. Launch roscore in the terminal

```sh
roscore
```

2. open a Matlab #1, change directory to `<your path>/soft-rcm-control/teleop`, and run in Matlab terminal.

```matlab
init_dvrk
run_Flexiv_cmd
```

  It will send a ROS command of initial pose of Flexiv arm. And you can change the initial pose in the script.
  


3. run flexiv ROS communication

```sh
source flexiv_teleop/devel/setup.bash
rosrun flexiv_teleop main
```

  And the Flexiv arm will move to initial pose.

4. run master arm

Run simulated MTM (If you do not have physical MTM)

```sh
source <dvrk-ws>/devel/setup.bash
roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=MTML config:=/home/bmt_group/code/dvrk_2_1/src/cisst-saw/sawIntuitiveResearchKit/share/cuhk-daVinci-2-0/console-MTML-simulated.json
```

Run physical MTM:
```sh
qlacloserelays
roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=MTML config:=/home/bmt_group/code/dvrk_2_1/src/cisst-saw/sawIntuitiveResearchKit/share/cuhk-daVinci-2-0/console-MTML-simulated.json
```

After popping out the console, click `Home` button to move MTM to home position.

5. Run Slave Arm in Simulation (Optional, if you do not have physical slave). In terminal

Flexiv simulation:
```sh
cd <path-to>/soft-rcm-control/simulator
python flexiv_simulator.py
```
Wrist Simulation
```sh
cd <path-to>/soft-rcm-control/simulator
python wrist_simulator.py
```

5. Open another Matlab #2 with another terminal, change directory to `<your path>/soft-rcm-control/teleop`, run controller in the matlab terminal

```matlab
init_dvrk
c = teleopRCM('MTML') # create controller
c.move_master_alignment() # will move MTM for alignment to reduce rotational tracking error 
c.start() # will start controller
```

6. Demos:

- Circular Trajectory Test

In Matlab #1, you run the script `run_MTM_traj_test.m` in directory `<your path>/soft-rcm-control/teleop`. Master and slave will run in the circular trajectory.

- Teleoperation by MTM

Use ROS or Matlab to send zero torque to MTM, so that it can be moved freely with gravity compensation. And then teleoperation works fine.





