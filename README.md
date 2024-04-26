# Minimal environment for quick prototyping with Cassie
This repo provides a Cassie model and a few helpful utilities to jump-start pure-python 
prototpying of robotics algorithms using a full scale biped example and [pydrake](https://drake.mit.edu/pydrake/index.html).

## Setup
A virtual environment is recommended. Any OS supported by Drake v1.28.0 is supported using the supplied requirements.txt file. 
If you are on a Linux distro which is not supported by the requested Drake version, you will likely have good luck simply 
calling `pip install drake`

### Ubuntu 22.04 or macOS Ventura/Sonoma (arm only)
```commandline
    python3 -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt
```

### Other operating systems (Untested)
```commandline
    python3 -m venv venv
    source venv/bin/activate
    pip install drake
```


## Walking data
We include trajectories of Cassie walking from a few hardware experiments. 
The trajectories include the floating base state (estimated using the contact-aided invariant EKF `Hartley 2019`), 
joint positions and velocities, and measured motor torques. 

The data was collected using an updated version of the MPC walking controller from [Bipedal Walking on Constrained Footholds with MPC Footstep Control
](https://dair.seas.upenn.edu/assets/pdf/Acosta2023.pdf).

If you find the data helpful, please consider citing us:

```bibtex
@inproceedings{Acosta2023,
  title = {Bipedal Walking on Constrained Footholds with MPC Footstep Control},
  author = {Acosta, Brian and Posa, Michael},
  year = {2023},
  month = dec,
  booktitle = {IEEE-RAS International Conference on Humanoid Robotics},
  youtube = {aTI6s2a3JSg},
  arxiv = {2309.07993},
  url = {https://ieeexplore.ieee.org/abstract/document/10375170},
  doi = {10.1109/Humanoids57100.2023.10375170}
}
```

The trajectories are stored in compressed `.npz` files and can be loaded in as shown in the example script: `dynamics_examples.py`

The state indices are defined below, the description of the joint dofs are defined in the file `cassie_v2.urdf`.
The joint ordering is defined using the Drake depth-first convention when loading in the urdf.
There is not an encoder on either `ankle_spring_joint` on the physical robot Cassie. 

We estimate the joint position using inverse kinematics in our state estimator and include it in the trajectory data; 
however, we leave the velocity of those joints at 0.

Positions:

| Index | Name |
|---|---|
| 0 | base_qw |
| 1 | base_qx |
| 2 | base_qy |
| 3 | base_qz |
| 4 | base_x |
| 5 | base_y |
| 6 | base_z |
| 7 | hip_roll_left |
| 8 | hip_yaw_left |
| 9 | hip_pitch_left |
| 10 | knee_left |
| 11 | knee_joint_left |
| 12 | ankle_joint_left |
| 13 | ankle_spring_joint_left |
| 14 | toe_left |
| 15 | hip_roll_right |
| 16 | hip_yaw_right |
| 17 | hip_pitch_right |
| 18 | knee_right |
| 19 | knee_joint_right |
| 20 | ankle_joint_right |
| 21 | ankle_spring_joint_right |
| 22 | toe_right |

Velocities:

| Index | Name |
|---|---|
| 0 | base_wx |
| 1 | base_wy |
| 2 | base_wz |
| 3 | base_vx |
| 4 | base_vy |
| 5 | base_vz |
| 6 | hip_roll_leftdot |
| 7 | hip_yaw_leftdot |
| 8 | hip_pitch_leftdot |
| 9 | knee_leftdot |
| 10 | knee_joint_leftdot |
| 11 | ankle_joint_leftdot |
| 12 | ankle_spring_joint_leftdot |
| 13 | toe_leftdot |
| 14 | hip_roll_rightdot |
| 15 | hip_yaw_rightdot |
| 16 | hip_pitch_rightdot |
| 17 | knee_rightdot |
| 18 | knee_joint_rightdot |
| 19 | ankle_joint_rightdot |
| 20 | ankle_spring_joint_rightdot |
| 21 | toe_rightdot |

Inputs:

| Index | Name |
|---|---|
| 0 | hip_roll_left_motor |
| 1 | hip_roll_right_motor |
| 2 | hip_yaw_left_motor |
| 3 | hip_yaw_right_motor |
| 4 | hip_pitch_left_motor |
| 5 | hip_pitch_right_motor |
| 6 | knee_left_motor |
| 7 | knee_right_motor |
| 8 | toe_left_motor |
| 9 | toe_right_motor |