# Quadrotor Simulation Example

This example code sets up a Hummingbird quadrotor in the [simulator](https://github.com/skousik/simulator) framework. It creates an example trajectory, then lets the quadrotor track that trajectory.

## Get started

1. download the simulator repo: https://github.com/skousik/simulator
2. make sure the simulator repo and this quadrotor repo are on your path in MATLAB
3. run the `run_example` script

## References
1. This code is used to simulate the drone in our paper: https://arxiv.org/abs/1904.05728
2. Trajectories are parameterized using the method in this paper: https://ieeexplore.ieee.org/document/7299672
3. The Hummingbird quadrotor in this work has a low-level controller (i.e., tracking controller): https://ieeexplore.ieee.org/document/5980409
