# CBFandAPF
Comparison of Control Barrier Functions (CBF) and Artificial Potential Fields (APF) for controlling a drone (MATLAB).


This code implements the path planning method of
DOI: 10.1109/IROS51168.2021.9636670 by Singletary et al., using control
barrier functions (CBF).

Singletary, Andrew, Karl Klingebiel, Joseph Bourne, Andrew Browning,
Phil Tokumaru, and Aaron Ames. "Comparative analysis of control barrier
functions and artificial potential fields for obstacle avoidance." In
2021 IEEE/RSJ International Conference on Intelligent Robots and Systems
(IROS), pp. 8129-8136. IEEE, 2021.

Authors: Francesco Bernardini, Ryan Lewis, Aaron T. Becker
Last update: May 29, 2024

The main functions are:
1. ObstacleAvoidanceUsingAPF.m
and
2. ObstacleAvoidanceUsingCBF.m

which implement obstacle avoidance with artificial potential fields (APF) 
and control barrier functions (CBF), for an arbitrary number of parameters 
rho0 (for APF) and alpha (for CBF).

The functions call other two functions which compute the individual 
trajectories:
3. computeTrajectoryAPF.m
4. computeTrajectoryCBF.m


The auxiliary functions:
5. drawcircles.m
6. findAxisLimits.m
7. plotTrajectories.m 
take care of the graphical aspects and plot the results.
