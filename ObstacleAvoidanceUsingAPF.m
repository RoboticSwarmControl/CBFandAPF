function ObstacleAvoidanceUsingAPF(CustomParametersFlag)
% This script implements the path planning method of
% DOI: 10.1109/IROS51168.2021.9636670 by Singletary et al., using
% artificial potential fields (APF).
%
%  Singletary, Andrew, Karl Klingebiel, Joseph Bourne, Andrew Browning,
%  Phil Tokumaru, and Aaron Ames. "Comparative analysis of control barrier
%  functions and artificial potential fields for obstacle avoidance." In
%  2021 IEEE/RSJ International Conference on Intelligent Robots and Systems
%  (IROS), pp. 8129-8136. IEEE, 2021.
%
% Some flags can be used to select the input or to format the output
% 1. CustomParametersFlag: performs the simulation with the parameters of
% the paper (FALSE) or with custom parameters (TRUE).
% 2. animateFlag: plots the trajectories all at once (FALSE) or timestep by
% timestep (TRUE).
%
%  Run time: On a 2023 Macbook Pro M2 with 32 Gb, the default code requires
%  20 s.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Authors: Francesco Bernardini, Ryan Lewis, Aaron T. Becker
% Last update: May 29, 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

format compact;

if(nargin<1)
    CustomParametersFlag = false;
end

%%%%%%%%%Initial parameters setup%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%CustomParametersFlag = false; %selects the set of parameters to use

if(CustomParametersFlag)
    %Customize these parameters at will
    dT = 0.001; %Timestep
    S = [0,0,0]; %Starting point 
    G = [3,5,5]; %Goal point
    %Define the obstacles
    O = [];
    O.C(1,:) = [1,2,2.5]; 
    O.R(1) = 0.5;
    O.C(2,:) = [2.5,3,3.5];
    O.R(2) = 0.5;
    O.C(3,:) = [1.5,2.5,3.5];
    O.R(3) = 1.0;
    rho0 = [0.1, 0.25, 0.5, 1]; %An array of safety distances
    Katt = 1; %Attraction factor
    MaxTime = 10000; %Maximum simulation time
    Krep = 1; %Repulsion factor
    Ndim = 3; %Defines if obstacles and paths are 2D or 3D
    %%%%%%%%%End parameters setup%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
else
    %%%%%%%%%Initial parameters setup%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Use these parameters if you want to reproduce the results of the paper
    %%%%%%%%%%%%%%%%%%%%DO NOT ALTER THIS SECTION%%%%%%%%%%%%%%%%%%%%%%%%%%

    dT = 0.001; %Timestep
    S = [0,0,0]; %Starting point 
    G = [3,5,5]; %Goal point
    %Define the obstacles
    O = [];
    O.C(1,:) = [1,2,2.5]; 
    O.R(1) = 0.5;
    O.C(2,:) = [2.5,3,3.5];
    O.R(2) = 0.5;
    rho0 = [0.1, 0.25, 0.5, 1]; %An array of safety distances
    Katt = 1; %Attraction factor
    MaxTime = 10000; %Maximum simulation time
    
    Krep = 1; %Repulsion factor
    Ndim = 2; %Defines if obstacles and paths are 2D or 3D
    %%%%%%%%%End parameters setup%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Invoke APF function%%%%%%%%%%%%%%%%%%%%%%%%%%
%Allocate space for the trajectories
Trajectories = cell(numel(rho0),1);

%Call the path planning function
for k = 1:numel(rho0)
    Trajectories{k}= computeTrajectoryAPF(S(1:Ndim), G(1:Ndim), O, rho0(k), dT, Katt, Krep, MaxTime, Ndim);
end

CBFflag = false;
APFflag = true;
animateFlag = true; %false: plots the trajectories as a whole
                     %true: plots one timestep at a time
plotTrajectories(rho0, Ndim, S, G, O, Trajectories, animateFlag, CBFflag, APFflag)

end