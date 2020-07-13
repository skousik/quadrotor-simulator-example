%% description
% This script generates a spline trajectory (i.e. a time-varying polynomial
% in each spatial dimension), then has the quadrotor track that spline. The
% spline in this case is a 2 s long trajectory towards a desired position.
%
% You can play with this script by editing the values in the "user
% parameters" cell. To understand what is going on better, explore what is
% happening in the subsequent cells (i.e. "automated from here")
%
% Author: Shreyas Kousik
% Created: 13 Jul 2020
%
%% user parameters
% quadrotor initial conditions
v_0 = zeros(3,1) ; % velocity in \R^3
a_0 = zeros(3,1) ; % acceleration in \R^3

% trajectory timing info (comment out one of the trajectory generation
% lines in the cell below)
time_horizon = 3 ; % seconds
dt = 0.01 ; % discretization fineness
t_extra = 1 ; % amount of time to add at end of trajectory

% info specific to desired position trajectory
p_des = [1;1;0] ; % (x,y,z) desired location

% info specific to peak speed trajectory
v_peak = [1;1;0.5] ; % m/s
t_peak = 1.5 ; % time at which peak speed is achieved

% NOTE: t_extra adds some extra time to the end of the trajectory, which
% lets us see what happens when we command a hover after the drone is
% already moving

%% automated from here
% make low-level controller
LLC = Mellinger_LLC() ;

% set low-level controller gains (these are the default values)
LLC.K_p = 2 .* eye(3) ;
LLC.K_v = 0.5 .* eye(3) ;
LLC.K_R = 1 .* eye(3) ;
LLC.K_w = 0.01*eye(3) ;

% make quadrotor agent
A = hummingbird_agent('LLC',LLC) ;

% set initial state
z_0 = [zeros(3,1) ; v_0 ; zeros(3,1)] ; % (position, velocity, angular velocity)
R_0 = eye(3) ; % initial attitude (rotation matrix)
A.reset(z_0,R_0) % reset takes in an initial 9-by-1 state and a 3-by-1 attitude

%% create trajectory to track (comment out one of the following)
% make desired position trajectory
[T,U,Z] = generate_spline_desired_position(p_des,v_0,a_0,time_horizon,dt,t_extra) ;

% make peak speed trajectory
% [T,U,Z] = generate_spline_peak_velocity(v_0,a_0,v_peak,t_peak,time_horizon,dt,t_extra) ;

%% make robot track desired trajectory
% A.move takes in an amount of time to track a
% trajectory, plus the reference trajectory specified as (T,U,Z) which is
% the time array T, feedforward input U, and desired trajectory Z
A.move(time_horizon + t_extra,T,U,Z) ;

%% plot
figure(1) ; clf ; hold on ; axis equal ; view(3) ; grid on ;
plot(A)
animate(A)