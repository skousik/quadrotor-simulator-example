function [T,U,Z] = generate_spline_peak_velocity(v_0,a_0,v_peak,t_peak,t_total,sample_time,t_extra)
% [T,U,Z] = generate_spline_peak_velocity(v_0,a_0,v_peak,t_peak,t_total,...
%                                         åsample_time,t_extra)
%
% Create a spline, with associated nominal control inputs, for a quadcopter
% to track as in https://flyingmachinearena.org/wp-content/uploads/mueTRO15.pdf
%
% This function outputs a spline that reaches a desired "peak" velocity,
% then slows to a stop; it starts from the location (0,0,0) with initial
% velocity v_0, goes up to v_peak at time t_peak, then down to 0 at time
% t_total
%
% The outputs are a 1-by-N time array T, a 4-by-N feedforward input U, and
% a 15-by-N nominal trajectory Z
%
% Inputs:
%   p_f - desired final location (relative to (x,y,z) = (0,0,0))
%   v_0 - initial speed (vx,vy,vz)_0
%   a_0 - initial acceleration
%   time_horizon - default is 1.5 s
%   sample_time - default is 0.05 s
%   t_extra - default is 0 (time added as hover at end of the trajectory)
%
% Outputs:
%   T - time vector of output
%   U - nominal thrust and body rates (in inertial frame)
%   Z - desired center-of-mass traj (position, speed, accel, jerk in x,y,z)
%
% note that, given the current initial condition set, the time horizon
% should be chosen so that the robot can come to a stop with some max
% allowable acceleration magnitude from the peak speed
%
% Author: Shreyas Kousik

    % parse inputs
    if nargin < 7
        t_extra = 0 ;
        if nargin < 6
            sample_time = 0.05 ;
            if nargin < 5
                t_total = 1.5 ;
            end
        end
    end
    
%% ensure inputs are right
    v_0 = v_0(:) ;
    a_0 = a_0(:) ;
    v_peak = v_peak(:) ;
    
%% compute the first part of the spline, up to v_peak
    % time vector
    T_to_peak = 0:sample_time:t_peak ; % assume t_peak/dt is an integer
    
    % desired acceleration at peak speed
    a_peak = [0;0;0] ;
    
    % compute change in velocity/accel for each axis
    Dv = v_peak - v_0 - a_0*t_peak ;
    Da = a_peak - a_0 ;
    
    % compute spline parameters
    [ax,bx,cx] = single_axis_params(Dv(1),Da(1),t_peak) ;
    [ay,by,cy] = single_axis_params(Dv(2),Da(2),t_peak) ;
    [az,bz,cz] = single_axis_params(Dv(3),Da(3),t_peak) ;
    
    a = [ax ay az] ;
    b = [bx by bz] ;
    c = [cx cy cz] ;
    
    % compute spline
    p_0 = [0;0;0] ; % default initial position
    Z_to_peak = make_spline(T_to_peak,p_0,v_0,a_0,a,b,c) ;

%% compute second part of the spline, coming to a stop
    % create time vector for second half
    t_to_stop = t_total - t_peak ;
    T_to_stop = 0:sample_time:t_to_stop ;    

    % desired end speed and acceleration
    v_f = [0;0;0] ;
    a_f = [0;0;0] ;

    % for each axis, compute the change in velocity/accel
    Dv = v_f - v_peak - a_peak.*t_to_stop ;
    Da = a_f - a_peak ;

    [ax,bx,cx] = single_axis_params(Dv(1),Da(1),t_to_stop) ;
    [ay,by,cy] = single_axis_params(Dv(2),Da(2),t_to_stop) ;
    [az,bz,cz] = single_axis_params(Dv(3),Da(3),t_to_stop) ;

    a = [ax ay az] ;
    b = [bx by bz] ;
    c = [cx cy cz] ;
    
    % compute spline
    p_peak = Z_to_peak(1:3,end) ;
    Z_to_stop = make_spline(T_to_stop,p_peak,v_peak,a_peak,a,b,c) ;

    % connect splines and times for the to-peak and to-stop portions
    T = [T_to_peak(1:end-1), T_to_stop + t_peak] ;
    Z = [Z_to_peak(:,1:end-1), Z_to_stop] ;

%% compute the feedforward (nominal) control input
    U = generate_nominal_input(T,Z) ;
    
%% add extra time to end of spline if desired
    if t_extra > 0     
        T = [T, T(end) + sample_time, T(end) + t_extra] ;
        Z = [Z, [Z(1:3,end);zeros(12,1)], [Z(1:3,end);zeros(12,1)]] ;
        U = [U, repmat(U(:,end),1,2)] ;
    end
end

function [a,b,c] = single_axis_params(Dv,Da,T)
    M = [0 0 ;
         -12 6*T ;
         6*T -2*T^2] ;
         
    out = (1/T^3)*M*[Dv;Da] ;
    a = out(1) ; b = out(2) ; c = out(3) ;
end