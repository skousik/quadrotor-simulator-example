function [T,U,Z] = generate_spline_desired_position(p_f,v_0,a_0,t_total,sample_time,t_extra)
% [T,U,Z] = generate_spline_desired_position(p_f,v_0,a_0,t_total,...
%                                            sample_time,t_extra)
%
% Create a spline, with associated nominal control inputs, for a quadcopter
% to track as in https://flyingmachinearena.org/wp-content/uploads/mueTRO15.pdf
%
% This function outputs a spline that reaches a desired final location,
% starting from the location (0,0,0), within the duration t_total
%
% The outputs are a 1-by-N time array T, a 4-by-N feedforward input U, and
% a 15-by-N nominal trajectory Z
%
% Inputs:
%   p_f - desired final location (relative to (x,y,z) = (0,0,0))
%   v_0 - initial speed (vx,vy,vz)_0
%   a_0 - initial acceleration
%   t_total - default is 1.5 s
%   sample_time - default is 0.05 s
%   t_extra - default is 0 (time added as hover at end of the trajectory)
%
% Outputs:
%   T - time vector of output
%   U - nominal thrust and body rates (in inertial frame)
%   Z - desired center-of-mass traj (position, speed, accel, jerk in x,y,z)
%
% Author: Shreyas Kousik

    % parse inputs
    if nargin < 6
        t_extra = 0 ;
        if nargin < 5
            sample_time = 0.05 ;
            if nargin < 4
                t_total = 1.5 ;
            end
        end
    end
    
    % create a time vector
    T = 0:sample_time:t_total ;
    
    % create default initial position and gravity
    p_0 = [0;0;0] ;
    g = [0;0;-9.81] ;

    % desired future speed and acceleration (QC stops at end of desired traj)
    vf = [0;0;0] ;
    af = [0;0;0] ;

    % for each axis, compute the change in position/velocity/accel
    Dp = p_f - p_0 - v_0.*t_total - 0.5.*a_0.*t_total^2 ;
    Dv = vf - v_0 - a_0.*t_total ;
    Da = af - a_0 ;

    [ax,bx,cx] = single_axis_params(Dp(1),Dv(1),Da(1),t_total) ;
    [ay,by,cy] = single_axis_params(Dp(2),Dv(2),Da(2),t_total) ;
    [az,bz,cz] = single_axis_params(Dp(3),Dv(3),Da(3),t_total) ;

    % compute the trajectory in each dimension
    a = [ax ay az] ;
    b = [bx by bz] ;
    c = [cx cy cz] ;

    Z = make_spline(T,p_0,v_0,a_0,a,b,c) ;
    
%% compute the feedforward (nominal) control input
    U = generate_nominal_input(T,Z) ;
    
%% add extra time if necessary
    if t_extra > 0
        T = [T, T(end) + t_extra] ;
        U = [U, U(:,end)] ;
        Z = [Z, Z(:,end)] ;
    end
end

function [a,b,c] = single_axis_params(Dp,Dv,Da,T)
    M = [720, -360*T, 60*T^2 ;
         -360*T, 168*T^2, -24*T^3 ;
         60*T^2, -24*T^3, 3*T^4] ;
         
    out = (1/T^5)*M*[Dp;Dv;Da] ;
    a = out(1) ; b = out(2) ; c = out(3) ;
end