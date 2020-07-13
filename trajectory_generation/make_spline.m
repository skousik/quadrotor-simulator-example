function Z_out = make_spline(T_in,p_0,v_0,a_0,a,b,c)
% Z_out = make_spline(T_in,p_0,v_0,a_0,a,b,c)
%
% Given a time vector T_in (usually T_in = 0:dt:t_final), initial
% position/velocity/acceleration (p_0/v_0/a_0), and coefficient vectors
% a,b,c \in \R^3, output a spline Z_out.
%
% The output format is a 15-by-N array where N = length(T_in), of the form
%   Z_out = [position (3-by-N) ;
%            velocity (3-by-N) ;
%            acceleration (3-by-N) ;
%            jerk (3-by-N) ;
%            snap (3-by-N)]
%
% Adapted from https://flyingmachinearena.org/wp-content/uploads/mueTRO15.pdf
%
% Author: Shreyas Kousik

    % get spline params
    ax = a(1) ; ay = a(2) ; az = a(3) ;
    bx = b(1) ; by = b(2) ; bz = b(3) ;
    cx = c(1) ; cy = c(2) ; cz = c(3) ;

    % position:
    px = (ax/120).*T_in.^5 + (bx/24).*T_in.^4 + (cx/6).*T_in.^3 + (a_0(1)/2).*T_in.^2 + v_0(1).*T_in + p_0(1) ;
    py = (ay/120).*T_in.^5 + (by/24).*T_in.^4 + (cy/6).*T_in.^3 + (a_0(2)/2).*T_in.^2 + v_0(2).*T_in + p_0(2) ;
    pz = (az/120).*T_in.^5 + (bz/24).*T_in.^4 + (cz/6).*T_in.^3 + (a_0(3)/2).*T_in.^2 + v_0(3).*T_in + p_0(3) ;
    
    % speed:
    vx = (ax/24).*T_in.^4 + (bx/6).*T_in.^3 + (cx/2).*T_in.^2 + (a_0(1)).*T_in + v_0(1) ;
    vy = (ay/24).*T_in.^4 + (by/6).*T_in.^3 + (cy/2).*T_in.^2 + (a_0(2)).*T_in + v_0(2) ;
    vz = (az/24).*T_in.^4 + (bz/6).*T_in.^3 + (cz/2).*T_in.^2 + (a_0(3)).*T_in + v_0(3) ;
    
    % acceleration:
    ax = (ax/6).*T_in.^3 + (bx/2).*T_in.^2 + (cx).*T_in + (a_0(1)) ;
    ay = (ay/6).*T_in.^3 + (by/2).*T_in.^2 + (cy).*T_in + (a_0(2)) ;
    az = (az/6).*T_in.^3 + (bz/2).*T_in.^2 + (cz).*T_in + (a_0(3)) ;
    
    % jerk
    jx = (ax/2).*T_in.^2 + (bx).*T_in + (cx) ;
    jy = (ay/2).*T_in.^2 + (by).*T_in + (cy) ;
    jz = (az/2).*T_in.^2 + (bz).*T_in + (cz) ;
    
    % snap
    sx = ax.*T_in + (bx) ;
    sy = ay.*T_in + (by) ;
    sz = az.*T_in + (bz) ;
    
    % create output traj
    Z_out = [px ; py ; pz ;
             vx ; vy ; vz ;
             ax ; ay ; az ;
             jx ; jy ; jz ;
             sx ; sy ; sz ] ;
end