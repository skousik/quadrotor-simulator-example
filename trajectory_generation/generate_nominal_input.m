function U = generate_nominal_input(T,Z)
% U = generate_nominal_input(T,Z)
%
% Given a 1-by-N time array T and associated trajectory Z as a 15-by-N,
% compute the nominal thrust and body moments for a quadrotor as in
% http://www-personal.acfr.usyd.edu.au/spns/cdm/papers/Mellinger.pdf
%
% Author: Shreyas Kousik
% Created: 13 July 2020
% Updated: not yet

    % get accelerations
    ax = Z(7,:) ;
    ay = Z(8,:) ;
    az = Z(9,:) ;

    % thrust
    g = [0;0;-9.81] ; % gravity compensation
    f = sum(([ax ; ay ; az] - g).^2,1).^(1/2) ;

    % thrust direction
    n_f = [ax;ay;az] - repmat(g,1,length(ax));

    % check for zeros
    n_zero = all(n_f == 0,1) ;
    n_f(:,n_zero) = repmat([0;0;1],1,sum(n_zero)) ;
    n = n_f./repmat(vecnorm(n_f,2,1),3,1) ;

    % get thrust directions 1 ms ahead of current ones
    T_adv = T + 0.001 ;
    n_adv = [match_trajectories(T_adv(1:end-1),T,n), n(:,end)] ;

    % get body rates in inertial frame numerically
    w = cross(n,n_adv,1) ;

    % create nominal input
    U = [f ; w] ;
end