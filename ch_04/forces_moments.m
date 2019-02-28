% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % PARTH == precalculating angles values
    Ctheta = cos(theta);
    Stheta = sin(theta);
    Ttheta = tan(theta);
    Cpsi   = cos(psi);
    Spsi   = sin(psi);
    Cphi   = cos(phi);
    Sphi   = sin(phi);
    
    % compute wind data in NED
    w_n = w_ns + u_wg;
    w_e = w_es + v_wg;
    w_d = w_ds + w_wg;
    %w_n = w_ns*(Ctheta*Cpsi) + w_es*(Ctheta*Spsi) + w_ds*(-Stheta) + u_wg;
    %w_e = w_ns*(Sphi*Stheta*Cpsi - Cphi*Spsi)+ w_es*(Sphi*Stheta*Spsi + Cphi*Cpsi) + w_ds*(Sphi*Ctheta) + v_wg;
    %w_d = w_ns*(Cphi*Stheta*Cpsi + Sphi*Spsi) + w_es*(Cphi*Stheta*Spsi - Sphi*Cpsi) + w_ds*(Cphi*Ctheta) + w_wg;
    
    % PARTH = airspeed in body-frame Va
    ur = u - w_n;
    vr = v - w_e;
    wr = w - w_d;
    
    % compute air data
    Va = sqrt((ur^2) + (vr^2) + (wr^2));
    %display(Va)
    alpha = atan(wr/ur);
    %display(alpha)
    beta = asin(vr/Va);
    %display(beta)
    
    % compute external forces and torques on aircraft
    % PARTH - Linear Aerodynamic Model
    % F lift
    Force(1) =  0.5*P.rho*(Va^2)*P.S_wing*(P.C_L_0 + P.C_L_alpha*alpha + 0.5*P.C_L_q*P.c*q/Va + P.C_L_delta_e*delta_e);
    % F drag
    Force(2) =  0.5*P.rho*(Va^2)*P.S_wing*(P.C_D_0 + P.C_D_alpha*alpha + 0.5*P.C_D_q*P.c*q/Va + P.C_D_delta_e*delta_e);
    % F lateral
    Force(3) =  0.5*P.rho*(Va^2)*P.S_wing*(P.C_Y_0 + P.C_Y_beta*beta + 0.5*P.C_Y_p*P.c*p/Va + 0.5*P.C_Y_r*P.c*r/Va + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r);
    
    % Roll Moment
    Torque(1) = 0.5*P.rho*(Va^2)*P.S_wing*(P.C_ell_0 + P.C_ell_beta*beta + 0.5*P.C_ell_p*P.c*p/Va + 0.5*P.C_ell_r*P.c*r/Va + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r);
    % Pitch Moment
    Torque(2) = 0.5*P.rho*(Va^2)*P.S_wing*P.c*(P.C_m_0 + P.C_m_alpha*alpha + 0.5*P.C_m_q*P.c*q/Va + P.C_m_delta_e*delta_e);   
    % Yaw Moment
    Torque(3) = 0.5*P.rho*(Va^2)*P.S_wing*(P.C_n_0 + P.C_n_beta*beta + 0.5*P.C_n_p*P.c*p/Va + 0.5*P.C_n_r*P.c*r/Va + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r);
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



