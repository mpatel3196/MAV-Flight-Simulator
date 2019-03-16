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
%    w_ns    = 0; % steady wind - North
%    w_es    = 0; % steady wind - East
%    w_ds    = 0; % steady wind - Down
%    u_wg    = 0; % gust along body x-axis
%    v_wg    = 0; % gust along body y-axis    
%    w_wg    = 0; % gust along body z-axis
    
    % define rotation matrix (right handed)
    R_roll = [...
              1, 0, 0;...
              0, cos(phi), sin(phi);...
              0, -sin(phi), cos(phi)];
    R_pitch = [...
              cos(theta), 0, -sin(theta);...
              0, 1, 0;...
              sin(theta), 0, cos(theta)];
    R_yaw = [...
              cos(psi), sin(psi), 0;...
              -sin(psi), cos(psi), 0;...
              0, 0, 1];
    R_I_Body = R_roll*R_pitch*R_yaw;  
    
    % compute wind data in NED
    w_inertia = [w_ns, w_es, w_ds]' + R_I_Body' * [u_wg, v_wg, w_wg]';
    w_n = w_inertia(1);
    w_e = w_inertia(2);
    w_d = w_inertia(3);
    
    w_body = R_I_Body*[w_ns, w_es, w_ds]' + [u_wg, v_wg, w_wg]';
    u_r = u - w_body(1);
    v_r = v - w_body(2);
    w_r = w - w_body(3);
    
    % compute air data
    Va = (u_r^2 + v_r^2 + w_r^2)^0.5;
    alpha = atan(w_r/u_r);
    beta = asin(v_r/(u_r^2 + v_r^2 + w_r^2)^0.5);
    
    % Coeficients
    sigma_Alpha = (1+exp(-P.M*(alpha-P.alpha0))+exp(P.M*(alpha+P.alpha0)))/...
            ((1+exp(-P.M*(alpha-P.alpha0)))*(1+exp(P.M*(alpha+P.alpha0))));
    C_L_Alpha = (1-sigma_Alpha)*(P.C_L_0+P.C_L_alpha*alpha) + ...
            sigma_Alpha*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
    C_D_Alpha = P.C_D_p + ((P.C_L_0+P.C_L_alpha*alpha)^2)/(pi*P.e*P.b^2/P.S_wing);
        
    C_X = -C_D_Alpha*cos(alpha)+C_L_Alpha*sin(alpha);
    C_X_q = -P.C_D_q*cos(alpha)+P.C_L_q*sin(alpha);
    C_X_delta_e = -P.C_D_delta_e*cos(alpha)+P.C_L_delta_e*sin(alpha);
    C_Z = -C_D_Alpha*sin(alpha)-C_L_Alpha*cos(alpha);
    C_Z_q = -P.C_D_q*sin(alpha)-P.C_L_q*cos(alpha);
    C_Z_delta_e = -P.C_D_delta_e*sin(alpha)-P.C_L_delta_e*cos(alpha);
    
    % compute external forces and torques on aircraft
    Force(1) =  -P.mass*P.gravity*sin(theta) + ...
        0.5*P.rho*Va^2*P.S_wing * (C_X + C_X_q*(P.c/(2*Va))*q + C_X_delta_e*delta_e) + ...
        0.5*P.rho*P.S_prop*P.C_prop * ((P.k_motor*delta_t)^2-Va^2);
    Force(2) =  P.mass*P.gravity*cos(theta)*sin(phi) + ...
        0.5*P.rho*Va^2*P.S_wing * (P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*(P.b/(2*Va))*p ...
                            + P.C_Y_r*(P.b/(2*Va))*r + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r);
    Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi) + ...
        0.5*P.rho*Va^2*P.S_wing * (C_Z + C_Z_q*(P.c/(2*Va))*q + C_Z_delta_e*delta_e);
    
    Torque(1) = 0.5*P.rho*Va^2*P.S_wing * P.b * ...
        (P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*(P.b/(2*Va))*p + P.C_ell_r*(P.b/(2*Va))*r +...
                                P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r) - ...
        P.k_T_P*(P.k_Omega*delta_t)^2;
    Torque(2) = 0.5*P.rho*Va^2*P.S_wing * P.c * ...
        (P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*(P.c/(2*Va))*q + P.C_m_delta_e*delta_e);   
    Torque(3) = 0.5*P.rho*Va^2*P.S_wing * P.b * ...
        (P.C_n_0 + P.C_n_beta*beta + P.C_n_p*(P.b/(2*Va))*p + P.C_n_r*(P.b/(2*Va))*r + ...
                                P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r);

   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



