%%%%%%%
P.gravity = 9.8;
   
%physical parameters of airframe
P.mass = 1.56;
P.Jx   = 0.1147;
P.Jy   = 0.0576;
P.Jz   = 0.1712;
P.Jxz  = 0.0015;

% aerodynamic coefficients
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;
P.rho           = 1.2682;
P.c             = 0.3302;
P.b             = 1.4224;
P.S_wing        = 0.2589;
P.S_prop        = 0.0314;
P.k_motor       = 20;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;


P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;



% wind parameters
P.wind_n = 0;
P.wind_e = 0;
P.wind_d = 0;
P.L_u = 1250;
P.L_v = 1750;
P.L_w = 1750;
P.sigma_u = 1; 
P.sigma_v = 1;
P.sigma_w = 0.7;
% P.sigma_wx = 0; 
% P.sigma_wy = 0;
% P.sigma_wz = 0;
P.Va0 = 35;

% autopilot sample rate
P.Ts = 0.01;

% compute trim conditions using 'mavsim_chap5_trim.mdl'
P.Va    = 35;         % desired airspeed (m/s)
gamma = 5*pi/180;  % desired flight path angle (radians)
R     = 0;         % desired radius (m) - use (+) for right handed orbit, 
                    %                          (-) for left handed orbit
% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -100;  % initial Down position (negative altitude)
P.u0     = P.Va; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0*pi/180;  % initial heading angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
   = compute_tf_model(x_trim,u_trim,P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

% select gains for roll loop
    % get transfer function data for delta_a to phi
    [num,den]=tfdata(T_phi_delta_a,'v');
    a_phi2 = num(3);
    a_phi1 = den(2);
    % maximum possible aileron command
    delta_a_max = 45*pi/180;
    % Roll command when delta_a_max is achieved
    phi_max = 15*pi/180;
    % pick natural frequency to achieve delta_a_max for step of phi_max
    zeta_roll = 0.7;
    %wn_roll = sqrt(a_phi2*delta_a_max/phi_max);
    wn_roll = sqrt(a_phi2*delta_a_max*sqrt(1-zeta_roll^2)/phi_max);
    
    % set control gains based on zeta and wn
    P.roll_kp = wn_roll^2/a_phi2;
    P.roll_kd = 1.1*(2*zeta_roll*wn_roll - a_phi1)/a_phi2;
    P.roll_ki = 0.1;
    
    % add extra roll damping
    P.roll_kd = P.roll_kd+1;
    
    
% select gains for course loop
   zeta_course = 0.8;
   wn_course = wn_roll/30;
   P.course_kp = 2*zeta_course*wn_course*P.Va/P.gravity;
   P.course_ki = wn_course^2*P.Va/P.gravity;
   P.course_kd = 0;
   
% select gains for sideslip hold
    % get transfer function data for delta_r to vr
    [num,den]=tfdata(T_v_delta_r,'v');
    a_beta2 = num(2);
    a_beta1 = den(2);
    % maximum possible rudder command
    delta_r_max = 20*pi/180;
    % Roll command when delta_r_max is achieved
    vr_max = 3;
    % pick natural frequency to achieve delta_a_max for step of phi_max
    zeta_beta = 0.707;
    P.beta_kp = delta_r_max/vr_max;
    wn_beta = (a_beta2*P.beta_kp+a_beta1)/2/zeta_beta;
    P.beta_ki = 0;%wn_beta^2/a_beta2;
    P.beta_kd = 0;

   
% select gains for the pitch loop
   % get transfer function delta_e to theta
   [num,den]=tfdata(T_theta_delta_e,'v');
   a_theta1 = den(2);
   a_theta2 = den(3);
   a_theta3 = num(3);
   % maximum possible elevator command
   delta_e_max = 45*pi/180;
   % Pitch command when delta_e_max is achieved
   theta_max = 15*pi/180;
   % pick natural frequency to achieve delta_e_max for step of theta_max
   zeta_pitch = 0.9;
   wn_pitch = sqrt(abs(a_theta3)*delta_e_max*sqrt(1-zeta_pitch^2)/theta_max);
   % set control gains based on zeta and wn
   P.pitch_kp = (wn_pitch^2-a_theta2)/a_theta3;
   P.pitch_kd = (2*zeta_pitch*wn_pitch - a_theta1)/a_theta3;
   P.pitch_ki = 0;
   P.K_theta_DC = P.pitch_kp*a_theta3/(a_theta2+P.pitch_kp*a_theta3);

% select gains for altitude loop
   zeta_altitude = 0.7;
   wn_altitude = wn_pitch/40;
   P.altitude_kp = 2*zeta_altitude*wn_altitude/P.K_theta_DC/P.Va;
   P.altitude_ki = wn_altitude^2/P.K_theta_DC/P.Va;
   P.altitude_kd = 0;
 
% airspeed hold using pitch
   [num,den]=tfdata(T_Va_theta,'v');
   a_V1 = den(2);
   zeta_airspeed_pitch = 0.707;
   wn_airspeed_pitch = wn_pitch/10;
   P.airspeed_pitch_kp = (a_V1-2*zeta_airspeed_pitch*wn_airspeed_pitch)/P.K_theta_DC/P.gravity;
   P.airspeed_pitch_ki = -wn_airspeed_pitch^2/P.K_theta_DC/P.gravity;
   P.airspeed_pitch_kd = 0;
 
% airspeed hold using throttle
   [num,den]=tfdata(T_Va_delta_t,'v');
   a_Vt1 = den(2);
   a_Vt2 = num(2);
   zeta_airspeed_throttle = 0.707;
%    wn_airspeed_throttle = 5;   % a value of 5 causes instability...
   wn_airspeed_throttle = 3;
   P.airspeed_throttle_kp = (2*zeta_airspeed_throttle*wn_airspeed_throttle-a_Vt1)/a_Vt1;
   P.airspeed_throttle_ki = wn_airspeed_throttle^2/a_Vt2;
   P.airspeed_throttle_kd = 0;
   P.airspeed_throttle_integrator_gain = a_Vt1/a_Vt2/P.airspeed_throttle_ki;
 
% gains for slideslip
   P.sideslip_kp = .1;
   P.sideslip_kd = -.5;
   P.sideslip_ki = 0;

% climbrate using pitch
   P.climbrate_pitch_kp = 1.5;
   P.climbrate_pitch_ki = 0.5;
   P.climbrate_pitch_kd = .5;
   
   
% altitude zone
P.altitude_take_off_zone = 10;
P.altitude_hold_zone = 30;

% gain on dirty derivative
P.tau = 5;



