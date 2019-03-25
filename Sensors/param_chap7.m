P.gravity = 9.81;
   
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 25;%13.5;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
%P.k_motor = 15;
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
%     % HACK: prop has too much power for aerosonde
%     P.C_prop = 0.5;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prop parameters
P.D_prop = 20*(0.0254);     % prop diameter in m

% Motor parameters
P.K_V = 145.;                   % from datasheet RPM/V
P.KQ = (1. / P.K_V) * 60. / (2. * pi);  % KQ in N-m/A, V-s/rad
P.R_motor = 0.042;              % ohms
P.i0 = 1.5;                     % no-load (zero-torque) current (A)


% Inputs
P.ncells = 12.;
P.V_max = 3.7 * P.ncells;  % max voltage for specified number of battery cells

% Coeffiecients from prop_data fit
P.C_Q2 = -0.01664;
P.C_Q1 = 0.004970;
P.C_Q0 = 0.005230;
P.C_T2 = -0.1079;
P.C_T1 = -0.06044;
P.C_T0 = 0.09357;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P.gamma = P.Jx*P.Jz-P.Jxz^2;
P.gamma_1 = (P.Jxz*(P.Jx-P.Jy+P.Jz))/P.gamma;
P.gamma_2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/P.gamma;
P.gamma_3 = P.Jz/P.gamma;
P.gamma_4 = P.Jxz/P.gamma;
P.gamma_5 = (P.Jz-P.Jx)/P.Jy;
P.gamma_6 = P.Jxz/P.Jy;
P.gamma_7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/P.gamma;
P.gamma_8 = P.Jx/P.gamma;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va = 35;        % m/s (~85 mph)
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = Inf;       % desired radius (m) - use (+) for right handed orbit, 
h0    = 0  % initial altitude

% autopilot sample rate
P.Ts = 0.01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.Va; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate
%                          (-) for left handed orbit

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -h0;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Autopilot Gains
P.tau = 5;  % gain on dirty derivative
P.altitude_take_off_zone = 10;
P.altitude_hold_zone = 10;
P.take_off_throttle = 1;
P.climbzone_throttle = 1;
P.descent_trottle = 0; %0.61;
P.altitudehold_throttle =0.61; %u_trim(4) ;% +  airspeed from throttle control;
P.theta_c_max = 30*pi/180; % maximum pitch angle command
P.phi_max = 45*pi/180;

%%% Roll Attitude Hold
   % inputs :-
      [num,den] = tfdata(T_phi_delta_a, 'v');
      a_phi1 = den(2);     % transfer function coefficient
      a_phi2 = num(3);     % transfer function coefficient
      % P.Va      % nominal airspeed
      delta_a_max = 45*pi/180;   % aileron limit   % max possible aileron command
   % tuning parameters :-
      e_phi_max = 10*pi/180;    % saturaton limit  % maxmimum anticipated roll error
      zeta_phi = 0.8;   % zeta_roll    % damping coefficient
      P.ki_phi = 0;   % P.roll_ki    % integrator gain
   % computing the natural frequency :-
      wn_phi   = sqrt(abs(a_phi2)*delta_a_max/e_phi_max);   % natural frequency of the inner loop
   % computing gains :-
      P.kd_phi = 0.1584;%(2*zeta_phi*wn_phi/a_phi2);
      P.kp_phi = 0.47;%sign(a_phi2)*delta_a_max/e_phi_max;
      P.kd_phi = P.kd_phi - 0.2*(P.kd_phi); % 20% decreading
      
%%% Course Hold
   % inputs :-
      %%%%
   % tuning parameters :-
      zeta_chi = 0.8;
      WChi     = 8;   % bandwidth separation
   % computing the natural frequincy :-
      wn_chi   = wn_phi/WChi ;   % natural frequency of the outer loop
   % computing gains :-
      P.kp_chi = 1.25;%2*zeta_chi*wn_chi*P.Va/P.gravity;
      P.ki_chi = 0.2;%(wn_chi^2)*P.Va/P.gravity;
      P.kd_chi =  0;

%%% Slideslip Hold
   % inputs :-
      [num,den]=tfdata(T_v_delta_r,'v');
      a_beta2 = num(2);
      a_beta1 = den(2);
      delta_r_max = 45*pi/180;   % rudder limit
   % tuning parameters :-
      e_beta_max = 10*pi/180;    % saturaton limit
      zeta_beta = 0.8;
   % computing the natural frequency :-
      %%%%
   % computing gains :-
      P.kp_beta = 0;%sign(a_beta2)*delta_r_max/e_beta_max;
      P.ki_beta = 0;%(1/a_beta2)*((a_beta1+(a_beta2*P.kp_beta)/2*zeta_beta)^2);
      P.kd_beta = 0;

%%% Pitch Attitide Hold
   % inputs :-
      [num,den]=tfdata(T_theta_delta_e,'v');
      a_theta1 = num(2);
      a_theta2 = den(3);
      a_theta3 = num(3);
      delta_e_max = 45*pi/180;   % elevator limit
   % tuning parameters :-
      e_theta_max = 10*pi/180;    % saturaton limit
      zeta_theta = 0.707;
   % computing the natural frequency :-
      wn_theta = sqrt(a_theta2 + (sign(a_theta3*delta_e_max/e_theta_max)));
   % computing gains :-
      P.kd_theta = -0.7;%(2*zeta_theta*wn_theta - a_theta1)/a_theta3;% -25;
      P.kp_theta = -4.5;%sign(a_theta3)*delta_e_max/e_theta_max;% -250;
      P.ki_theta = 0;
      P.k_theta_DC = P.kp_theta*a_theta3/(a_theta2 + P.kd_theta*a_theta3);  % DC Gain

%%% Altitude Hold using Commanded Pitch
   % inputs :-
      %%%%
   % tuning parameters :-
      zeta_h = 1;
      Wh     = 2;   % bandwidth separation
   % computing the natural frequency :-
      wn_h = wn_theta/Wh;
   % computing gains :-
      P.kp_h = 0.05;%2*zeta_h*wn_h / P.k_theta_DC*P.Va;
      P.ki_h = 0.011;%(wn_h^2)/P.k_theta_DC*P.Va;
      P.kd_h = 0;


%%% Airspeed Hold using Commanded Pitch      (v2 ==> airspeed hold using pitch)
   % inputs :-
      [num,den]=tfdata(T_Va_theta,'v');
      a_V1 = den(2);
   % tuning parameters :-
      zeta_v2 = 1;
      Wv2     = 10;    % bandwidth saperation
   % computing the natural frequency :-
      wn_v2 = wn_theta/Wv2;
   % computing gains :-
      P.kp_v2 = -0.008;%(a_V1 - 2*zeta_v2*wn_v2)/(P.k_theta_DC*P.gravity);
      P.ki_v2 = -0.0019;%-(wn_v2^2)/(P.k_theta_DC*P.gravity);
      P.kd_v2 = 0;

%%% Airspeed Hold using Throttle      (v ==> airspeed hold using throttle)
   % inputs :-
      [num,den]=tfdata(T_Va_delta_t,'v');
      a_V1 = den(2);
      a_V2 = num(2);
   % tuning parameters :-
      zeta_v = 1.3;
   % computing the natural frequency :-
      wn_v = 0.7;
   % computing gains :-
      P.kp_v = 1.25;%(wn_v^2)/a_V1;
      P.ki_v = 0.35;%(2*zeta_v*wn_v - a_V1)/a_V2;
      P.kd_v = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% sensor parameters
P.Ts_gyros = .01;
% sensor parameters
P.sigma_gyro = 0.13*pi/180; % standard deviation of gyros in rad/sec
P.bias_gyro_x = 0;%0.1*pi/180*rand; % bias on x_gyro
P.bias_gyro_y = 0;%0.1*pi/180*rand; % bias on y_gyro
P.bias_gyro_z = 0;%0.1*pi/180*rand; % bias on z_gyro
P.sigma_accel = 0.0025%*9.8; % standard deviation of accelerometers in m/s^2
%P.sigma_static_pres = 0.01*1000; % standard deviation of static pressure sensor in Pascals
%P.sigma_diff_pres = 0.002*1000;  % standard deviation of diff pressure sensor in Pascals


% simulate pressure sensors
P.sigma_abs_pres = 0.01e3; % Pa
P.beta_abs_pres = .125e3; % Pa
P.sigma_diff_pres = 0.002e3; % Pa
P.beta_diff_pres = 0.02e3; % Pa


%%% GPS parameters
P.Ts_gps = 1; % sample rate of GPS in s
P.beta_gps = 1/1100; % 1/s
P.sigma_n_gps = 0.4;
P.sigma_e_gps = 0.4; 
P.sigma_h_gps = 0.7;
P.sigma_Vg_gps = 2.1;
%P.sigma_course_gps = P.sigma_Vg_gps/P.Vg;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%