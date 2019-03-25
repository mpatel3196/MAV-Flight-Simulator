P.gravity = 9.81; % m/s^2
   
%physical parameters of airframe
P.mass = 13.5; % kg
P.Jx   = 0.8244; % kg m^2
P.Jy   = 1.135; % kg m^2
P.Jz   = 1.759; % kg m^2
P.Jxz  = 0.1204; % kg m^2

% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
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
P.C_M_0         = -0.02338;
P.C_M_alpha     = -0.38;
P.C_M_q         = -3.6;
P.C_M_delta_e   = -0.5;
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
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = 5;  % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P.gamma = P.Jx*P.Jz-P.Jxz^2;
P.gamma_1 = (P.Jxz*(P.Jx-P.Jy+P.Jz))/P.gamma;
P.gamma_2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/P.gamma;
P.gamma_3 = P.Jz/P.gamma;
P.gamma_4 = P.Jxz/P.gamma;
P.gamma_5 = (P.Jz-P.Jx)/P.Jy;
P.gamma_6 = P.Jxz/P.Jy;
P.gamma_7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/P.gamma;
P.gamma_8 = P.Jx/P.gamma;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% wind
P.wind_n = 0;
P.wind_e = 0;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06;
P.sigma_v = 1.06;
P.sigma_w = 0.7;
P.Va0 = 25;

% sample time
P.Ts = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Propeller thrust / torque parameters (see addendum by McLain)
% Prop parameters
P.D_prop = 20*(0.0254)     % prop diameter in m

% Motor parameters
P.K_V = 145.                   % from datasheet RPM/V
P.KQ = (1. / P.K_V) * 60. / (2. * pi)  % KQ in N-m/A, V-s/rad
P.R_motor = 0.042              % ohms
P.i0 = 1.5                     % no-load (zero-torque) current (A)


% Inputs
P.ncells = 12.
P.V_max = 3.7 * P.ncells  % max voltage for specified number of battery cells

% Coeffiecients from prop_data fit
P.C_Q2 = -0.01664
P.C_Q1 = 0.004970
P.C_Q0 = 0.005230
P.C_T2 = -0.1079
P.C_T1 = -0.06044
P.C_T0 = 0.09357

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
