function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
% u_trim is the trimmed input


Va_trim = sqrt(x_trim(4)^2 + x_trim(5)^2 + x_trim(6)^2) ;
theta_trim = x_trim(8) ;
alpha_trim = atan(x_trim(6)/x_trim(4)) ;
delta_e_trim = u_trim(1) ;
delta_t_trim = u_trim(4) ;

C_P_p = P.gamma_3*P.C_ell_p + P.gamma_4*P.C_n_p;
a_phi1 = -0.25*P.rho*Va_trim*P.S_wing*(P.b^2)*C_P_p;
C_P_delta_a = P.gamma_3*P.C_ell_delta_a + P.gamma_4*P.C_n_delta_a;
a_phi2 = 0.5*P.rho*(Va_trim^2)*P.S_wing*P.b*C_P_delta_a;

a_theta1 = -0.25*P.rho*Va_trim*(P.c^2)*P.S_wing*P.C_m_q/P.Jy;
a_theta2 = -0.5*P.rho*(Va_trim^2)*P.c*P.S_wing*P.C_m_alpha/P.Jy;
a_theta3 = 0.5*P.rho*(Va_trim^2)*P.c*P.S_wing*P.C_m_delta_e/P.Jy;

a_V1 = (P.rho*Va_trim*P.S_wing/P.mass)*(P.C_D_0 + P.C_D_alpha*alpha_trim + P.C_D_delta_e*delta_e_trim) + (P.rho*P.S_prop*P.e*Va_trim/P.mass);
a_V2 = P.rho*P.S_prop*P.e*(P.k_motor^2)*delta_t_trim/P.mass;
a_V3 = P.gravity*cos(theta_trim - alpha_trim);

a_beta1 = -0.5*P.rho*Va_trim*P.S_wing*P.C_Y_beta/P.mass;
a_beta2 = 0.5*P.rho*Va_trim*P.S_wing*P.C_Y_delta_r/P.mass;

    
% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([Va_trim*a_beta2],[1,a_beta1]);

