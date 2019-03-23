% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 


function xhat = estimate_states(uu, P)

   % rename inputs  %%% u[n]
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
   
   %%%%% Low-pass Fileter / State Estimation using Sensor Model Inversion
   
   persistent lpf_gyro_x
   persistent lpf_gyro_y
   persistent lpf_gyro_z
   persistent lpf_static
   persistent lpf_diff
   persistent lpf_accel_x
   persistent lpf_accel_y
   persistent lpf_accel_z
   persistent lpf_gps_n
   persistent lpf_gps_e
   persistent lpf_gps_Vg
   persistent lpf_gps_course
   persistent alphalpf_R;
   persistent alphalpf_A;
   persistent alphalpf_P;
   persistent alphalpf_G;
   persistent alphalpf_T;

   a_R = 50;  % a value of Rate Gyros
   a_A = 10;  % a value of Accelrometers
   a_P = 50;  % a value of Pressure Sensors
   a_G = 50;  % a value of GPS
   a_T = 10;  % a values of theta

   if t == 0
    alphalpf_R = exp(-a_R*P.Ts);   % alpha_lpf value for rate gyros
    alphalpf_A = exp(-a_A*P.Ts);   % alpha_lpf value for Accelerometers
    alphalpf_P = exp(-a_P*P.Ts);   % alpha_lpf value for Pressure Sensors
    alphalpf_G = exp(-a_G*P.Ts);   % alpha_lpf value for GPS
    alphalpf_T = exp(-a_T*P.Ts);   % alpha_lpf value for theta
    lpf_gyro_x = 0;
    lpf_gyro_y = 0;
    lpf_gyro_z = 0;
    lpf_accel_x = 0;
    lpf_accel_y = 0;
    lpf_accel_z = 0;
    lpf_static = P.rho*P.gravity*(-P.pd0);
    lpf_diff = 0.5*P.rho*P.gravity*P.Va;
    lpf_gps_n = P.pn0;
    lpf_gps_e = P.pe0;
    lpf_gps_course = P.psi0;
    lpf_gps_Vg = P.Va;
   end
   
   lpf_gyro_x = alphalpf_R * lpf_gyro_x + (1-alphalpf_R)*y_gyro_x;
   lpf_gyro_y = alphalpf_R * lpf_gyro_y + (1-alphalpf_R)*y_gyro_y;
   lpf_gyro_z = alphalpf_R * lpf_gyro_z + (1-alphalpf_R)*y_gyro_z;
   lpf_static = alphalpf_P * lpf_static + (1-alphalpf_P)*y_static_pres;
   lpf_diff = alphalpf_P * lpf_diff + (1-alphalpf_P)*y_diff_pres;
   lpf_accel_x = alphalpf_A * lpf_accel_x + (1-alphalpf_A)*y_accel_x;
   lpf_accel_y = alphalpf_A * lpf_accel_y + (1-alphalpf_A)*y_accel_y;
   lpf_accel_z = alphalpf_A * lpf_accel_z + (1-alphalpf_A)*y_accel_z;
   lpf_gps_n = alphalpf_G * lpf_gps_n + (1-alphalpf_G)*y_gps_n;
   lpf_gps_e = alphalpf_G * lpf_gps_e + (1-alphalpf_G)*y_gps_e;
   lpf_gps_course = alphalpf_T * lpf_gps_course + (1-alphalpf_T)*y_gps_course;
   lpf_gps_Vg = alphalpf_G * lpf_gps_Vg + (1-alphalpf_G)*y_gps_Vg;

   phat = lpf_gyro_x;
   qhat = lpf_gyro_y;
   rhat = lpf_gyro_z;
   
   hhat = lpf_static/(P.rho*P.gravity);
   
   Vahat = ((2/P.rho)*lpf_diff)^0.5;
   
   phihat = atan(lpf_accel_y/lpf_accel_z);
   thetahat = asin(lpf_accel_x/P.gravity);
   
   
   pnhat = lpf_gps_n;
   pehat = lpf_gps_e;
   chihat = lpf_gps_course;
   
   Vghat = lpf_gps_Vg;
   
   wnhat = 0;
   wehat = 0;
   psihat = lpf_gps_course;

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
end
