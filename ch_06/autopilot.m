function y = autopilot(uu,P)
%
% autopilot for mavsim
%  

    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
%    alpha    = uu(5+NN);  % angle of attack
%    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
%    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
%    bx       = uu(17+NN); % x-gyro bias
%    by       = uu(18+NN); % y-gyro bias
%    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    
    autopilot_version = 2;
        % autopilot_version == 1 <- used for tuning
        % autopilot_version == 2 <- standard autopilot defined in book
        % autopilot_version == 3 <- Total Energy Control for longitudinal AP
    switch autopilot_version
        case 1,
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 2,
           [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 3,
               [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
    end
    y = [delta; x_command];
end
    
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % autopilot_tuning
% %   - used to tune each loop
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

%     mode = 5;
%     switch mode
%         case 1, % tune the roll loop
%             phi_c = chi_c; % interpret chi_c to autopilot as course command
%             delta_a = roll_hold(phi_c, phi, p, P);
%             delta_r = 0; % no rudder
%             % use trim values for elevator and throttle while tuning the lateral autopilot
%             delta_e = P.u_trim(1);
%             delta_t = P.u_trim(4);
%             theta_c = 0;
%         case 2, % tune the course loop
%             if t==0,
%                 phi_c   = course_hold(chi_c, chi, r, 1, P);
%             else
%                 phi_c   = course_hold(chi_c, chi, r, 0, P);
%             end                
%             delta_a = roll_hold(phi_c, phi, p, P);
%             delta_r = 0; % no rudder
%             % use trim values for elevator and throttle while tuning the lateral autopilot
%             delta_e = P.u_trim(1);
%             delta_t = P.u_trim(4);
%             theta_c = 0;
%         case 3, % tune the throttle to airspeed loop and pitch loop simultaneously
%             theta_c = 20*pi/180 + h_c;
%             chi_c = 0;
%             if t==0,
%                 phi_c   = course_hold(chi_c, chi, r, 1, P);
%                 delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
%            else
%                 phi_c   = course_hold(chi_c, chi, r, 0, P);
%                 delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
%             end
%             delta_e = pitch_hold(theta_c, theta, q, P);
%             delta_a = roll_hold(phi_c, phi, p, P);
%             delta_r = 0; % no rudder
%             % use trim values for elevator and throttle while tuning the lateral autopilot
%         case 4, % tune the pitch to airspeed loop 
%             chi_c = 0;
%             delta_t = P.u_trim(4);
%             if t==0,
%                 phi_c   = course_hold(chi_c, chi, r, 1, P);
%                 theta_c = airspeed_with_pitch_hold(Va_c, Va, 1, P);
%            else
%                 phi_c   = course_hold(chi_c, chi, r, 0, P);
%                 theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
%             end
%             delta_a = roll_hold(phi_c, phi, p, P);
%             delta_e = pitch_hold(theta_c, theta, q, P);
%             delta_r = 0; % no rudder
%             % use trim values for elevator and throttle while tuning the lateral autopilot
%         case 5, % tune the pitch to altitude loop 
%             chi_c = 0;
%             if t==0,
%                 phi_c   = course_hold(chi_c, chi, r, 1, P);
%                 theta_c = altitude_hold(h_c, h, 1, P);
%                 delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
%            else
%                 phi_c   = course_hold(chi_c, chi, r, 0, P);
%                 theta_c = altitude_hold(h_c, h, 0, P);
%                 delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
%             end
%             delta_a = roll_hold(phi_c, phi, p, P);
%             delta_e = pitch_hold(theta_c, theta, q, P);
%             delta_r = 0; % no rudder
%             % use trim values for elevator and throttle while tuning the lateral autopilot
%       end
%     %----------------------------------------------------------
%     % create outputs
    
%     % control outputs
%     delta = [delta_e; delta_a; delta_r; delta_t];
%     % commanded (desired) states
%     x_command = [...
%         0;...                    % pn
%         0;...                    % pe
%         h_c;...                  % h
%         Va_c;...                 % Va
%         0;...                    % alpha
%         0;...                    % beta
%         phi_c;...                % phi
%         %theta_c*P.K_theta_DC;... % theta
%         theta_c;
%         chi_c;...                % chi
%         0;...                    % p
%         0;...                    % q
%         0;...                    % r
%         ];
            
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_uavbook
%   - autopilot defined in the uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,      % thrust == zero
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        % use commanded roll angle to regulate heading
        phi_c   = course_hold(chi_c, chi, r, 1, P);
        % use aileron to regulate roll angle
        delta_a = roll_hold(phi_c, phi, p, 1, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
        delta_a = roll_hold(phi_c, phi, p, 0, P);
    end     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    persistent initialize_integrator;
    % initialize persistent variable
    if t==0,
        if h<=P.altitude_take_off_zone,     
            altitude_state = 1;
        elseif h<=h_c-P.altitude_hold_zone, 
            altitude_state = 2;
        elseif h>=h_c+P.altitude_hold_zone, 
            altitude_state = 3;
        else
            altitude_state = 4;
        end
        initialize_integrator = 1;
    end
    
    % implement state machine
    switch altitude_state,
        case 1,  % in take-off zone
            delta_t = 1;
            theta_c = 30*pi/180;
            if h>=P.altitude_take_off_zone
                altitude_state = 2;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
            
        case 2,  % climb zone
            delta_t = 1;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            if h>h_c-P.altitude_hold_zone
                altitude_state = 4;
                initialize_integrator = 1;
            elseif h<P.altitude_take_off_zone
                altitude_state = 1;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
             
        case 3, % descend zone
            delta_t = 0;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            if h<h_c+P.altitude_hold_zone
                altitude_state = 4;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end

        case 4, % altitude hold zone
            delta_t = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            theta_c = altitude_hold(h_c, h, initialize_integrator, P);
            if h<=h_c-P.altitude_hold_zone,
                altitude_state = 2;
                initialize_integrator = 1;
            elseif h>=h_c+P.altitude_hold_zone,
                altitude_state = 3;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
    end
    
    delta_e = pitch_hold(theta_c, theta, q, 1, P);  % putting flag = 0 is giving error why? and giving error_dl=[] in pitch_holf ??
    % artificially saturation delta_t
    delta_t = sat(delta_t,1,0);

    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];

    y = [delta; x_command];

    end


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % autopilot_TECS
% %   - longitudinal autopilot based on total energy control systems
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

%     %----------------------------------------------------------
%     % lateral autopilot
%     if t==0,
%         % assume no rudder, therefore set delta_r=0
%         delta_r = 0;%coordinated_turn_hold(beta, 1, P);
%         phi_c   = course_hold(chi_c, chi, r, 1, P);

%     else
%         phi_c   = course_hold(chi_c, chi, r, 0, P);
%         delta_r = 0;%coordinated_turn_hold(beta, 0, P);
%     end
%     delta_a = roll_hold(phi_c, phi, p, P);     
  
    
%     %----------------------------------------------------------
%     % longitudinal autopilot based on total energy control
    
    
%     delta_e = 0;
%     delta_t = 0;
 
    
%     %----------------------------------------------------------
%     % create outputs
    
%     % control outputs
%     delta = [delta_e; delta_a; delta_r; delta_t];
%     % commanded (desired) states
%     x_command = [...
%         0;...                    % pn
%         0;...                    % pe
%         h_c;...                  % h
%         Va_c;...                 % Va
%         0;...                    % alpha
%         0;...                    % beta
%         phi_c;...                % phi
%         %theta_c*P.K_theta_DC;... % theta
%         theta_c;
%         chi_c;...                % chi
%         0;...                    % p
%         0;...                    % q
%         0;...                    % r
%         ];
            
%     y = [delta; x_command];
 
% end
   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%  LATERAL AUTOPILOT  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%  roll_hold                                               %%%%
%%%%%%  - regulate roll using aileron                           %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p,flag, P)
    persistent integrator;
    persistent error_d1;
    % initialize persistent variables at beginning of simulation
    if flag==1;
        integrator = 0;
        error_d1   = 0; % error at last sample (d1 - delayed by one sample)
    end

    % compute the current error
    error = phi_c - phi;

    % update the integrator
    integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule

    % proportional term
    up = P.roll_kp * error;

    % intergral term
    ui = P.roll_ki * integrator;

    % derivative term
    ud = -P.roll_kd * p;   % assumption: p == del(phi)/dt

    % implement PID control
    delta_a = sat(up + ui + ud, 45*pi/180, -45*pi/180);

    % implement integrator anti-windup
    if P.roll_ki~=0,
        delta_a_unsat = up + ui + ud;
        k_antiwindup = P.Ts/P.roll_ki;
        integrator = integrator + k_antiwindup*(delta_a - delta_a_unsat);
    end

    % update persistent variables
    error_d1 = error;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%  course_hold                                             %%%%
%%%%%%  - regulate heading using the roll command               %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function phi_c = course_hold(chi_c, chi, r, flag, P)
    persistent integrator;
    persistent error_d1;
    % initialize persistent variables at beginning of simulation
    if flag==1,
    integrator = 0; 
    error_d1   = 0; % error at last sample (d1-delayed by one sample)
    end

    % compute the current error
    error = chi_c - chi;

    % update the integrator
    if abs(error)>15*pi/180;
        integrator = 0;
    else
        integrator = integrator + (P.Ts/2)*(error + error_d1);
    end

    % proportional term
    up = P.course_kp * error;

    % integrator term
    ui = P.course_ki * integrator;

    % derivative term
    ud = -P.course_kd * r;

    
    % implement PID contol
    phi_c = sat(up + ui + ud, 45*pi/180, -45*pi/180);

    % implement integrator anti-windup
    if P.course_ki~=0,
        phi_c_unsat = up+ui+ud;
        k_antiwindup = P.Ts/P.course_ki;
        integrator = integrator + k_antiwindup*(phi_c-phi_c_unsat);
    end

    % update persistent variables
    error_d1 = error;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%  coordinated_turn_hold                                   %%%%
%%%%%%  - sideslip with rudder                                  %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_r = coordinated_turn_hold(v, flag, P)
    persistent integrator;
    persistent error_d1;
    % initialize persistent variables at beginning of simulation
    if flag==1,
        integrator = 0; 
        error_d1   = 0; 
    end
   
    % compute the current error
    error = -v;
    
    % update the integrator
    integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
    
    % proportional term
    up = P.sideslip_kp * error;
    
    % integral term
    ui = P.sideslip_ki * integrator;
    
    % derivative term
    ud = 0;%-P.sideslip_kd * r;
    
    
    % implement PID control
    theta_r = sat(up + ui + ud, 30*pi/180, -30*pi/180);
    
    % implement integrator antiwindup
    if P.sideslip_ki~=0,
      theta_r_unsat = up + ui + ud;
      k_antiwindup = P.Ts/P.sideslip_ki;
      integrator = integrator + k_antiwindup*(theta_r-theta_r_unsat);
    end
    
    % update persistent variables
    error_d1 = error;
   
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%  LONGITUDINAL AUTOPILOT  %%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%  pitch_hold                                              %%%%
%%%%%%  - regulate pitch using elevator                         %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_hold(theta_c, theta, q, flag, P)
    persistent integrator;
    persistent error_d1;
    % initialize persistent variables at beginning of simulation
    if flag==1,
        integrator = 0; 
        error_d1   = 0; % error at last sample (d1-delayed by one sample)
    end
 
    % compute the current error
    error = theta_c - theta;

    % update the integrator
    integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
    
    % proportional term
    up = P.pitch_kp * error;

    % integral term
    ui = P.pitch_ki * integrator;

    % derivative term
    ud = -P.pitch_kd * q;


    % implement PID control
    delta_e = sat(up + ui + ud, 45*pi/180, -45*pi/180);
    
    % implement integrator anti-windup
    if P.pitch_ki~=0,
        delta_e_unsat = up + ui + ud;
        k_antiwindup = P.Ts/P.pitch_ki;
        integrator = integrator + k_antiwindup*(delta_e-delta_e_unsat);
    end

    % update persistent variables
    error_d1 = error;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%  climbrate_with_pitch_hold                               %%%%
%%%%%%  - regulate climbrate using pitch angle                  %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [theta_c,hdot_] = climbrate_with_pitch_hold(hdot_c, h, flag, P)
    persistent integrator;
    persistent differentiator;
    persistent differentiator_d1;
    persistent error_d1;
    persistent hdot;
    persistent h_d1;
    % initialize persistent variables at beginning of simulation    
    if flag==1
        integrator        = 0; 
        differentiator    = 0;
        differentiator_d1 = 0;
        error_d1          = 0; 
        hdot              = 0;
        h_d1              = h;
    end

    % compute current climbrate using dirty derivative
    hdot = ((2*P.tau-P.Ts)*hdot + 2*(h-h_d1))/(2*P.tau+P.Ts);

    
    % compute the current error
    error = hdot_c - hdot;

    % update the integrator
    integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule

    % update the differentiator
    differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
    + (2/(2*P.tau+P.Ts))*(error - error_d1);

    % proportional term
    up = P.climbrate_pitch_kp * error;

    % integral term
    ui = P.climbrate_pitch_ki * integrator;

    % derivative term
    ud = P.climbrate_pitch_kd * differentiator;


    % implement PID control
    theta_c = sat(up + ui + ud, 30*pi/180, -30*pi/180);

    % implement integrator antiwindup
    if P.climbrate_pitch_ki~=0,
        theta_c_unsat = up + ui + ud;
        k_antiwindup = P.Ts/P.climbrate_pitch_ki;
        integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
    end

    % update persistent variables
    error_d1 = error;
    differentiator_d1 = differentiator;
    h_d1 = h;

    hdot_ = hdot;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%  airspeed_with_pitch_hold                                %%%%
%%%%%%  - regulate airspeed using pitch angle                   %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
    persistent integrator;
    persistent differentiator;
    persistent differentiator_d1;
    persistent error_d1;
    % initialize persistent variables at beginning of simulation
    if flag==1,
        integrator = 0; 
        differentiator = 0;
        differentiator_d1 = 0;
        error_d1   = 0; 
    end
   
    % compute the current error
    error = Va_c - Va;
    
    % update the integrator
    integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
    
    % update the differentiator
    differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
        + (2/(2*P.tau+P.Ts))*(error - error_d1);
    
    % proportional term
    up = P.airspeed_pitch_kp * error;
    
    % integral term
    ui = P.airspeed_pitch_ki * integrator;
    
    % derivative term
    ud = P.airspeed_pitch_kd * differentiator;
    
    
    % implement PID control
    theta_c = sat(up + ui + ud, 30*pi/180, -30*pi/180);
    
    % implement integrator antiwindup
    if P.airspeed_pitch_ki~=0,
      theta_c_unsat = up + ui + ud;
      k_antiwindup = P.Ts/P.airspeed_pitch_ki;
      integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
    end
  
    % update persistent variables
    error_d1 = error;
    differentiator_d1 = differentiator;
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%  airspeed_with_throttle_hold                             %%%%
%%%%%%  - regulate airspeed using throttle                      %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
    persistent integrator;
    persistent differentiator;
    persistent differentiator_d1;
    persistent error_d1;
    % initialize persistent variables at beginning of simulation
    if flag==1,
        integrator = 0; 
        differentiator = 0;
        differentiator_d1 = 0;
        error_d1   = 0; 
    end
   
    % compute the current error
    error = Va_c - Va;
    
    % update the integrator
    integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
    
    % update the differentiator
    differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
        + (2/(2*P.tau+P.Ts))*(error - error_d1);
    
    % proportional term
    up = P.airspeed_throttle_kp * error;
    
    % integral term
    ui = P.airspeed_throttle_ki * integrator;
    
    % derivative term
    ud = P.airspeed_throttle_kd * differentiator;  
    
    % implement PID control
    delta_t = sat(P.u_trim(4)+up + ui + ud, 1, 0);
    
    % implement integrator anti-windup
    if P.airspeed_throttle_ki~=0,
      delta_t_unsat = P.u_trim(4) + up + ui + ud;
      k_antiwindup = P.Ts/P.airspeed_throttle_ki;
      integrator = integrator + k_antiwindup*(delta_t-delta_t_unsat);
    end
    
    % update persistent variables
    error_d1 = error;
    differentiator_d1 = differentiator;
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%  altitude_hold                                           %%%%
%%%%%%  - regulate altitude using pitch angle                   %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = altitude_hold(h_c, h, flag, P)
    persistent integrator;
    persistent differentiator;
    persistent differentiator_d1;
    persistent error_d1;
    % initialize persistent variables at beginning of simulation
    if flag==1,
        integrator = 0; 
        differentiator = 0;
        differentiator_d1 = 0;
        error_d1   = 0; 
    end
   
    % compute the current error
    error = h_c - h;
    
    % update the integrator
    integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
    
    % update the differentiator
    differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
        + (2/(2*P.tau+P.Ts))*(error - error_d1);
    
    % proportional term
    up = P.altitude_kp * error;
    
    % integral term
    ui = P.altitude_ki * integrator;
    
    % derivative term
    ud = P.altitude_kd * differentiator;
    
    
    % implement PID control
    theta_c = sat(up + ui + ud, 30*pi/180, -30*pi/180);
    
    % implement integrator anti-windup
    if P.altitude_ki~=0,
      theta_c_unsat = up + ui + ud;
      k_antiwindup = P.Ts/P.altitude_ki;
      integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
    end
    
    % update persistent variables
    error_d1 = error;
    differentiator_d1 = differentiator;
  
end
  




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end
  


