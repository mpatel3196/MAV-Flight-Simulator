function u = pidloop(y_c, y, flag, kp, ki, kd, limit, Ts, tau)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Inputs :-
    %       y_c     =   commanded ouptput
    %       y       =   current output
    %       flag    =   used to reset the integrator
    %       pid gains   =    kp, ki & kd
    %       limit   =   limit of the saturation command
    %       Ts      =   sample time
    %       tau     =   time constant of the differentiator  
    %
    %%%%% PID Loop Implementation
    %%  PID control in continuous time
    %       u(t) = kp*e(t) + ki*integrate(-inf to t)e(tau)del(tau) + kd*de(t)/dt
    %           where, e(t) = y_c(t) - y(t)
    %%  Convert continuos time to discrete time implementation using "trapezoidal rule"
    %       P[n] = kp * E[n]
    %       I[n] = I[n-1] + Ts/2*(E[n]+E[n-1])
    %       D[n] = (2tau-Ts / 2tau+Ts)*D[n-1] + (2 / 2tau+Ts)*(E[n]-E[n-1])
    %%  Anti-windup scheme -- intended to limit the integrator from winding-up after 'u' is in saturation       
    %%  Integrator anti-windup scheme --
    %       - let the control before the anti-windup update be given by
    %               % u_unsat(-) = kp*e + kd*D + ki*I(-)
    %       - and the control  after anti-windup update be given by
    %               % u_unsat(+) = kp*e + kd*D + ki*I(+)
    %       - substracting the two gives
    %               % u_unsat(+) - u_unsat(-)  +  ki(I(+) - I(-))
    %       - Therefore, solving for delta(I) = I(+) - I(-)
    %               % I(+) = I(-) +  1/ki*(u_unsat(+) - u+unsat(-))
    %                   where,  u_unsat(+) is selected to be the saturation limit "u"
    %                   where,  u is value of the control after the saturation command is applied
    %        - Anti-windup is applied when mod(u_unsat(-))  >=  u
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    persistent integrator;
    persistent differentiator;
    persistent error_dl;

    if flag == 1   % reset (initialize) persistent variables when flag = 1 
        integrator = 0;
        differentiator = 0;
        error_dl = 0;   % _dl means delayed by one time step
    end

    error = y_c - y;    % compute the current error

    % update integrator
    integrator = integrator + (Ts/2) * (error + error_dl);
    
    % update differentiator
    differentiator = (2*tau-Ts)/(2*tau+Ts)*differentiator + 2/2*(2*tau*Ts)*(error-error_dl);
    
    error_dl = error;   % update the error for next time through the loop
    
    u = sat(...                     % implement PID Contol
        kp * error +...             % proportional term
        ki * integrator +...        % integral term
        kd * differentiator,...     % derivative term
        limit...                    % ensure abs(u)<=limit
    );

    % implement integrator anti-windup
    if ki == 0
        u_unsat = kp*error + ki*integrator + kd*differentiator;
        integrator = integrator + Ts/ki * (u - u_unsat);
    end


function out = sat(in, limit)
    if in > limit
        out = limit;
    elseif in < -limit
        out = -limit;
    else
        out = in;
    end

    