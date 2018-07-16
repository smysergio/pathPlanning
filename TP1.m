clear all

global Ka Kr1 Kr2 Kv N_obs;
Ka = 2;         % Attractive
Kr1 = 0.01;     % Repulsive object
Kr2 = 0.001;   % Repulsive view
Kv = 8;         % Velocity

% Initial conditions of all states and state derivatives
x = [0; 0; 0; 0;];      % [x, y, vx, yx]
xdot= [0; 0; 0; 0; 0;];
%Input Initial Conditions
a = [0; 0;]; % [ax, ay]
%Objective coordinates
xa = [1; 1;];

% Defining obstacles
N_obs = 5;
obs(1) = obstacle([0.2, 0.15], 0.025);
obs(2) = obstacle([0.3, 0.4], 0.05);
obs(3) = obstacle([0.5, 0.4], 0.075);
obs(4) = obstacle([0.6, 0.7], 0.05);
obs(5) = obstacle([0.8, 0.75], 0.05);

% Define parameters for the simulation
end_flag = true;
stepsize = 0.01;
i = 0;							
time = 0;

% Used symbolic matlab to obtain the attractive and repulsive functions  
syms sx sy sKa xax xay;
pa = sqrt( (sx-xax)^2 + (sy-xay)^2 );
Ua = 0.5*sKa*pa^2;

sax = diff(Ua,sx)
say = diff(Ua,sy)

syms sx sy sKr xobsx xobsy spo;
pr = sqrt( (sx-xobsx)^2 + (sy-xobsy)^2 );
Ur = 0.5*sKr*(1/pr - 1/spo)^2;

sax = diff(Ur,sx)
say = diff(Ur,sy)

while(end_flag)
    
    i = i+1;                % increment counter
    time = time + 0.01;
     t_out(i) = time;	 	% store time
    x_out(i,:) = x;         % store state. 
    a_out(i,:) = a;
    
    % Attractive Forces
    Ua(1) = (Ka*(2*x(1) - 2*xa(1)))/2; 
    Ua(2) = (Ka*(2*x(2) - 2*xa(2)))/2;
    
    % Repulsive Forces
    Ur = [0, 0];
    for n = 1:N_obs
        Ur = Ur + obs(n).Fr(x);
    end
    
    % Calculate accelerations
    a(1) = - Ua(1) - Kv*x(3) - Ur(1);
    a(2) = - Ua(2) - Kv*x(4) - Ur(2); 
    
    % Differentiate numerically using Runge-Kutta
    x = rk4int('model', stepsize, x, a);
    
    % Check if x is close enough to xa to stop the simulation
    if( (abs(x(1)-xa(1))<0.01) && (abs(x(2)-xa(2))<0.01) )
        end_flag = false;
    end
    % simulation
    if(mod(time,10000))
    %    plot_trajectory(1,x_out,obs,xa);
    end
end

% plot graphs
plot_trajectory(1,x_out,obs,xa);

% END OF SIMULATION PROGRAM

