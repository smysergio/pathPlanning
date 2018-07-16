clear all

Ka = 2; %attractive
Kr = 0.01; %repulsive
Kv = 5; %velocity

%Input Initial Conditions
a = [0; 0;]; % [ax, ay]

%Objective and Obstacles
xa = [1; 1;];


xobs = [0.4; 0.5;];
po = 0.1;

th = 0:pi/50:2*pi;
circle(:,1) = po * cos(th) + xobs(1);
circle(:,2) = po * sin(th) + xobs(2);

% Initial conditions of all states and state derivatives
x = [0; 0; 0; 0;];      % [x, y, vx, yx]
xdot= [0; 0; 0; 0; 0;];    

x_labels = ['x', 'y', 'vx', 'yx'];

% Define parameters for the simulation
end_flag = true;
stepsize = 0.01;
i = 0;							
time = 0;

syms sx sy sKa xax xay;
pa = sqrt( (sx-xax)^2 + (sy-xay)^2 );
Ua = 0.5*sKa*pa^2;

sax = diff(Ua,sx);
say = diff(Ua,sy);

%syms sx sy sKr xobsx xobsy spo;
%pr = sqrt( (sx-xobsx)^2 + (sy-xobsy)^2 );
%Ur = 0.5*sKr*(1/pr + 1/spo)^2;

%sax = diff(Ur,sx)
%say = diff(Ur,sy)

  
while(end_flag)
    
    i = i+1;			% increment counter
    time = time + 0.01;
    t_out(i) = time;	 	% store time
    x_out(i,:) = x;		% store states
    a_out(i,:) = a;
    
    %a(1) = -vpa(subs(sax,sx,x(1)))-Kv*x(3);
    %a(2) = -vpa(subs(say,sy,x(2)))-Kv*x(4);
    p = sqrt(abs(x(1) - xobs(1))^2 + abs(x(2) - xobs(2))^2);
    if ( p < po )
        Urx = -(Kr*(1/po + 1/((x(1) - xobs(1))^2 + (x(2) - xobs(2))^2)^(1/2))*(2*x(1) - 2*xobs(1)))/(2*((x(1) - xobs(1))^2 + (x(2) - xobs(2))^2)^(3/2));
        Ury = -(Kr*(1/po + 1/((x(1) - xobs(1))^2 + (x(2) - xobs(2))^2)^(1/2))*(2*x(2) - 2*xobs(2)))/(2*((x(1) - xobs(1))^2 + (x(2) - xobs(2))^2)^(3/2));
    else
        Urx = 0;
        Ury = 0;    
    end
    
    Uax = (Ka*(2*x(1) - 2*xa(1)))/2;
    Uay = (Ka*(2*x(2) - 2*xa(2)))/2;
    
    a(1) = - Uax - Kv*x(3) - Urx;
    a(2) = - Uay - Kv*x(4) - Ury; 
    
    x = rk4int('model', stepsize, x, a);
    
    if(x(1)>0.99)
        "hey"
    end
    
    if( (abs(x(1)-xa(1))<0.01) && (abs(x(2)-xa(2))<0.01) )
        end_flag = false;
    end
    
    if(mod(time,1000))
        figure(2);
        plot(x_out(:,1),x_out(:,2));
        hold on;
        plot(xobs(1),xobs(2),'o');
        hold on;
        plot(circle(:,1),circle(:,2));
        hold on;
        plot(xa(1),xa(2),'o'); 
        hold off;
    end
end

figure(1);
    plot(t_out,x_out(:,1));
    hold on;
    plot(t_out,x_out(:,2));
    hold off;
    legend("x","y");
figure(2);
    plot(x_out(:,1),x_out(:,2));
    hold on;
    plot(xobs(1),xobs(2),'o');
    hold on;
    plot(circle(:,1),circle(:,2));
    hold on;
    plot(xa(1),xa(2),'o'); 
    hold off;
% END OF SIMULATION PROGRAM

