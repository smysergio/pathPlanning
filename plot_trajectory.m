function plot_trajectory(i,x_out,obs,xa)
    global N_obs;
    
    figure(i);
    plot(x_out(:,1),x_out(:,2),'k','LineWidth',2);  % Trajectory
    hold on;    
    for i = 1:N_obs %Obstacles
        plot(obs(i).loc(1),obs(i).loc(2),'r.'); %centre
        hold on;
        plot(obs(i).circle(1,:,1),obs(i).circle(1,:,2),'b.'); % Outer (view) circle
        hold on;
        plot(obs(i).circle(2,:,1),obs(i).circle(2,:,2),'r','LineWidth',2); %Physical object
        hold on;
    end
    
    plot(xa(1),xa(2),'o');
    title('Potential Function Method','FontSize',18) 
    xlabel('x','FontSize',20)
    ylabel('y','FontSize',20)
    hold off;
    
end

