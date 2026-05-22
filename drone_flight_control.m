%% ADVANCED REPOSITORY 01
% Autonomous Drone Flight Control System
% Stable 2D Drone Simulation

clc;
clear;
close all;

%% TIME SETTINGS

dt = 0.02;
tEnd = 20;

t = 0:dt:tEnd;

N = length(t);

%% DRONE PARAMETERS

m = 1.0;
g = 9.81;

%% STATES

x  = zeros(1,N);
z  = zeros(1,N);

vx = zeros(1,N);
vz = zeros(1,N);

%% INITIAL CONDITIONS

x(1) = 0;
z(1) = 1;

%% REFERENCE TRAJECTORY

x_ref = linspace(0,20,N);

z_ref = 5 + 0.5*sin(0.4*t);

%% CONTROLLER GAINS

Kp_x = 1.2;
Kd_x = 1.8;

Kp_z = 4.5;
Kd_z = 3.0;

%% WIND DISTURBANCE

wind = 0.3*sin(0.5*t);

%% CONTROL INPUTS

ax_cmd = zeros(1,N);
az_cmd = zeros(1,N);

%% MAIN SIMULATION LOOP

for i = 1:N-1
    
    %% POSITION ERRORS
    
    ex = x_ref(i) - x(i);
    ez = z_ref(i) - z(i);
    
    %% VELOCITY ERRORS
    
    evx = -vx(i);
    evz = -vz(i);
    
    %% CONTROLLER
    
    ax_cmd(i) = ...
        Kp_x*ex + Kd_x*evx;
    
    az_cmd(i) = ...
        Kp_z*ez + Kd_z*evz;
    
    %% LIMIT ACCELERATION
    
    ax_cmd(i) = ...
        max(min(ax_cmd(i),3),-3);
    
    az_cmd(i) = ...
        max(min(az_cmd(i),5),-5);
    
    %% SYSTEM DYNAMICS
    
    ax = ax_cmd(i) + wind(i);
    
    az = az_cmd(i);
    
    %% INTEGRATE
    
    vx(i+1) = vx(i) + ax*dt;
    vz(i+1) = vz(i) + az*dt;
    
    x(i+1) = x(i) + vx(i)*dt;
    z(i+1) = z(i) + vz(i)*dt;
    
    %% PREVENT GROUND COLLISION
    
    if z(i+1) < 0
        
        z(i+1) = 0;
        vz(i+1) = 0;
        
    end
    
end

%% TRAJECTORY FIGURE

figure(1);

plot(x_ref,z_ref,'--','LineWidth',2);
hold on;

plot(x,z,'LineWidth',2);

grid on;

xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');

title('Drone Trajectory Tracking');

legend({'Reference','Drone Path'});

%% SAVE FIGURE 1
saveas(gcf,'drone_trajectory.png');

%% STATES FIGURE

figure(2);

subplot(2,1,1);

plot(t,x,'LineWidth',1.5);
hold on;

plot(t,x_ref,'--');

grid on;

ylabel('X Position');

legend({'Actual','Reference'});

subplot(2,1,2);

plot(t,z,'LineWidth',1.5);
hold on;

plot(t,z_ref,'--');

grid on;

ylabel('Z Position');

xlabel('Time (s)');

legend({'Actual','Reference'});

%% SAVE FIGURE 2
saveas(gcf,'drone_states.png');

%% ANIMATION

figure(3);

filename = 'drone_animation.gif';

for i = 1:10:N
    
    clf;
    
    plot(x_ref,z_ref,'--');
    hold on;
    
    plot(x(1:i),z(1:i),'b','LineWidth',2);
    
    % DRONE BODY
    droneWidth = 0.6;
    
    droneX = [x(i)-droneWidth ...
              x(i)+droneWidth];
          
    droneZ = [z(i) z(i)];
    
    plot(droneX,droneZ,'r','LineWidth',5);
    
    % DRONE CENTER
    plot(x(i),z(i),'ko',...
        'MarkerSize',8,...
        'MarkerFaceColor','k');
    
    xlim([0 22]);
    ylim([0 8]);
    
    xlabel('Horizontal Position (m)');
    ylabel('Vertical Position (m)');
    
    title('Autonomous Drone Animation');
    
    grid on;
    
    drawnow;
    
    frame = getframe(gcf);
    im = frame2im(frame);
    
    [A,map] = rgb2ind(im,256);
    
    if i == 1
        
        imwrite(A,map,filename,...
            'gif',...
            'LoopCount',Inf,...
            'DelayTime',0.05);
        
    else
        
        imwrite(A,map,filename,...
            'gif',...
            'WriteMode','append',...
            'DelayTime',0.05);
        
    end
    
end

%% PERFORMANCE METRICS

trajectoryError = mean( ...
    sqrt((x_ref - x).^2 + ...
         (z_ref - z).^2));

%% RESULTS

fprintf('\nAUTONOMOUS DRONE RESULTS:\n');

fprintf('Final X Position = %.4f m\n',x(end));

fprintf('Final Z Position = %.4f m\n',z(end));

fprintf('Mean Trajectory Error = %.4f m\n',trajectoryError);

fprintf('\nFiles saved successfully:\n');

fprintf('1. drone_trajectory.png\n');
fprintf('2. drone_states.png\n');
fprintf('3. drone_animation.gif\n');

%% END