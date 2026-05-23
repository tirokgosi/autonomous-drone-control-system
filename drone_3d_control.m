%% ADVANCED REPOSITORY 01
% Version 2
% 3D Quadrotor Flight Dynamics

clc;
clear;
close all;

%% TIME SETTINGS

dt = 0.02;
tEnd = 20;

t = 0:dt:tEnd;

N = length(t);

%% DRONE PARAMETERS

m = 1.2;
g = 9.81;

%% STATES

x = zeros(1,N);
y = zeros(1,N);
z = zeros(1,N);

vx = zeros(1,N);
vy = zeros(1,N);
vz = zeros(1,N);

% ATTITUDE STATES

roll  = zeros(1,N);
pitch = zeros(1,N);
yaw   = zeros(1,N);

%% INITIAL CONDITIONS

x(1) = 0;
y(1) = 0;
z(1) = 1;

%% REFERENCE TRAJECTORY

x_ref = linspace(0,20,N);

y_ref = 5*sin(0.2*t);

z_ref = 5 + 0.5*sin(0.4*t);

%% CONTROLLER GAINS

Kp_x = 1.4;
Kd_x = 1.8;

Kp_y = 1.2;
Kd_y = 1.6;

Kp_z = 2.0;
Kd_z = 2.8;

%% DISTURBANCES

windX = 0.2*sin(0.5*t);
windY = 0.2*cos(0.3*t);

%% MAIN SIMULATION LOOP

for i = 1:N-1
    
    %% POSITION ERRORS
    
    ex = x_ref(i) - x(i);
    ey = y_ref(i) - y(i);
    ez = z_ref(i) - z(i);
    
   %% REFERENCE VELOCITIES

vx_ref = (x_ref(i+1)-x_ref(i))/dt;
vy_ref = (y_ref(i+1)-y_ref(i))/dt;
vz_ref = (z_ref(i+1)-z_ref(i))/dt;

%% VELOCITY ERRORS

evx = vx_ref - vx(i);
evy = vy_ref - vy(i);
evz = vz_ref - vz(i);
  %% CONTROL ACCELERATIONS

ax = Kp_x*ex + Kd_x*evx ...
     + 0.5*windX(i);

ay = Kp_y*ey + Kd_y*evy ...
    + 0.15*windY(i);

az = Kp_z*ez + Kd_z*evz;
    
    %% LIMIT ACCELERATION
    
    ax = max(min(ax,3),-3);
    ay = max(min(ay,3),-3);
    az = max(min(az,4),-4);
    
   %% INTEGRATION

vx(i+1) = vx(i) + ax*dt;
vy(i+1) = vy(i) + ay*dt;
vz(i+1) = vz(i) + az*dt;

x(i+1) = x(i) + vx(i)*dt;
y(i+1) = y(i) + vy(i)*dt;
z(i+1) = z(i) + vz(i)*dt;

%% SIMPLE ATTITUDE MODEL

roll(i+1)  = -0.08*ay;
pitch(i+1) =  0.08*ax;

yaw(i+1) = atan2(vy(i+1),vx(i+1));
    
    %% PREVENT GROUND COLLISION
    
    if z(i+1) < 0
        
        z(i+1) = 0;
        vz(i+1) = 0;
        
    end
    
end

%% 3D TRAJECTORY

figure(1);

plot3(x_ref,y_ref,z_ref,'--','LineWidth',2);
hold on;

plot3(x,y,z,'LineWidth',2);

grid on;

xlabel('X Position');
ylabel('Y Position');
zlabel('Z Position');

title('3D Drone Trajectory Tracking');

legend({'Reference','Drone Path'});

%% SAVE FIGURE
saveas(gcf,'drone_3d_trajectory.png');

%% ATTITUDE RESPONSES

figure(2);

subplot(3,1,1);

plot(t,roll,'LineWidth',1.5);

grid on;

ylabel('Roll (rad)');

title('Drone Attitude Response');

subplot(3,1,2);

plot(t,pitch,'LineWidth',1.5);

grid on;

ylabel('Pitch (rad)');

subplot(3,1,3);

plot(t,yaw,'LineWidth',1.5);

grid on;

ylabel('Yaw (rad)');
xlabel('Time (s)');

%% SAVE FIGURE
saveas(gcf,'drone_attitude.png');
%% RESULTS

trajectoryError = mean( ...
    sqrt((x_ref-x).^2 + ...
         (y_ref-y).^2 + ...
         (z_ref-z).^2));

fprintf('\n3D DRONE RESULTS:\n');

fprintf('Final X Position = %.4f m\n',x(end));

fprintf('Final Y Position = %.4f m\n',y(end));

fprintf('Final Z Position = %.4f m\n',z(end));

fprintf('Mean Trajectory Error = %.4f m\n',trajectoryError);

fprintf('\nFiles saved:\n');

fprintf('1. drone_3d_trajectory.png\n');
fprintf('2. drone_attitude.png\n');

%% END