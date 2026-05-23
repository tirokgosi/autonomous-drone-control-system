%% ADVANCED REPOSITORY 01
% Version 4
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

% ANGULAR RATES

p = zeros(1,N);   % Roll rate
q = zeros(1,N);   % Pitch rate
r = zeros(1,N);   % Yaw rate

%% INITIAL CONDITIONS

x(1) = 0;
y(1) = 0;
z(1) = 1;

%% REFERENCE TRAJECTORY

x_ref = linspace(0,20,N);

y_ref = 5*sin(0.2*t);

z_ref = 5 + 0.5*sin(0.4*t);

%% LQR-STYLE CONTROL MATRICES

Qx = 2.5;
Qy = 2.0;
Qz = 3.0;

Rx = 0.4;
Ry = 0.4;
Rz = 0.5;

%% DERIVED FEEDBACK GAINS

Kp_x = sqrt(Qx/Rx);
Kd_x = 1.8*sqrt(Qx);

Kp_y = sqrt(Qy/Ry);
Kd_y = 1.8*sqrt(Qy);

Kp_z = sqrt(Qz/Rz);
Kd_z = 2.0*sqrt(Qz);

%% DISTURBANCES

windX = 0.2*sin(0.5*t);
windY = 0.2*cos(0.3*t);
%% CONTROL INPUT STORAGE

ux_store = zeros(1,N);
uy_store = zeros(1,N);
uz_store = zeros(1,N);

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
 %% LQR-STYLE CONTROL LAW

ux = Kp_x*ex + Kd_x*evx;
uy = Kp_y*ey + Kd_y*evy;
uz = Kp_z*ez + Kd_z*evz;

%% CONTROL EFFORT PENALTY

ux = ux/(1 + Rx*abs(ux));
uy = uy/(1 + Ry*abs(uy));
uz = uz/(1 + Rz*abs(uz));

%% STORE CONTROL INPUTS

ux_store(i) = ux;
uy_store(i) = uy;
uz_store(i) = uz;

%% APPLY DISTURBANCES

ax = ux + 0.5*windX(i);

ay = uy + 0.15*windY(i);

az = uz;
    
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

%% ATTITUDE DYNAMICS

% DESIRED ANGLES

roll_ref  = -0.04*ay;
pitch_ref =  0.04*ax;

% ANGLE ERRORS

eroll  = roll_ref  - roll(i);
epitch = pitch_ref - pitch(i);

% RATE DYNAMICS

p(i+1) = p(i) + 4*eroll*dt;
q(i+1) = q(i) + 4*epitch*dt;

% DAMPING

p(i+1) = 0.98*p(i+1);
q(i+1) = 0.98*q(i+1);

% ANGLE INTEGRATION

roll(i+1)  = roll(i)  + p(i+1)*dt;
pitch(i+1) = pitch(i) + q(i+1)*dt;

% YAW CONTROL

yaw_ref = atan2(vy(i+1),vx(i+1));

eyaw = yaw_ref - yaw(i);

r(i+1) = r(i) + 2*eyaw*dt;

r(i+1) = 0.99*r(i+1);

yaw(i+1) = yaw(i) + r(i+1)*dt;
    
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

% QUADROTOR SNAPSHOTS

skip = 80;

armLength = 0.4;

for k = 1:skip:N
    
    % BODY AXES
    
    xBody = [ ...
        x(k)-armLength*cos(yaw(k)), ...
        x(k)+armLength*cos(yaw(k))];
    
    yBody = [ ...
        y(k)-armLength*sin(yaw(k)), ...
        y(k)+armLength*sin(yaw(k))];
    
    zBody = [z(k) z(k)];
    
    % CROSS ARM
    
    xCross = [ ...
        x(k)-armLength*cos(yaw(k)+pi/2), ...
        x(k)+armLength*cos(yaw(k)+pi/2)];
    
    yCross = [ ...
        y(k)-armLength*sin(yaw(k)+pi/2), ...
        y(k)+armLength*sin(yaw(k)+pi/2)];
    
    zCross = [z(k) z(k)];
    
    plot3(xBody,yBody,zBody,...
        'r','LineWidth',2);
    
    plot3(xCross,yCross,zCross,...
        'k','LineWidth',2);
    
end
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
%% 3D QUADROTOR ANIMATION

figure(3);

filename = 'quadrotor_animation.gif';

armLength = 0.4;

for k = 1:10:N
    
    clf;
    
    hold on;
    
    % REFERENCE TRAJECTORY
    plot3(x_ref,y_ref,z_ref,'--');
    
    % ACTUAL TRAJECTORY
    plot3(x(1:k),y(1:k),z(1:k),...
        'b','LineWidth',2);
    
    %% QUADROTOR BODY
    
    xBody = [ ...
        x(k)-armLength*cos(yaw(k)), ...
        x(k)+armLength*cos(yaw(k))];
    
    yBody = [ ...
        y(k)-armLength*sin(yaw(k)), ...
        y(k)+armLength*sin(yaw(k))];
    
    zBody = [z(k) z(k)];
    
    xCross = [ ...
        x(k)-armLength*cos(yaw(k)+pi/2), ...
        x(k)+armLength*cos(yaw(k)+pi/2)];
    
    yCross = [ ...
        y(k)-armLength*sin(yaw(k)+pi/2), ...
        y(k)+armLength*sin(yaw(k)+pi/2)];
    
    zCross = [z(k) z(k)];
    
    plot3(xBody,yBody,zBody,...
        'r','LineWidth',3);
    
    plot3(xCross,yCross,zCross,...
        'k','LineWidth',3);
    
    % DRONE CENTER
    
    plot3(x(k),y(k),z(k),...
        'ko','MarkerSize',6,...
        'MarkerFaceColor','k');
    
    %% AXES
    
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    title('3D Quadrotor Animation');
    
    grid on;
    
    axis equal;
    
    xlim([0 22]);
    ylim([-8 8]);
    zlim([0 8]);
    
    view(35,25);
    
    drawnow;
    
    %% GIF EXPORT
    
    frame = getframe(gcf);
    
    im = frame2im(frame);
    
    [A,map] = rgb2ind(im,256);
    
    if k == 1
        
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

%% CONTROL INPUT RESPONSES

figure(4);

subplot(3,1,1);

plot(t(1:N-1),ux_store(1:N-1),...
    'LineWidth',1.5);

grid on;

ylabel('u_x');

title('Optimal Control Inputs');

subplot(3,1,2);

plot(t(1:N-1),uy_store(1:N-1),...
    'LineWidth',1.5);

grid on;

ylabel('u_y');

subplot(3,1,3);

plot(t(1:N-1),uz_store(1:N-1),...
    'LineWidth',1.5);

grid on;

ylabel('u_z');

xlabel('Time (s)');

%% SAVE FIGURE
saveas(gcf,'drone_lqr_control.png');

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
fprintf('3. quadrotor_animation.gif\n');
fprintf('4. drone_lqr_control.png\n');

%% END