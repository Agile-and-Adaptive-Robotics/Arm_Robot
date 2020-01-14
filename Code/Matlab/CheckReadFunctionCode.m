%% Check Read Function Code

%This script was used briefly to assess the viability of using the motor read command.

%Clear Everything
clear
close all
clc

%% Define the Geometry of the Robot.

% %VERTICAL HOME POSITION DESIGN - ALTERNATING MOTOR AXES, COUPLED
% %Define the link lengths of the robot.
% rs = [1 + 7/16, 4 + 1/4, 2 + 3/8, 3 + 1/4, 2 + 1/8, 3 + 1/4, 3];          %With shorter bracket holding the fifth motor.
rs = [1 + 7/16, 4 + 1/4, 2 + 3/8, 3 + 1/4, 3 + 1/8, 3 + 1/4, 3];            %With longer bracket holding the fifth motor.

%Define the home orientation of each joint.  To compute the appropriate joint angles, we only need the home orientation of the end effector.  However, in order to plot the movement of all of the joints, we need all home orientations.
M1 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1); 0 0 0 1];
M2 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1) + rs(2); 0 0 0 1];
M3 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1) + rs(2) + rs(3); 0 0 0 1];
M4 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1) + rs(2) + rs(3) + rs(4); 0 0 0 1];
M5 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1) + rs(2) + rs(3) + rs(4) + rs(5); 0 0 0 1];
M6 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1) + rs(2) + rs(3) + rs(4) + rs(5) + rs(6); 0 0 0 1];
M7 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1) + rs(2) + rs(3) + rs(4) + rs(5) + rs(6) + rs(7); 0 0 0 1];

% %Store the joint orientations in a multidimensional matrix.  Each layer corresponds to one of the joint home positions.
M = cat(3, M1, M2, M3, M4, M5, M6, M7);

%Define the screw axes.
S1 = [1 0 0 0 rs(1) 0]';
S2 = [1 0 0 0 rs(1) + rs(2) 0]';
S3 = [0 1 0 -(rs(1) + rs(2) + rs(3)) 0 0]';
S4 = [0 1 0 -(rs(1) + rs(2) + rs(3) + rs(4)) 0 0]';
S5 = [1 0 0 0 (rs(1) + rs(2) + rs(3) + rs(4) + rs(5)) 0]';
S6 = [1 0 0 0 (rs(1) + rs(2) + rs(3) + rs(4) + rs(5) + rs(6)) 0]';

%Store the screw axes in a matrix.  Each column of S is a screw axis for a different joint.
S = [S1 S2 S3 S4 S5];

%% Read in the theta inputs.

thetas_sent = dlmread('C:\Users\USER\Documents\Coursework\MSME\Year1\Winter2018\ME557_IntroToRobotics\Project\SentData.txt');
thetas_read = dlmread('C:\Users\USER\Documents\Coursework\MSME\Year1\Winter2018\ME557_IntroToRobotics\Project\ReadData.txt');
thetas_read(:, 1) = [];

ns = 1:size(thetas_sent, 2);

figure, hold on, grid on
plot(ns, thetas_sent, '.-', 'Markersize', 10)
title('Sent Angles vs Command Number'), xlabel('Command Number [#]'), ylabel('Send Angle [rad]')

figure, hold on, grid on
plot(ns, thetas_read, '.-', 'Markersize', 10)
title('Read Angles vs Command Number'), xlabel('Command Number [#]'), ylabel('Read Angle [rad]')

figure

[rs, cs] = GetSubplotRCs(size(thetas_sent, 1));

for k = 1:size(thetas_sent, 1)
    subplot(rs, cs, k), hold on, grid on
    title('Sent / Read Angles vs Command Number'), xlabel('Command Number [#]'), ylabel('Read Angle [rad]')
    plot(ns, (180/pi)*thetas_sent(k, :), '.-', 'Markersize', 10)
    plot(ns, (180/pi)*thetas_read(k, :), '.-', 'Markersize', 10)
    legend('Send Values', 'Read Values')
end

%% Read in the theta outputs.

thetas_result1 = dlmread('C:\Users\USER\Documents\Coursework\MSME\Year1\Winter2018\ME557_IntroToRobotics\Project\thetas_result1.txt');
thetas_result2 = dlmread('C:\Users\USER\Documents\Coursework\MSME\Year1\Winter2018\ME557_IntroToRobotics\Project\thetas_result2.txt');

ns = 1:size(thetas_result1, 2);

figure, hold on, grid on
plot(ns, thetas_result1, '.-', 'Markersize', 10)
title('Sent Angles vs Command Number'), xlabel('Command Number [#]'), ylabel('Send Angle [rad]')

figure, hold on, grid on
plot(ns, thetas_result2, '.-', 'Markersize', 10)
title('Read Angles vs Command Number'), xlabel('Command Number [#]'), ylabel('Read Angle [rad]')

figure

[rs, cs] = GetSubplotRCs(size(thetas_result1, 1));

for k = 1:size(thetas_result1, 1)
    subplot(rs, cs, k), hold on, grid on
    title('Sent / Read Angles vs Command Number'), xlabel('Command Number [#]'), ylabel('Read Angle [rad]')
    plot(ns, (180/pi)*thetas_result1(k, :), '.-', 'Markersize', 10)
    plot(ns, (180/pi)*thetas_result2(k, :), '.-', 'Markersize', 10)
    legend('Send Values', 'Read Values')
end

%% Solve the Forward Kinematics Problem.

ps1 = MotorAngles2Trajectory( S, M, thetas_result1 );
ps2 = MotorAngles2Trajectory( S, M, thetas_result2 );

figure, hold on, grid on, view(30, 30), rotate3d on, axis equal
for k = 1:size(ps1, 3)
    plot3(ps1(1, :, k), ps1(2, :, k), ps1(3, :, k), '.-', 'Markersize', 20)
    plot3(ps2(1, :, k), ps2(2, :, k), ps2(3, :, k), '.-', 'Markersize', 20)
end



