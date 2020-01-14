%% ME 557: Project, Plan Letter Trajectory

%This script reads in a template letter, the workspace of the robot near the white board, and
%the associated motor angles, then modifies the template letter based on
%the workspace size (ie, center letter, change letter size), next it selects the workspace positions and associated
%motor angles that most closely hit the letter points, and finally writes
%the selected workspace positions and motors angles to .txt files.

%Clear Everything
clear, close('all'), clc

%% Read in the Letter Data.

%Define the Letter of Interest.
Ltr = 'E';

%Read in the target letter points.
Ltr_Folder = 'C:\Users\USER\Documents\Coursework\MSME\Year1\Winter2018\ME557_IntroToRobotics\Project\LetterPoints';
Ltr_FileName = strcat('LetterPts_', Ltr, '.txt');
Ltr_Path = strcat(Ltr_Folder, '\', Ltr_FileName);
Lpts = dlmread(Ltr_Path);

%% Modified the Letter Template.
%We have a letter template, but we need to center it on the board, zero the center of the letter, and set the size of the letter.

%Define the center of the letter.
[xmid, ymid, zmid] = deal( 0, 18, 7 );

%Define the extent by which to scale the letter.
scl = 3;

%Scale the letter points.
Lpts = scl*Lpts;

%Translate the letter points.
[Lpts(1, :), Lpts(2, :), Lpts(3, :)] = deal( Lpts(1, :) + xmid, Lpts(2, :) + 18, Lpts(3, :) + zmid );


%% Solve the Inverse Kinematics Problem -- Determine which angles produce the desired positions.

%Define the link lengths of the robot.
rs = [7 7 3 7 3 7];

%Define the initial angle guess.
theta_guess = [0 0 0 0 0 0]';

%Define the orientation of each joint.
M2 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1); 0 0 0 1];
M3 = [1 0 0 rs(2); 0 1 0 0; 0 0 1 rs(1); 0 0 0 1];
M4 = [1 0 0 rs(2); 0 1 0 0; 0 0 1 (rs(1) - rs(3)); 0 0 0 1];
M5 = [1 0 0 (rs(2) + rs(4)); 0 1 0 0; 0 0 1 (rs(1) - rs(3)); 0 0 0 1];
M6 = [1 0 0 (rs(2) + rs(4)); 0 1 0 0; 0 0 1 (rs(1) - rs(3) + rs(5)); 0 0 0 1];
M7 = [1 0 0 (rs(2) + rs(4) + rs(6)); 0 1 0 0; 0 0 1 (rs(1) - rs(3) + rs(5)); 0 0 0 1];

% %Store the joint orientations in a multidimensional matrix.
[M(:, :, 1), M(:, :, 2), M(:, :, 3), M(:, :, 4), M(:, :, 5), M(:, :, 6)] = deal(M2, M3, M4, M5, M6, M7);

%Define the screw axes.
S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 -rs(1) 0 0]';
S3 = [1 0 0 0 rs(1) 0]';
S4 = [0 1 0 (rs(3) - rs(1)) 0 rs(2)]';
S5 = [1 0 0 0 (rs(1) - rs(3)) 0]';
S6 = [0 1 0 (rs(3) - rs(1) - rs(5)) 0 (rs(2) + rs(4))]';

%Store the screw axes in a matrix.
S = [S1 S2 S3 S4 S5 S6];

%Define the orientational tolerance.
[eomg, ev] = deal( 1e-3, 1e-3 );

%Define the desired orientation.
R = [0 -1 0; 1 0 0; 0 0 1];

T = Rp2TransMatrix( R, Lpts );

%Compute the motor angles & positions.
[ nthetas, mpos ] = Trajectory2MotorAngles( S, M, T, theta_guess, eomg, ev );

%Compute the trajectory of each link point based on the link lengths and
%motor angles.
[ ~, ~, ~, ~, ~, nPts ] = MotorAngles2Trajectory( S, M, nthetas );

%Compute the componentwise error in the end effector position.
errs = nPts - Lpts;

%Compute the magnitude of the error in the end effector position.
errmags = vecnorm(errs);

%Compute angle changes for reference.
dnthetas = diff(nthetas, 1, 2);

%% Plot the Pts and Letter.

%Create a figure to store the workspace points & letter.
figure, hold on, grid on

%Plot a points at the center of the workspace.
plot3(xmid, ymid, zmid, '.g', 'Markersize', 5)

%Plot the modified letter points.
plot3(Lpts(1, :), Lpts(2, :), Lpts(3, :), '-k', 'Linewidth', 3)

%Plot the workspace points that are nearby the letter points.
plot3(nPts(1, :), nPts(2, :), nPts(3, :), '.r', 'Markersize', 20)

%Format the plot.
title('Workspace'), xlabel('x-axis'), ylabel('y-axis'), zlabel('z-axis')
view(30, 30), rotate3d on

%% Compute the associated Motor Positions and plot them.

%Define the points to move through.
ns = 1:size(nthetas, 2);

% %Convert the angles into motor positions.
% mpos = Rad2MotorPos(nthetas);

%Plot the angles versus command number.
figure, subplot(1, 3, 1), hold on, grid on
plot(ns , nthetas(1, :), '.-', 'Markersize', 20);
plot(ns , nthetas(2, :), '.-', 'Markersize', 20);
plot(ns , nthetas(3, :), '.-', 'Markersize', 20);
plot(ns , nthetas(4, :), '.-', 'Markersize', 20);
plot(ns , nthetas(5, :), '.-', 'Markersize', 20);
plot(ns , nthetas(6, :), '.-', 'Markersize', 20);
xlabel('Command Number'), ylabel('Angle [rad]')
title('Angles vs Command Number')
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6')

%Plot the angles versus command number.
subplot(1, 3, 2), hold on, grid on
plot(ns , nthetas(1, :)*180/pi, '.-', 'Markersize', 20);
plot(ns , nthetas(2, :)*180/pi, '.-', 'Markersize', 20);
plot(ns , nthetas(3, :)*180/pi, '.-', 'Markersize', 20);
plot(ns , nthetas(4, :)*180/pi, '.-', 'Markersize', 20);
plot(ns , nthetas(5, :)*180/pi, '.-', 'Markersize', 20);
plot(ns , nthetas(6, :)*180/pi, '.-', 'Markersize', 20);
xlabel('Command Number'), ylabel('Angle [deg]')
title('Angles vs Command Number')
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6')

%Plot the motor position versus command number.
subplot(1, 3, 3), hold on, grid on
plot(ns , mpos(1, :), '.-', 'Markersize', 20);
plot(ns , mpos(2, :), '.-', 'Markersize', 20);
plot(ns , mpos(3, :), '.-', 'Markersize', 20);
plot(ns , mpos(4, :), '.-', 'Markersize', 20);
plot(ns , mpos(5, :), '.-', 'Markersize', 20);
plot(ns , mpos(6, :), '.-', 'Markersize', 20);
xlabel('Command Number'), ylabel('Motor Position [0-4096]')
title('Motor Position vs Command Number')
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6')

%% Plot the Motor Difference Positions.

%Define the points to move through.
dns = 1:size(dnthetas,2);

%Convert the angles into motor positions.
dmpos = Rad2MotorPos(dnthetas);

%Plot the angles versus command number.
figure, subplot(1, 3, 1), hold on, grid on
plot(dns , dnthetas(1, :), '.-', 'Markersize', 20);
plot(dns , dnthetas(2, :), '.-', 'Markersize', 20);
plot(dns , dnthetas(3, :), '.-', 'Markersize', 20);
plot(dns , dnthetas(4, :), '.-', 'Markersize', 20);
plot(dns , dnthetas(5, :), '.-', 'Markersize', 20);
plot(dns , dnthetas(6, :), '.-', 'Markersize', 20);
xlabel('Command Number'), ylabel('Angle [rad]')
title('Difference Angles vs Command Number')
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6')

%Plot the angles versus command number.
subplot(1, 3, 2), hold on, grid on
plot(dns , dnthetas(1, :)*180/pi, '.-', 'Markersize', 20);
plot(dns , dnthetas(2, :)*180/pi, '.-', 'Markersize', 20);
plot(dns , dnthetas(3, :)*180/pi, '.-', 'Markersize', 20);
plot(dns , dnthetas(4, :)*180/pi, '.-', 'Markersize', 20);
plot(dns , dnthetas(5, :)*180/pi, '.-', 'Markersize', 20);
plot(dns , dnthetas(6, :)*180/pi, '.-', 'Markersize', 20);
xlabel('Command Number'), ylabel('Angle [deg]')
title('Difference Angles vs Command Number')
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6')

%Plot the motor position versus command number.
subplot(1, 3, 3), hold on, grid on
plot(dns , dmpos(1, :), '.-', 'Markersize', 20);
plot(dns , dmpos(2, :), '.-', 'Markersize', 20);
plot(dns , dmpos(3, :), '.-', 'Markersize', 20);
plot(dns , dmpos(4, :), '.-', 'Markersize', 20);
plot(dns , dmpos(5, :), '.-', 'Markersize', 20);
plot(dns , dmpos(6, :), '.-', 'Markersize', 20);
xlabel('Command Number'), ylabel('Motor Position [0-4096]')
title('Difference Motor Position vs Command Number')
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6')

%% Plot the End Effector Position Error.

%Create a plot for the end effector position error components.
figure, subplot(1, 2, 1), hold on, grid on
plot(ns, errs(1, :), '.-'), plot(ns, errs(2, :), '.-'), plot(ns, errs(3, :), '.-')
xlabel('Command Number'), ylabel('Position Error [in]')
title('Error Components vs Command Number')
legend('x-error', 'y-error', 'z-error')

%Create a plot for the end effector position error magnitude.
subplot(1, 2, 2), hold on, grid on
plot(ns, errmags, '.-')
xlabel('Command Number'), ylabel('Position Error [in]')
title('Error Magnitude vs Command Number'), legend('Error Mag')

%% Write out the nearest workspace points and their associated angles.

%Define the folder.
RawTrajFolder = 'C:\Users\USER\Documents\Coursework\MSME\Year1\Winter2018\ME557_IntroToRobotics\Project\Trajectories';

%Define the filenames.
fname_MotorAngles = strcat('MotorAnglesTrajectory_', Ltr, '.txt');
fname_MotorPositions = strcat('MotorPositionTrajectory_', Ltr, '.txt');

%Define the paths.
fpath_MotorAngles = strcat(RawTrajFolder, '\', fname_MotorAngles);
fpath_MotorPositions= strcat(RawTrajFolder, '\', fname_MotorPositions);

%Write out the data.
dlmwrite(fpath_MotorAngles, nthetas)        %Motor Angles.
dlmwrite(fpath_MotorPositions, mpos)        %Motor Positions.

