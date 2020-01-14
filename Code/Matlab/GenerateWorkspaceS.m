%% ME 557: Project, Generate Workspace.

%This script takes in robot geometry and plots the workspace of the robot
%and the target whiteboard.  Workspace points near the whiteboard are
%stored in a .txt file.  The associated motor angles are also stored in a
%.txt file.

%Clear Everything
clear
close all
clc

%% Setup Plot Properties.

%Define the markersize for many plotting elements.
mrksz = 1;

%% Define the white board points.

%Define the distance to the white board.
d = 18;

%Define the width of the White Board.
r = 50;

%Parameterize a plane.
xs = linspace(-r, r, 100);
zs = linspace(-r, r, 100);

%Convert to matrices.
[Xs, Zs] = meshgrid(xs, zs);

%The Y values of the whiteboard should be constant, since we want the
%whiteboard to be parallel to the zx-plane.
Ys = d*ones(length(xs), length(zs));

%Create a 3D matrix to store the white board points.
[WBs(:, :, 1), WBs(:, :, 2), WBs(:, :, 3)] = deal( Xs, Ys, Zs);

%% Setup the Robot Geometry.

%Define the number of positions for each motor.  Note that each motor can
%achieve 1023 positions, but setting this number anywhere near that upper
%bound creates memory problems.
n = 10;

%Define the link lengths of the robot.
rs = [10 7 3 7 3 7];

%Define an array of angles.  Note that thetas1 has half as many points
%because we are restricting the arm to always face the board.  Points where
%the arm does not face the board are not helpful.
thetas1 = linspace(0, pi, n/2);
[thetas2, thetas3, thetas4, thetas5, thetas6] = deal( linspace(0, 2*pi, n) );

%Define the starting point.
p0 = [0; 0; 0];

%Define the orientation of each joint.
M2 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1); 0 0 0 1];
M3 = [1 0 0 rs(2); 0 1 0 0; 0 0 1 rs(1); 0 0 0 1];
M4 = [1 0 0 rs(2); 0 1 0 0; 0 0 1 (rs(1) - rs(3)); 0 0 0 1];
M5 = [1 0 0 (rs(2) + rs(4)); 0 1 0 0; 0 0 1 (rs(1) - rs(3)); 0 0 0 1];
M6 = [1 0 0 (rs(2) + rs(4)); 0 1 0 0; 0 0 1 (rs(1) - rs(3) + rs(5)); 0 0 0 1];
M7 = [1 0 0 (rs(2) + rs(4) + rs(6)); 0 1 0 0; 0 0 1 (rs(1) - rs(3) + rs(5)); 0 0 0 1];

% %Store the joint orientations in a multidimensional matrix.
% [M(:, :, 1), M(:, :, 2), M(:, :, 3), M(:, :, 4), M(:, :, 5), M(:, :, 6)] = deal(M2, M3, M4, M5, M6, M7);

%Define the screw axes.
S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 -rs(1) 0 0]';
S3 = [1 0 0 0 rs(1) 0]';
S4 = [0 1 0 (rs(3) - rs(1)) 0 rs(2)]';
S5 = [1 0 0 0 (rs(1) - rs(3)) 0]';
S6 = [0 1 0 (rs(3) - rs(1) - rs(5)) 0 (rs(2) + rs(4))]';

%Store the screw axes in a matrix.
S = [S1 S2 S3 S4 S5 S6];

%% Determine all Points in the Workspace.

%Preallocate a matrix to store the data points.
[ps1, ps2, ps3, ps4, ps5, ps6, ps7] = deal( zeros(3, length(thetas1)*length(thetas2)*length(thetas3)) );

%Preallocate a matrix to store the angle values.
Mtheta = zeros(6, length(thetas1)*length(thetas2)*length(thetas3));

%Define a counter variable.
cnt = 0;

%Iterate through all of the possible joint angles.
for k1 = 1:length(thetas1)
    for k2 = 1:length(thetas2)
        for k3 = 1:length(thetas3)
            for k4 = 1:length(thetas4)
                for k5 = 1:length(thetas5)
                    for k6 = 1:length(thetas6)
                        %Count the number of iterations.
                        cnt = cnt + 1;
                        
                        %Define the current theta vector.
                        thetavec = [thetas1(k1); thetas2(k2); thetas3(k3); thetas4(k4); thetas5(k5); thetas6(k6)];
                        
                        %Compute the orientation of the joints.
                        T2 = FKinSpace(M2, S(:, 1:2), thetavec(1:2));
                        T3 = FKinSpace(M3, S(:, 1:3), thetavec(1:3));
                        T4 = FKinSpace(M4, S(:, 1:4), thetavec(1:4));
                        T5 = FKinSpace(M5, S(:, 1:5), thetavec(1:5));
                        T6 = FKinSpace(M6, S, thetavec);
                        T7 = FKinSpace(M7, S, thetavec);
                        
                        %Retrieve the position of each joint.
                        ps2(:, cnt) = T2(1:3, 4);
                        ps3(:, cnt) = T3(1:3, 4);
                        ps4(:, cnt) = T4(1:3, 4);
                        ps5(:, cnt) = T5(1:3, 4);
                        ps6(:, cnt) = T6(1:3, 4);
                        ps7(:, cnt) = T7(1:3, 4);
                        
                        %Record the associated angle value.
                        Mtheta(:, cnt) = thetavec;
                    end
                end
            end
        end
    end
end

%% Plot the Workspaces.

%Setup a figure for the data points.
fig1 = figure;

%Setup the first subplot.
subplot(2, 4, 1)
hold on, grid on
PlotWorkspacePoints( ps1, WBs, mrksz )

%Setup the second subplot.
subplot(2, 4, 2)
hold on, grid on
PlotWorkspacePoints( ps2, WBs, mrksz )

%Setup the third subplot.
subplot(2, 4, 3)
hold on, grid on
PlotWorkspacePoints( ps3, WBs, mrksz )

%Setup the fourth subplot.
subplot(2, 4, 4)
hold on, grid on
PlotWorkspacePoints( ps4, WBs, mrksz )

%Setup the fifth subplot.
subplot(2, 4, 5)
hold on, grid on
PlotWorkspacePoints( ps5, WBs, mrksz )

%Setup the sixth subplot.
subplot(2, 4, 6)
hold on, grid on
PlotWorkspacePoints( ps6, WBs, mrksz )

%Setup the seventh subplot.
subplot(2, 4, 7)
hold on, grid on
PlotWorkspacePoints( ps7, WBs, mrksz )


%% Select the Points that are Near the White Board.

%Define a tolerance.  Points that are farther than this perpendicular to
%the whiteboard will be exclude as not viable.
tol = 0.125;

%Retrieve the points within the tolerance.
locs_crit = abs(ps7(2, :) - d) < tol ;

%Setup a figure to plot the points near the whiteboard.
fig2 = figure;
hold on, grid on
PlotWorkspacePoints( [], WBs, mrksz )

%Plot the points near the white board.
plot3(ps7(1, locs_crit), ps7(2, locs_crit), ps7(3, locs_crit), '.b', 'Markersize', mrksz);

%% Write the nearby points to a file.

%Write the nearby points.
dlmwrite('C:\Users\USER\Documents\Coursework\MSME\Year1\Winter2018\ME557_IntroToRobotics\Project\WorkspaceData\WorkspacePoints_Test.txt', ps7(:, locs_crit));

%Write the associated angles.
dlmwrite('C:\Users\USER\Documents\Coursework\MSME\Year1\Winter2018\ME557_IntroToRobotics\Project\WorkspaceData\WorkspaceAngles_Test.txt', Mtheta(:, locs_crit))

