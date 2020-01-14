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
mrksz = 5;

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
n = 300;

%Define the link lengths of the robot.
[r1, r2, r3, r4] = deal(10, 10, 1, 10);

%Define an array of angles.  Note that thetas1 has half as many points
%because we are restricting the arm to always face the board.  Points where
%the arm does not face the board are not helpful.
[thetas1, thetas2, thetas3] = deal( linspace(0, pi, n/2), linspace(0, 2*pi, n), linspace(0, 2*pi, n) );

%Define the starting point.
p0 = [0; 0; 0];

%Create the translation matrices.  These will depend on the geometry of the
%robot.
T1 = TransMat4(0, 0, r1);
T2 = TransMat4(r2, 0, 0);
T3 = TransMat4(0, r3, 0);
T4 = TransMat4(r4, 0, 0);

%% Determine all Points in the Workspace.

%Preallocate a matrix to store the data points.
[ps1, ps2, ps3, ps4] = deal( zeros(4, length(thetas1)*length(thetas2)*length(thetas3)) );

%Preallocate a matrix to store the angle values.
Mtheta = zeros(3, length(thetas1)*length(thetas2)*length(thetas3));

%Define a counter variable.
k4 = 0;

%Iterate through all of the possible joint angles.
for k1 = 1:length(thetas1)
    
    %Create the first rotation matrix.
    R1 = RotzMat3( thetas1(k1) );
    
    %Increase the Rank of the rotation matrix.
    R1 = [R1 [0; 0; 0]; 0 0 0 1];
    
    for k2 = 1:length(thetas2)
        
        %Create the second rotation matrix.
        R2 = RotxMat3( thetas2(k2) );
        
        %Increase the Rank of the rotation matrix.
        R2 = [R2 [0; 0; 0]; 0 0 0 1];
        
        for k3 = 1:length(thetas3)
            
            %Count the number of iterations.
            k4 = k4 + 1;
            
            %Create the third rotation matrix.
            R3 = RotyMat3( thetas3(k3) );
            
            %Increase the Rank of the rotation matrices.
            R3 = [R3 [0; 0; 0]; 0 0 0 1];
            
            %Translate the first point.
            ps1(:, k4) = R1*T1*[p0; 1];
            
            %Translate the second point.
            ps2(:, k4) = R1*T1*R2*T2*[p0; 1];
            
            %Translate the third point.
            ps3(:, k4) = R1*T1*R2*T2*R3*T3*[p0; 1];
            
            %Translate the fourth point.
            ps4(:, k4) = R1*T1*R2*T2*R3*T3*T4*[p0; 1];
            
            %Record the associated angle value.
            Mtheta(:, k4) = [thetas1(k1); thetas2(k2); thetas3(k3)];
        end
    end
end

%% Plot the Workspaces.

%Setup a figure for the data points.
fig1 = figure;

%Setup the first subplot.
subplot(1, 4, 1)
hold on, grid on
PlotWorkspacePoints( ps1, WBs, mrksz )

%Setup the second subplot.
subplot(1, 4, 2)
hold on, grid on
PlotWorkspacePoints( ps2, WBs, mrksz )

%Setup the third subplot.
subplot(1, 4, 3)
hold on, grid on
PlotWorkspacePoints( ps3, WBs, mrksz )

%Setup the fourth subplot.
subplot(1, 4, 4)
hold on, grid on
PlotWorkspacePoints( ps4, WBs, mrksz )


%% Select the Points that are Near the White Board.

%Define a tolerance.  Points that are farther than this perpendicular to
%the whiteboard will be exclude as not viable.
tol = 0.125;

%Retrieve the points within the tolerance.
locs_crit = abs(ps4(2, :) - d) < tol ;

%Setup a figure to plot the points near the whiteboard.
fig2 = figure;
hold on, grid on
PlotWorkspacePoints( [], WBs, mrksz )

%Plot the points near the white board.
plot3(ps4(1, locs_crit), ps4(2, locs_crit), ps4(3, locs_crit), '.b', 'Markersize', mrksz);

%% Write the nearby points to a file.

%Write the nearby points.
dlmwrite('C:\Users\USER\Documents\Coursework\MSME\Year1\Winter2018\ME557_IntroToRobotics\Project\WorkspaceData\WorkspacePoints_Test.txt', ps4(:, locs_crit));

%Write the associated angles.
dlmwrite('C:\Users\USER\Documents\Coursework\MSME\Year1\Winter2018\ME557_IntroToRobotics\Project\WorkspaceData\WorkspaceAngles_Test.txt', Mtheta(:, locs_crit))

