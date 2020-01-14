%% ME 557: Project, Motor Control

%This script opens a serial port connection with an attached ardunio,
%through which it sends motor position commands.

%Clear Everything
clear, close('all'), clc

%% Setup the Serial Port Connection.

%Open the serial port.
s = OpenSerialPort( 'COM4', 9600 );

%% Read in the Trajectory Values.

Folder = 'C:\Users\USER\Documents\Coursework\MSME\Year1\Winter2018\ME557_IntroToRobotics\Project\Trajectories';
Ltr = 'A';
fName = strcat('MotorPositionTrajectory_', Ltr, '.txt');
mpos = dlmread( strcat(Folder, '\', fName) );

figure, hold on, grid on
plot(1:size(mpos, 2), mpos, '.-', 'Markersize', 20)
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4')

%% Move the Motor to the Target Position.

%Create the points to send to the motor(s).
% ts = linspace(0, 2*pi, 10);
% mps1 = round(1500*sin(ts) + 2000);
% mps2 = round(1500*sin(ts) + 2000);
% mps3 = round(500*sin(ts) + 512);
% mps4 = round(500*sin(ts) + 512);

%Assemble the motor position commands in a matrix.

% mps1 = [0 4000 0 4000];
% mps2 = [0 4000 0 4000];
% mps3 = [0 1023 0 1023];
% mps4 = [0 1023 0 1023];

% mps = [mps1; mps2; mps3; mps4];


mps = [2046; 2046; 512; 512; 512];

% mps = mpos;

%Define the motor IDs of interest.
MotorID = [1; 2; 3; 4; 5];

%Get the motor position.
[cmps, bGotReponse] = GetMotorPosition2( s, MotorID );

%Set the motor positions.
[nmps, bGotResponse] = SetMotorPosition2( s, MotorID, mps);


%% Plot the Desired Motor Positions & the Actual Motor Positions.

%Define the number of commands.
ns = 1:size(mps, 2);

%Create a figure for the desired motor positions and actual motor positions.
figure

%Determine the number of rows and columns to use on the subplot.
[ nrows, ncols ] = GetSubplotRCs( size(mps, 1), false );

%Plot the desired and actual motor positions & build the associated legend.
for k = 1:size(mps, 1)                                      %Iterate through each set of motor positions...
    
    %Format the current subplot.
    subplot(nrows, ncols, k), hold on, grid on
    title('Motor Positions vs Command Number'), ylim([0 1.25*max(mps(k, :))])
    xlabel('Command Number [#]'), ylabel('Motor Position [0-1023] / [0-4096]')
    
    %Plot the desired & actual motor positions.
    h1 = plot(ns, mps(k, :), '.-', 'Markersize', 20);
    h2 = plot(ns, nmps(k, :), '.-', 'Markersize', 20);
    
    %Add a legend to the subplot.
    legend('Write Position', 'Read Position')
    
end



%% Shutdown the Serial Port Connection.

%Close the serial port.
CloseSerialPort( s )
