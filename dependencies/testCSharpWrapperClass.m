%% Connecting to the device
clear all
clc

comPort = 'COM7';%3
robot = HardwareAPI(comPort, true, true);

id = robot.deviceId
%% Info about C# HardwareAPI assemblies
% The info variable contains information about the C# assemblies that 
% Matlab can now use to communicate with the Inverse3
info = robot.packageInfo();
%% Information about the controlled device
modelNumber = robot.deviceModelNumber
hardwareVersion = robot.deviceHardwareVersion
firmwareVersion = robot.deviceFirmwareVersion
%% Joint Torque Control Example 
% A soft block is simulated on the first joint

% Always call DeviceWakeup() before starting a control loop
robot.DeviceWakeup();

request = zeros(3,1);
totalTimeElapsed = 0;
tic;
timediffs = zeros(1,1000);
count = 0;

k = 400.0; % stifness constant(mN/deg)
wall_angle = -30; % (deg)
while true
    count = count + 1;
    if mod(count, 100) == 0 
       angle = pos_angle(1)
       torque = request(1)
    end
    [pos_angle, ang_vel] = robot.JointTorques(request);
    % the response and request structures are defined to have single precision floats in c#
    % the default precison for floats in matlab is double precision
     
    if  pos_angle(1) < wall_angle
        % If it is, output a fixed upwards force
        request(1) = k*(wall_angle - pos_angle(1));
    else
        % Otherwise, output a zero force 
        request(1) = 0.0;
    end
    
    while toc < 0.001  % 1000 Hz
        % we wait
    end
    
    totalTimeElapsed = totalTimeElapsed + toc;
    
    % timediffs(cc) = toc;
    
    tic;
    
    if (totalTimeElapsed > 30.0)
        break
    end
    
end
%% Force Control Example 
% A soft wall is simulated. the wall is a plan at z = 0.17

% Always call DeviceWakeup() before starting a control loop
robot.DeviceWakeup();

request = zeros(3,1);
totalTimeElapsed = 0;
tic;
timediffs = zeros(1,1000);
count = 0;

k = 500.0; % stifness constant(mN/m)
wall_height = 0.17; % (m)
while true
    count = count + 1;
    if mod(count, 100) == 0 
       z = pos(3);
       up_force = request(3);
    end
    [pos, vel] = robot.EndEffectorForce(request);
    % the response and request structures are defined to have single precision floats in c#
    % the default precison for floats in matlab is double precision
     
    if  pos(3) < wall_height
        % If it is, output a fixed upwards force
        request(3) = k*(wall_height - pos(3));
    else
        % Otherwise, output a zero force 
        request(3) = 0.0;
    end
    
    while toc < 0.01  % 1000 Hz
        % we wait
    end
    
    totalTimeElapsed = totalTimeElapsed + toc;
    
    % timediffs(cc) = toc;
    
    tic;
    
    if (totalTimeElapsed > 30.0)
        break
    end
    
end

%% To stop communication with robot (automatic when calling clear all)
disp('Killing robot!')
robot.delete();

