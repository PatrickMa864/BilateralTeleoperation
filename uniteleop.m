%%HAPLY 5(6) AND 8
function teleopMasterSlave()
    %% 1) Setup Master and Slave
    clear all; clc; close all;

    % Setup Master (update COM port as needed)
    [~, robotMaster, ~, ~, ~, ~, ~] = controllerSetup('COM15');
    robotMaster.DeviceWakeup();
    
    % Setup Slave (update COM port as needed)
    [~, robotSlave, ~, ~, ~, ~, ~] = controllerSetup('COM9');
    robotSlave.DeviceWakeup();
    
    %% 2) Define Gains, Data Logging, and Timers
    % Gains: 
    %   K       - Proportional gain (keep high for strong force)
    %   B_base  - Base damping (kept low to not sacrifice force)
    %   B_scale - Additional damping based on slave velocity
    %   B_max   - Maximum damping allowed
    %   D       - Derivative gain (helps counteract fast oscillations)
    K = 30;
    B_base = 1;
    B_scale = 5;
    B_max = 20;
    D = 0.5;
    
    % Initialize force request vectors (3x1 zeros)
    requestMaster = zeros(3,1);
    requestSlave  = zeros(3,1);
    
    % Timing parameters
    maxTime = 15;  % seconds
    totalTime = 0;
    loopTimer = tic;
    
    % Pre-allocate data logging arrays (assume up to 7500 samples)
    maxSamples = 7500;
    positionsMaster = zeros(3, maxSamples);
    positionsSlave  = zeros(3, maxSamples);
    timeStamps      = zeros(1, maxSamples);
    
    %% 3) Setup Real-Time 3D Plot
    figure; 
    hold on; grid on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Master (red) & Slave (blue) - Real-Time Positions');
    view(3);
    
    %% 4) Main Control Loop
    count = 0;
    while totalTime < maxTime
        count = count + 1;
        
        % --- A) Read Master Position and Velocity
        [posMaster, velMaster] = robotMaster.EndEffectorForce(requestMaster);
        
        % --- B) Read Slave Position and Velocity
        [posSlave, velSlave] = robotSlave.EndEffectorForce(requestSlave);
        
        % --- C) Compute Velocity-Dependent Damping
        B_dynamic = min(B_base + B_scale * norm(velSlave), B_max);
        
        % --- D) Compute New PD+Derivative Force for the Slave
        % This adds a term that uses the difference between master and slave velocities.
        requestSlave = K * (posMaster - posSlave) - B_dynamic * velSlave - D * (velMaster - velSlave);
        
        % --- E) Log Data
        positionsMaster(:, count) = posMaster;
        positionsSlave(:, count)  = posSlave;
        timeStamps(count)         = totalTime;
        
        % --- F) Live 3D Plot (Master in red, Slave in blue)
        plot3(posMaster(1), posMaster(2), posMaster(3), 'r.');
        plot3(posSlave(1),  posSlave(2),  posSlave(3),  'b.');
        drawnow;
        
        % --- G) Print Debug Information to Console
        fprintf('Time=%.3f | Master=[%.3f, %.3f, %.3f] | Slave=[%.3f, %.3f, %.3f] | B_dynamic=%.3f\n',...
            totalTime, posMaster(1), posMaster(2), posMaster(3),...
            posSlave(1), posSlave(2), posSlave(3), B_dynamic);
        
        % --- H) Wait until 2ms have elapsed (reducing delay)
        while toc(loopTimer) < 0.002
            % Busy wait until 2ms are passed
        end
        dt = toc(loopTimer);
        totalTime = totalTime + dt;
        loopTimer = tic;  % Reset loop timer
        
        % --- I) Stop condition (after maxTime seconds)
        if totalTime >= maxTime
            break;
        end
    end
    
    %% 5) Final Position-vs-Time Plots
    % Trim arrays to the actual number of samples
    positionsMaster = positionsMaster(:, 1:count);
    positionsSlave  = positionsSlave(:, 1:count);
    timeStamps      = timeStamps(1:count);
    
    % Plot Master Position Over Time
    figure;
    subplot(2,1,1);
    plot(timeStamps, positionsMaster(1,:), 'r',...
         timeStamps, positionsMaster(2,:), 'g',...
         timeStamps, positionsMaster(3,:), 'b');
    xlabel('Time (s)'); ylabel('Master Position (m)');
    legend('X_M', 'Y_M', 'Z_M');
    title('Master End-Effector Position Over Time');
    grid on;
    
    % Plot Slave Position Over Time
    subplot(2,1,2);
    plot(timeStamps, positionsSlave(1,:), 'r--',...
         timeStamps, positionsSlave(2,:), 'g--',...
         timeStamps, positionsSlave(3,:), 'b--');
    xlabel('Time (s)'); ylabel('Slave Position (m)');
    legend('X_S', 'Y_S', 'Z_S');
    title('Slave End-Effector Position Over Time');
    grid on;
    
    disp('Done teleoperation after 15 seconds.');
end

%% Controller Setup Function
function [comPort, robot, id, info, modelNumber, hardwareVersion, firmwareVersion] = controllerSetup(portName)
    % Add dependency paths (adjust path as needed)
    addpath('C:\Users\patri\Downloads\SYSC 4206 Proj\dependencies');
    
    comPort = portName;
    robot = HardwareAPI(comPort, true, true);
    id = robot.deviceId;
    info = robot.packageInfo();
    modelNumber = robot.deviceModelNumber;
    hardwareVersion = robot.deviceHardwareVersion;
    firmwareVersion = robot.deviceFirmwareVersion;
end
