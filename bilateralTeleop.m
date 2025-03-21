function bilateralTeleop()
    %% 1) Setup Master and Slave
    clear all; clc; close all;

    % Setup Master (update COM port as needed)
    [~, robotMaster, ~, ~, ~, ~, ~] = controllerSetup('COM15');
    robotMaster.DeviceWakeup();
    
    % Setup Slave (update COM port as needed)
    [~, robotSlave, ~, ~, ~, ~, ~] = controllerSetup('COM9');
    robotSlave.DeviceWakeup();
    
    %% 2) Define Gains, Data Logging, and Timing
    % Gains for bilateral teleoperation:
    % The master receives a force feedback proportional to the position error,
    % while the slave is driven to follow the master.
    K = 10;    % Position (spring) gain
    B = 1.5;     % Damping gain

    % Initialize force request vectors for both master and slave
    requestMaster = zeros(3,1);  
    requestSlave  = zeros(3,1);
    
    maxTime = 15;    % Run teleoperation for 15 seconds
    totalTime = 0;
    loopTimer = tic;
    
    % Pre-allocate data logging arrays (assume up to 7500 samples at ~5ms loop)
    maxSamples = 7500;
    positionsMaster = zeros(3, maxSamples);
    positionsSlave  = zeros(3, maxSamples);
    timeStamps      = zeros(1, maxSamples);
    
    %% 3) Setup Real-Time 3D Plot for Visualization
    figure; 
    hold on; grid on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Bilateral Teleoperation: Master (red) & Slave (blue)');
    view(3);
    
    %% 4) Main Bilateral Control Loop
    count = 0;
    while totalTime < maxTime
        count = count + 1;
        
        % --- A) Read Master and Slave End-Effector States
        [posMaster, velMaster] = robotMaster.EndEffectorForce(requestMaster);
        [posSlave,  velSlave]  = robotSlave.EndEffectorForce(requestSlave);
        
        % --- B) Compute Force Commands
        % Slave should follow the master's position:
        requestSlave = K * (posMaster - posSlave) - B * velSlave;
        % Master receives force feedback from the slave:
        requestMaster = -K * (posMaster - posSlave) - B * velMaster;
        
        % --- C) Log Data for Post-Analysis
        positionsMaster(:, count) = posMaster;
        positionsSlave(:, count)  = posSlave;
        timeStamps(count)         = totalTime;
        
        % --- D) Update Live 3D Plot (Master in red, Slave in blue)
        plot3(posMaster(1), posMaster(2), posMaster(3), 'r.');
        plot3(posSlave(1),  posSlave(2),  posSlave(3),  'b.');
        drawnow;
        
        % --- E) Print Debug Information to Console
        fprintf('Time=%.3f s | Master=[%.3f, %.3f, %.3f] | Slave=[%.3f, %.3f, %.3f]\n',...
            totalTime, posMaster(1), posMaster(2), posMaster(3),...
                       posSlave(1), posSlave(2), posSlave(3));
        
        % --- F) Wait ~5ms (adjust loop rate as needed)
        while toc(loopTimer) < 0.005
            % busy wait until 5ms have elapsed
        end
        dt = toc(loopTimer);
        totalTime = totalTime + dt;
        loopTimer = tic;  % reset loop timer
        
        % --- G) Check Stop Condition (after maxTime seconds)
        if totalTime >= maxTime
            break;
        end
    end
    
    %% 5) Post-Process: Final Plots
    % Trim arrays to the actual number of samples
    positionsMaster = positionsMaster(:, 1:count);
    positionsSlave  = positionsSlave(:, 1:count);
    timeStamps      = timeStamps(1:count);
    
    % Plot Master Position Over Time
    figure;
    subplot(2,1,1);
    plot(timeStamps, positionsMaster(1,:), 'r', ...
         timeStamps, positionsMaster(2,:), 'g', ...
         timeStamps, positionsMaster(3,:), 'b');
    xlabel('Time (s)'); ylabel('Master Position (m)');
    legend('X_M','Y_M','Z_M','Location','best');
    title('Master End-Effector Position Over Time');
    grid on;
    
    % Plot Slave Position Over Time
    subplot(2,1,2);
    plot(timeStamps, positionsSlave(1,:), 'r', ...
         timeStamps, positionsSlave(2,:), 'g', ...
         timeStamps, positionsSlave(3,:), 'b');
    xlabel('Time (s)'); ylabel('Slave Position (m)');
    legend('X_S','Y_S','Z_S','Location','best');
    title('Slave End-Effector Position Over Time');
    grid on;
    
    disp('Bilateral teleoperation complete.');
end

%% Controller Setup Function
function [comPort, robot, id, info, modelNumber, hardwareVersion, firmwareVersion] = controllerSetup(portName)
    % Add dependency paths (adjust as needed for your system)
    addpath('C:\Users\patri\Downloads\SYSC 4206 Proj\dependencies');
    
    comPort = portName;
    robot = HardwareAPI(comPort, true, true);
    id = robot.deviceId;
    info = robot.packageInfo();
    modelNumber = robot.deviceModelNumber;
    hardwareVersion = robot.deviceHardwareVersion;
    firmwareVersion = robot.deviceFirmwareVersion;
end
