clear;

% Config variables
usingRobot = true;
maxTime = 1270;

bufferLength = 1e6;
imuMsgData = nan(bufferLength, 1 + 3 + 3);
robotMsgData = nan(bufferLength, 1 + 6 + 6);
t = nan(bufferLength,1);

% Setup ROS to get the AUBO robot data
if usingRobot
    rosinit('192.168.0.1');
    rosSub = rossubscriber('/robot1_jointLog');
end

% Setup arduino serial communication to read IMU data
arduino = serial('COM7', 'BaudRate', 115200);
fopen(arduino);

numMsgs = 1;
tic;

while true
    % Break after maxTime seconds
    currentTime = toc();
    if currentTime > maxTime
        break;
    end
    
    % Receive and process arduino message
    msg = fscanf(arduino);
    
    splitMsg = strsplit(msg);
    msgDouble = zeros(1,7);
    
    % Attempt to convert message values to floats
    try
        for jjj = 1:length(msgDouble)
            msgDouble(jjj) = str2double(splitMsg(jjj));
        end
    catch
        warning('Could not convert arduino message to double.');
        continue;
    end
    
    if usingRobot
        msg = receive(rosSub, 3);
        data = msg.Data;
    
        timestamp = data(1);
        qDes = data(2:7)';
        q = data(8:13)';

        robotMsgData(numMsgs,:) = [timestamp, qDes, q];
    end
    
    timestamp = msgDouble(1);
    a = msgDouble(2:4);
    w = msgDouble(5:7);
    
    imuMsgData(numMsgs,:) = [timestamp, a, w];
    t(numMsgs) = currentTime;
    
    numMsgs = numMsgs + 1;
end

% Cleanup connections
fclose(arduino);
rosshutdown();

% Trim data for values that aren't nans or zeros
rowsToKeep = not(any(isnan(imuMsgData),2));
rowsToKeep = and(rowsToKeep, not(all(imuMsgData(:,3:end) == 0, 2)));

if usingRobot
    rowsToKeep = and(rowsToKeep, not(any(isnan(robotMsgData),2)));
end

imuMsgData = imuMsgData(rowsToKeep,:);
robotMsgData = robotMsgData(rowsToKeep,:);
t = t(rowsToKeep);

% Save the collected data
filename = ['ImuCalibrationData_', datestr(now, 'yyyymmdd_HHMMSS'), '.mat'];
save(filename);

% Plot IMU data
figure(1);
clf;
subplot(2,1,1);
plot(imuMsgData(:,1), imuMsgData(:,2:4));
title('Linear Acceleration');
subplot(2,1,2);
plot(imuMsgData(:,1), imuMsgData(:,5:7));
title('Angular Velocity');


% Plot Robot Data
figure(2);
clf;
hold on;
plot(robotMsgData(:,1), robotMsgData(:,2:7), '.');
plot(robotMsgData(:,1), robotMsgData(:,8:13), '-');