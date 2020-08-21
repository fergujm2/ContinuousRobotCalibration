function PlotImuMeasurements(tRobot, q, qTruth, tImu, z, zTruth)

figure(1);
clf;

h = subplot(3,1,1);
hold on;

plot(tRobot,qTruth,'-');
h.ColorOrderIndex = 1;
plot(tRobot,q,'.');
title('Predicted vs. Measured Joint Values');
xlabel('time (s)');
ylabel('Joint Values (rad)');

h = subplot(3,1,2);
hold on;

plot(tImu,zTruth(:,1:3),'-');
h.ColorOrderIndex = 1;
plot(tImu,z(:,1:3),'.');
title('Predicted vs. Measured Accelleration');
xlabel('time (s)');
ylabel('Accelleration (m/s/s)');

h = subplot(3,1,3);
hold on;

plot(tImu,zTruth(:,4:6),'-');
h.ColorOrderIndex = 1;
plot(tImu,z(:,4:6),'.');
title('Predicted vs. Measured Angular Velocities');
xlabel('time (s)');
ylabel('Angular Velocity (rad/s)');

zError = z - zTruth;
qError = q - qTruth;

aError = zError(:,1:3);
wError = zError(:,4:6);

figure(2);
clf;

subplot(3,1,1);
hold on;

for iii = 1:size(qError,2)
    histogram(qError(:,iii));
end

title('Joint Value Noise');
xlabel('Joint Value Errors (rad)');
ylabel('Frequency');

subplot(3,1,2);
hold on;

for iii = 1:size(aError,2)
    histogram(aError(:,iii));
end

title('Accelleration Noise');
xlabel('Accelleration Errors (m/s/s)');
ylabel('Frequency');

subplot(3,1,3);
hold on;

for iii = 1:size(wError,2)
    histogram(wError(:,iii));
end

title('Angular Velocity Noise');
xlabel('Angular Velocity Errors (rad/s)');
ylabel('Frequency');

end