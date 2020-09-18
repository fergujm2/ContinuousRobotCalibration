function ReduceTrackerData(testDir, filename, fps, tSpan)

dataFilenameRaw = fullfile('..', testDir, 'DataRaw', [filename, '.csv']);
dataFilenameProcessed = fullfile('..', testDir, 'DataProcessed', filename);

if nargin < 4
    tSpan = [0, 1e6];
end

M = readmatrix(dataFilenameRaw);

qw = M(10:end,6);
qxyz = M(10:end,7:9);
q = [qw, qxyz];
p = M(10:end,10:12);
frame = M(10:end,3);
t = (frame - frame(1))/fps;

% Remove rows outside of the designated timespan
rowsToRemove = or(t < tSpan(1), t > tSpan(2));
rowsToKeep = not(rowsToRemove);

q = q(rowsToKeep,:);
p = p(rowsToKeep,:);
t = t(rowsToKeep,:);

% Convert p from mm to m
p = p./1000;

% Save results 
save(dataFilenameProcessed, 't', 'p', 'q');

% Plot results

figure(1);

subplot(2,1,1);

plot(t, p, '.');

title('Measured Position vs. Time');
xlabel('time (sec)');
ylabel('mm');
legend('p_x', 'p_y', 'p_z');

subplot(2,1,2);

plot(t, q, '.');

title('Measured Quaternion vs. Time');
xlabel('time (sec)');
legend('q_w', 'q_x', 'q_y', 'q_z');

figure(2);
plot3(p(:,1), p(:,2), p(:,3), '.k');

title('Measured Position vs. Time')

axis image;
daspect([1,1,1]);
grid on;

xlabel('x');
ylabel('y');
zlabel('z');

end