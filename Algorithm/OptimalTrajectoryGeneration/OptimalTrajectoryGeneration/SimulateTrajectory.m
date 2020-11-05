function SimulateTrajectory(t, q, filename, fps)

[~, ~, numParamsTotal] = GetRobotCalibInfo();
e = zeros(1,numParamsTotal);

numFrames = (t(end) - t(1))*fps;

tMovie = linspace(t(1), t(end), numFrames);
qMovie = interp1(t, q, tMovie);

F(numFrames) = struct('cdata', [], 'colormap', []);

h = figure(1);
h.Color = [1,1,1];
h.Position = [20, 50, 1100, 700];

for iii = 1:numFrames
    clf;
    
    subplot(1,2,1);
    setupAx();
    view([30, 20]);
    title('Left View', 'Interpreter', 'Latex');

    ComputeForwardKinematics(qMovie(iii,:), e, true);

    subplot(1,2,2);
    setupAx();
    view([150, 20]);
    title('Right View', 'Interpreter', 'Latex');

    ComputeForwardKinematics(qMovie(iii,:), e, true);
    
    drawnow();

    F(iii) = getframe(gcf);
end

v = VideoWriter(filename, 'Motion JPEG AVI');
v.FrameRate = fps;

open(v);

for iii = 1:numFrames
    writeVideo(v, F(iii));
end

close(v);

end

function setupAx()
    hold on;
    grid on;
    axis image;
    xlim([-.3, .65]);
    ylim([-.7, .7]);
    zlim([-.0, .9]);
    set(gca,'XTickLabel',[]);
    set(gca,'YTickLabel',[]);
    set(gca,'ZTickLabel',[]);
end

