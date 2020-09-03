function SimulateTrajectory(t, q, filename)

[~, ~, numParamsTotal] = GetRobotCalibInfo();
e = zeros(1,numParamsTotal);

fps = 3;
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
    hold on;
    grid on;
    axis image;
    xlim([-.2, .6]);
    ylim([-.5, .5]);
    zlim([-.2, 1.1]);
    view([30, 20]);
    set(gca,'XTickLabel',[]);
    set(gca,'YTickLabel',[]);
    set(gca,'ZTickLabel',[]);
    title('Left View', 'Interpreter', 'Latex');

    ComputeForwardKinematics(qMovie(iii,:), e, true);

    subplot(1,2,2);
    hold on;
    grid on;
    axis image;
    xlim([-.2, .6]);
    ylim([-.5, .5]);
    zlim([-.2, 1.1]);
    view([150, 20]);
    set(gca,'XTickLabel',[]);
    set(gca,'YTickLabel',[]);
    set(gca,'ZTickLabel',[]);
    title('Right View', 'Interpreter', 'Latex');

    ComputeForwardKinematics(qMovie(iii,:), e, true);
    drawnow();

    F(iii) = getframe(gcf);
end



v = VideoWriter(filename, 'MPEG-4');
v.FrameRate = fps;

open(v);

for iii = 1:numFrames
    writeVideo(v, F(iii));
end

close(v);

end