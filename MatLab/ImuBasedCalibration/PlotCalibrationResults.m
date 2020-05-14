function PlotCalibrationResults(thetaTruth, thetaStar, thetaStarCov)

% Now, its a normal distribution, so the marginal variances are just the
% diagonal elements of the covariance matrix
thetaStarStd = sqrt(diag(thetaStarCov));

[xStar, gStar, tauStar, sStar] = UnpackTheta(thetaStar);
[xTruth, gTruth, tauTruth, sTruth] = UnpackTheta(thetaTruth);
[xStarStd, gStarStd, tauStarStd, sStarStd] = UnpackTheta(thetaStarStd);

[calibBools, ~, ~, paramsMeters] = GetRobotCalibInfo();

xMeters = paramsMeters(calibBools);
xRadians = not(xMeters);

xErrors = abs(xStar - xTruth);
xErrorsMeters = xErrors(xMeters);
xErrorsRadians = xErrors(xRadians);

xErrorsMetersMax = max(xErrorsMeters);
xErrorsRadiansMax = max(xErrorsRadians);

fprintf('\n\nMaximum Estimation Errors: \n  Distance: %.6f mm \n     Angle: %.6f deg \n\n', xErrorsMetersMax*1000, xErrorsRadiansMax*180/pi);

figure(1);
clf;

subplot(5,1,1);
hold on;
y = [xStar(1:(end - 6)), xTruth(1:(end - 6))];
bar(y);

ngroups = size(y, 1); nbars = size(y, 2);
groupwidth = min(0.8, nbars/(nbars + 1.5));
x = (1:ngroups) - groupwidth/2 + (2*1-1) * groupwidth / (2*nbars);
errorbar(x, y(:,1), xStarStd(1:(end - 6)), '.k');

title('Robot Parameters');

subplot(5,1,2);
hold on;
y = [xStar((end - 5):end), xTruth((end - 5):end)];
bar(y);

ngroups = size(y, 1); nbars = size(y, 2);
groupwidth = min(0.8, nbars/(nbars + 1.5));
x = (1:ngroups) - groupwidth/2 + (2*1-1) * groupwidth / (2*nbars);
errorbar(x, y(:,1), xStarStd((end - 5):end), '.k');

title('IMU Offset Parameters');

subplot(5,1,3);
hold on;
y = [gStar, gTruth];
bar(y);

ngroups = size(y, 1); nbars = size(y, 2);
groupwidth = min(0.8, nbars/(nbars + 1.5));
x = (1:ngroups) - groupwidth/2 + (2*1-1) * groupwidth / (2*nbars);
errorbar(x, y(:,1), gStarStd, '.k');

legend('Estimated', 'Truth');
title('Gravity Components');

subplot(5,1,4);
hold on;
y = [tauStar, tauTruth];
bar(y, 'FaceColor', 'flat', 'CData', [0    0.4470    0.7410; 0.8500    0.3250    0.0980]);

ngroups = size(y, 1); nbars = size(y, 2);
groupwidth = min(0.8, nbars/(nbars + 1.5));
x = (1:ngroups) - groupwidth/2 + (2*1-1) * groupwidth / (2*nbars);
errorbar(x, y(:,1), tauStarStd, '.k');

title('Time Offset');

subplot(5,1,5);
hold on;
y = [sStar, sTruth];
bar(y);

ngroups = size(y, 1); nbars = size(y, 2);
groupwidth = min(0.8, nbars/(nbars + 1.5));
x = (1:ngroups) - groupwidth/2 + (2*1-1) * groupwidth / (2*nbars);
errorbar(x, y(:,1), sStarStd, '.k');

title('Data Scale Factors');

drawnow();
end