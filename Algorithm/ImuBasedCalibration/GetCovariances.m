function [zCov, qCov, qDotCov, qDDotCov] = GetCovariances()

% Computed using the data in the BNO data sheet
% N0 = 9.81*150e-6; % (m/s/s)/sqrt(Hz)
% B = 62.5; % Hz
% sigmaAlph = sqrt((N0^2)*2*B);
% sigmaOmeg = deg2rad(0.1);

% Guesses
sigmaAlph = 0.5;
sigmaOmeg = 0.05;

alphCov = (sigmaAlph^2).*eye(3);
omegCov = (sigmaOmeg^2).*eye(3);
zCov = blkdiag(alphCov, omegCov);

qCov = (1e-3^2).*eye(6);
qDotCov = (1e-2^2).*eye(6);
qDDotCov = (1e-0^2).*eye(6);

end