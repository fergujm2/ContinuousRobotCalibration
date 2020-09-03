function measCov = GetMeasurementCovariance(numMeas)


zCov = GetCovariances();
measCovDiag = repmat(diag(zCov), numMeas, 1);
szMeasCov = 6*numMeas;
measCov = spdiags(measCovDiag, 0, szMeasCov, szMeasCov);


% Convert tall matrix into sparse covariance matrix
% s = measCovWide(:);
% 
% szMeasCov = 6*numMeas;
% i = reshape(ones(6,1).*(1:szMeasCov), [], 1);
% j = reshape(reshape(repmat(1:6, 6, 1)', [], 1) + 6.*(0:numMeas - 1), [], 1);
% 
% measCov = sparse(i,j,s); % i and j took a lot of trial and error to determine
% measCov3 = reshape(s, 6, 6, numMeas);
end


% function [Jq, JqDot, JqDDot] = computeJacobians(q, qDot, qDDot, theta)
% 
%     [calibBools, ~, numParamsTotal] = GetRobotCalibInfo();
%     [x, g] = UnpackTheta(theta);
% 
%     % Robot parameters
%     e = zeros(1,numParamsTotal);
%     e(calibBools) = e(calibBools) + x';
%     
%     numJoints = length(q);
%     
%     Jq = nan(6, numJoints);
%     JqDot = nan(6, numJoints);
%     JqDDot = nan(6, numJoints);
%     
%     del = 1e-6;
%     
%     for iii = 1:numJoints
%         qLo = q;
%         qHi = q;
%         
%         qLo(iii) = q(iii) - del;
%         qHi(iii) = q(iii) + del;
%         
%         zLo = ComputeImuMeasurements(qLo, qDot, qDDot, e, g);
%         zHi = ComputeImuMeasurements(qHi, qDot, qDDot, e, g);
%         
%         Jq(:,iii) = (zHi - zLo)./(2*del);
%     end
%     
%     for iii = 1:numJoints
%         qDotLo = qDot;
%         qDotHi = qDot;
%         
%         qDotLo(iii) = qDot(iii) - del;
%         qDotHi(iii) = qDot(iii) + del;
%         
%         zLo = ComputeImuMeasurements(q, qDotLo, qDDot, e, g);
%         zHi = ComputeImuMeasurements(q, qDotHi, qDDot, e, g);
%         
%         JqDot(:,iii) = (zHi - zLo)./(2*del);
%     end
%     
%     for iii = 1:numJoints
%         qDDotLo = qDDot;
%         qDDotHi = qDDot;
%         
%         qDDotLo(iii) = qDDot(iii) - del;
%         qDDotHi(iii) = qDDot(iii) + del;
%         
%         zLo = ComputeImuMeasurements(q, qDot, qDDotLo, e, g);
%         zHi = ComputeImuMeasurements(q, qDot, qDDotHi, e, g);
%         
%         JqDDot(:,iii) = (zHi - zLo)./(2*del);
%     end
% end
% 
% 
