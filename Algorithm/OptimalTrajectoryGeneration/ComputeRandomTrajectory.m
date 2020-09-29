function ComputeRandomTrajectory()

T = 120;
sampleRate = 120;
knotsPerSecond = 1;
d = 3;

tSpan = [0, T];
k = length(tSpan(1):(1/knotsPerSecond):tSpan(2)) - 2;
y = MakeExtendedKnots(tSpan(1), tSpan(end), k, d);
numZeroPts = 3;

jointLimits = GetJointLimits();
jointMeans = mean(jointLimits,2);
jointLengths = jointLimits(:,2) - jointLimits(:,1);

n = d + k + 1;
C = repmat(jointMeans, 1, n);

for iii = (numZeroPts + 1):(n - numZeroPts)
    
    fprintf('Computing %.0f of %.0f spline coefficients.\n', iii, n - numZeroPts);
    while true
        Ci = 0.3.*jointLengths.*(rand(size(C,1),1) - 0.5) + jointMeans;
        C(:,iii) = Ci;
        
        [yd, Cd, dd] = DerVectorSpline(y, C, d);
        [ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);
    
        q = @(t) EvalVectorSpline(y, C, d, t);
        qDot = @(t) EvalVectorSpline(yd, Cd, dd, t);
        qDDot = @(t) EvalVectorSpline(ydd, Cdd, ddd, t);
        
        withinLimits = CheckJointLimits(q, qDot, qDDot, tSpan);
        
        if withinLimits
            break;
        end
    end
end

[qLimits, qDotLimits, qDDotLimits] = GetJointLimits();

filename = sprintf('BSplineRandom_d%.0f_%.0fs.mat', d, T);
fullFilename = fullfile('Output', filename);

save(fullFilename, 'y', 'C', 'd', 'k', 'n', 'knotsPerSecond', 'tSpan', 'sampleRate', 'qLimits', 'qDotLimits', 'qDDotLimits');

end