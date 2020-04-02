function q = GetOptimalPoses(numMeas)
filename = fullfile('Output', 'OptimalPoses', [num2str(numMeas), '.mat']);
obj = load(filename);

q = obj.qStar;
end