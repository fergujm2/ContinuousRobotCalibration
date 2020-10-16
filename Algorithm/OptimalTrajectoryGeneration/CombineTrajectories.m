function CombineTrajectories(filenames)

M = readmatrix(fullfile('Output', filenames{1}));

for iii = 2:length(filenames)
    M = [M; readmatrix(fullfile('Output', filenames{iii}))];
end

writematrix(M, fullfile('Output', 'Combined'));

plot(M(:,2:end));

end