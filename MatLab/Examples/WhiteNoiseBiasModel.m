clear;

sigb2 = (1.2e-6)^2; % (rad^2)/(s^4)/Hz
sigw2 = (20.57*(1/3600)*(pi/180))^2; % (rad^2)/(s^2)/Hz
taub = 657.02; % s


T = 1/400;

sigwd2 = 1/T*sigw2; % Assuming v is band-limited to 1/(2*T)
sigbd2 = T*sigb2; % Assuming taub >> T

phid = exp(-1/taub*T);

t = 0:T:1e4;
N = length(t);

w = mvnrnd(0, sigbd2, N)';
v = mvnrnd(0, sigwd2, N)';

x = zeros(size(t));

for iii = 1:(length(t) - 1)
    x(iii + 1) = phid*x(iii) + w(iii);
end

z = x + v;

figure(1);
clf;
hold on;
plot(t, x);
plot(t, z);

legend('Bias', 'Model');