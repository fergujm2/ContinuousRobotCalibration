clear;

%% Test regular fitting of vector spline function

a = 1.4;
b = 7.3;
d = 5;

xData = rand(3, 7);
tData = linspace(a, b, 7);

[y, C] = FitVectorSpline(xData', a, b, d);

t = linspace(a, b, 100);
x = EvalVectorSpline(y, C, d, t);

figure(1);
clf;
hold on;
plot(tData, xData, 'o');
plot(t, x, '-');

%% Test least squares fitting

xData = x + 0.01*randn(size(x));

[y, C] = LsqFitVectorSpline(xData, a, b, d, 20);
x = EvalVectorSpline(y, C, d, t);

figure(2);
clf;
hold on;
plot(t, xData, '.');
plot(t, x, '-');

%% Test derivatives of vector spline

[yd, Cd, dd] = DerVectorSpline(y, C, d);
[ydd, Cdd, ddd] = DerVectorSpline(yd, Cd, dd);

xd = EvalVectorSpline(yd, Cd, dd, t);
xdd = EvalVectorSpline(ydd, Cdd, ddd, t);

xdFd = diff(x, 1, 1)./mean(diff(t,1));
xddFd = diff(xdFd, 1, 1)./mean(diff(t,1));

figure(3);
clf;

subplot(3,1,1);
hold on;

plot(t(1:(end - 1)), xd(1:(end - 1),1) - xdFd(:,1), '-r', 'Linewidth', 1);
plot(t(1:(end - 2)), xdd(1:(end - 2),1) - xddFd(:,1), '-g', 'Linewidth', 1);

ylabel('Component 1');
ylim([-.001, .001]);
legend('xd Error', 'xdd Error');
title('Finite Difference Error');

subplot(3,1,2);
hold on;

plot(t(1:(end - 1)), xd(1:(end - 1),2) - xdFd(:,2), '-r', 'Linewidth', 1);
plot(t(1:(end - 2)), xdd(1:(end - 2),2) - xddFd(:,2), '-g', 'Linewidth', 1);

ylabel('Component 2');
ylim([-.001, .001]);
legend('xd Error', 'xdd Error');

subplot(3,1,3);
hold on;

plot(t(1:(end - 1)), xd(1:(end - 1),3) - xdFd(:,3), '-r', 'Linewidth', 1);
plot(t(1:(end - 2)), xdd(1:(end - 2),3) - xddFd(:,3), '-g', 'Linewidth', 1);

ylabel('Component 3');
ylim([-.001, .001]);
legend('xd Error', 'xdd Error');

%% Test integrals of vector spline
[yInt, CInt, dInt] = IntVectorSpline(y, C, d);

xInt = EvalVectorSpline(yInt, CInt, dInt, t);
xIntFd = cumtrapz(t, x);


figure(4);
clf;

subplot(3,1,1);
hold on;

plot(t, xInt(:,1) - xIntFd(:,1), '-r');
ylabel('Compenent 1');

subplot(3,1,2);
hold on;

plot(t, xInt(:,2) - xIntFd(:,2), '-r');
ylabel('Compenent 2');

subplot(3,1,3);
hold on;

plot(t, xInt(:,3) - xIntFd(:,3), '-r');
ylabel('Compenent 3');
