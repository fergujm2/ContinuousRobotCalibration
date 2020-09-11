clear;

%% Evaluating the B-Form

a = 0;
b = 10;

d = 5;
k = 9;

numRep = d + 1;

y0 = a*ones(1, numRep);
yf = b*ones(1, numRep);

yInt = linspace(a, b, k + 2);
yInt = yInt(2:(end - 1));

y = [y0, yInt, yf];

n = d + k + 1;
c = zeros(1, n);

c(7) = 1;

t = a:0.001:b;
x = sval2(d, y, c, t);

plot(t, x);

%% Cubic Hermite

t = 1:10;
x = [1, -2, -3, 4, 5, 6, -9, -8, 9, 10];
xDot = (1:10)/10;

[y, c] = cubherm(t, x, xDot);

t = 1:0.001:b;
x = sval2(3, y, c, t);

plot(t, x);

%% Shape Preserving Cubic Hermite

t = 1:10;
x = rand(size(t));

[y, c] = cubshape(t, x);

t = 1:0.001:b;
x = sval2(3, y, c, t);

plot(t, x);

%% Not a knot

t = 1:10;
x = [1, -2, 3, 4, 2, 4, 5, 4, 2, 5];

[y, c] = notaknot(5, t, x);

t = 1:0.001:b;
x = sval2(3, y, c, t);

plot(t, x);

%% Probably Decided on Not a Knot 

a = 1;
b = 20;

t = a:b;

x = zeros(size(t));

x1 = x;
x1(1:5) = 1:5;
x2 = x1;
x2(6:7) = [2, 1];

[y, c1] = notaknot(3, t, x1);
[y, c2] = notaknot(3, t, x2);

ts = a:0.001:b;
x1s = sval2(3, y, c1, ts);
x2s = sval2(3, y, c2, ts);

figure(1);
clf;
hold on;

plot(t, x2);
plot(ts, x1s);
plot(ts, x2s);

grid on;

%% Actually, probably should use B-Form because changing interpolant values has non-local effects on spline

a = 0;
b = 20;

d = 3;
k = 19;

numRep = d + 1;
y0 = a*ones(1, numRep);
yf = b*ones(1, numRep);
yInt = linspace(a, b, k + 2);
yInt = yInt(2:(end - 1));
y = [y0, yInt, yf];
n = d + k + 1;

pointsPerStep = 5;

c1 = zeros(1, n);
c1(1:pointsPerStep) = 1:pointsPerStep;
c2 = c1;
c2((pointsPerStep + 1):(2*pointsPerStep)) = -fliplr(1:pointsPerStep);
c3 = c2;
c3((2*pointsPerStep + 1):(3*pointsPerStep)) = 1:pointsPerStep;

% degree = m - 1
m = d + 1;
cInd = (pointsPerStep + 1):(2*pointsPerStep);

yEffectBeg = y(cInd(1));
yEffectEnd = y(cInd(end) + 1);
yEffectEndJointLim = y(cInd(end) + m);

table(yEffectBeg, yEffectEnd, yEffectEndJointLim)

ts = a:0.001:b;
x1s = sval2(d, y, c1, ts);
x2s = sval2(d, y, c2, ts);
x3s = sval2(d, y, c3, ts);

figure(1);
clf;
hold on;

plot(ts, x1s);
plot(ts, x2s);
plot(ts, x3s);

grid on;
