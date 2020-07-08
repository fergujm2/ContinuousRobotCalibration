clear;

%% Messing around with FourierSeries package

T = 1.3;
% f = @(x) mod(x, T).^2 - mod(x, T) + 1;
f = @(x) 4.*sin(2.*pi./T.*x);

x = 0:0.001:10;
y = f(x) + 0.01.*randn(size(x));

n = 20;

[a, b, yfit] = Fseries(mod(x, T), y, n);

figure(1);
clf;
hold on;

plot(x, y);
plot(x, yfit);

x = linspace(0, 10, 1000);
y = Fseriesval(a, b, mod(x, T));

figure(2);
clf;

plot(x, y);


%% Test Vector Fitting

T = 1.1;

f = @(t) [4.*sin(2.*pi./T.*t) - 3.*cos(2.*pi./T.*3.*t);
          1.3.*sin(2.*pi./T.*2.*t) - 4.3.*cos(2.*pi./T.*4.*t)]';

a = 1.3;
b = 10.4;

t = a:0.01:b;
Y = f(t);

n = 10;

[A, B] = FitVectorFourier(Y, a, b, T, n);

X = EvalVectorFourier(A, B, t, T);

figure(3);
clf;
hold on;

plot(t, X);
plot(t, Y);

%% Test Taking the Derivative of a Function

T = 1.1;
f = @(t) 3 + sin(1*2*pi/T.*t) + sin(2*2*pi/T.*t) + cos(2*2*pi/T.*t) + cos(3*2*pi/T.*t);
% f = @(x) mod(x, T).^2 - mod(x, T) + 1;

a = 1.3;
b = 10.4;

t = a:0.001:b;
y = f(t) + 0*randn(size(t));

n = 20;
[a, b] = FitVectorFourier(y', a, b, T, n);
[ad, bd] = DerFourier(a, b, T);
[add, bdd] = DerFourier(ad, bd, T);

yd = EvalVectorFourier(ad, bd, t, T);
ydd = EvalVectorFourier(add, bdd, t, T);

ydFinDiff = diff(y)./mean(diff(t));
yddFinDiff = diff(ydFinDiff)./mean(diff(t));

figure(4);
clf;
hold on;

plot(t, yd);
plot(t(1:end-1), ydFinDiff);

figure(5);
clf;
hold on;

plot(t, ydd);
plot(t(1:end-2), yddFinDiff);


%% Test Taking Derivatives of a Vector-Valued Function

T = 2;
% f = @(t) [2*abs(1 - mod(t, T)) - 1; sign(2*abs(1 - mod(t, T)) - 1)]';
f = @(t) [4.*sin(2.*pi./T.*t) - 3.*cos(2.*pi./T.*3.*t);
          1.3.*sin(2.*pi./T.*2.*t) - 4.3.*cos(2.*pi./T.*4.*t)]';
      
a = 1.3;
b = 10.4;

t = a:0.01:b;
X = f(t);

n = 10;

[A, B] = FitVectorFourier(X, a, b, T, n);
[Ad, Bd] = DerVectorFourier(A, B, T);
[Add, Bdd] = DerVectorFourier(Ad, Bd, T);

Y = EvalVectorFourier(A, B, t, T);
Yd = EvalVectorFourier(Ad, Bd, t, T);
Ydd = EvalVectorFourier(Add, Bdd, t, T);

Xd = diff(X, 1, 1)./mean(diff(t));
Xdd = diff(Xd, 1, 1)./mean(diff(t));

figure(6);
clf;

subplot(3,1,1);
hold on;

plot(t, X);
plot(t, Y);

subplot(3,1,2);
hold on;

plot(t(1:end-1), Xd);
plot(t, Yd);

% ylim([-7, 7]);

subplot(3,1,3);
hold on;

plot(t(1:end-2), Xdd);
plot(t, Ydd);

% ylim([-10, 10]);
