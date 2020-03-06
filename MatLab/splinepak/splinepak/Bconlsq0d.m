% Bconlsq0d 
% SplinePAK: Copyright Larry Schumaker 2014
% SPAK: Copyright Larry Schumaker 2012
% Computes C^0 spline interpolant of degree d subject to convexity conditions
% Starts with a lsq fit,  then approximates the coefs subject to constraints

% Input a triangulation
[n,x,y,nt,TRI] = readtri;
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Input data points &  sample a convex function 
nd = input('input number of data points nd ');
[xd,yd] = randxy(nd);
zd = conf(xd,yd);
wd = ones(nd,1);

% adjust the triangulation by swapping
%TRI = conswap(v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,x,y,z);

% Plot triangulation and data points
figure; hold;
plot(x,y,'LineStyle','none','LineWidth',2.5); %,'Marker','*',...
%  'MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',20)
h = triplot(TRI,x,y); set(h,'LineWidth',2.5);
plot(xd,yd,'LineStyle','none','LineWidth',2,'Marker','.',...
  'MarkerFaceColor','g', 'MarkerEdgeColor','k',  'MarkerSize',20)

% Add noise 
eps = input('input eps for noise ');
if eps > 0
 rv = readnoise(nd);
 zd = zd + eps*rv;
end

% Stage 1 -- do LSQ fit
d = input('input d ');
[co,G] = lsq0d(d,x,y,v1,v2,v3,e1,e2,e3,ie1,nd,xd,yd,zd,wd);

% Evaluate this spline on a grid
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,co,ng,xmin,xmax,ymin,ymax);

% Plot the spline
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the errors
e = errg(xg,yg,g,@conf);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Check convexity
ck = conck0d(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,co);
fprintf('ck %e\n',ck);
[dx,dy,dermin] = conck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,co,ng);
fprintf('dx,dy,dermin %e, %e, %e\n',dx,dy,dermin);

% Set up convexity constraints Cc <= 0
C = con0d(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy);
sa = size(C); ncon = sa(1); 
nc = sa(2); ci = zeros(nc,1); % start with zero coefs
b = zeros(ncon,1);

% Solve the constrained minimization problem
% Need co to be a column vector
obj = @(x,co) sum((x-co).^2);
options=optimset('TolFun',1e-4,'Display','off');

tic
[c,error,exitflag,output] = fmincon(@(x) obj(x,co),...
    ci,C,b,[],[],[],[],[],options);
fprintf('error %g and flag %g\n',error,exitflag);
toc

% Evaluate the new spline on a grid
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Calculate the errors
e = errg(xg,yg,g,@conf);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Check convexity
ckmin = conck0d(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);
fprintf('ckmin %e\n',ckmin);
[dx,dy,dermin] = conck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng);
fprintf('dx,dy,dermin %5.2e, %e, %5.2e\n',dx,dy,dermin);
