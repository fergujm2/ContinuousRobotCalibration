% Bcon03 
% SplinePAK: Copyright Larry Schumaker 2014
% PURPOSE:
%   Computes a convex C0 cubic spline interpolating scattered data
%   uses scat03 for the first stage, then approximate the coefs in the lsq 
%   sense subject to convexity constraints

% Input a triangulation
[n,x,y,nt,TRI] = readtri;
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Sample a convex function
z = conf(x,y);

% Swap edges to get an admissible triangulation
TRI = conswap(v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,x,y,z);
figure; triplot(TRI,x,y);
[nb,ne,nt,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy,...
vadj,eadj,adjstart,tadj,tstart,area,TRI] = trilists(x,y,TRI);

% Stage 1: local interpolation with estimated values at domain pts
co = scat03(x,y,z,v1,v2,v3,e1,e2,e3,ie1,ie2,adjstart,vadj); 
d = 3;

% plot the first-stage surface
ng = 51; xmin = min(x); xmax = max(x); ymin = min(y); ymax = max(y);
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,co,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Check the convexity
ckmin = conck0d(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,co);
fprintf('conck0d %5.2e\n',ckmin );
[dx,dy,dermin] = conck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,co,ng);
fprintf('dx,dy,dermin %5.2e, %5.2e, %5.2e\n',dx,dy,dermin);

% calculate the errors
e = errg(xg,yg,g,@conf);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));

% Set up convexity constraints
m = input('input m for order of convexity conditions  ');
A1 = conpatch(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,m);
A2 = conedge(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,bdy);
A = -[A1;A2];
sa = size(A); ncon = sa(1); nc = sa(2);
b = zeros(ncon,1);

% Set up the interpolation constraints
B = sparse(n,nc); Aeq = sparse([eye(n),zeros(n,nc-n)]); beq = z;

% Find a good starting set of coefs  -- degree raise a C0 linear spline
% on the swapped triangulation tri*
ds = 1; cs = z;
for i = 1:d-1
 [ds,cs] = degraise(ds,n,v1,v2,v3,e1,e2,e3,ie1,ie2,cs);
end

% Check convexity
ckmin = conck0d(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,cs);
fprintf('conck0d %e\n',ckmin);
[dxmin,dymin,dermin] = conck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,cs,ng);
fprintf('dx,dy,dermin %5.2e, %5.2e, %5.2e\n',dxmin,dymin,dermin);

% Solve the constrained minimization problem
% Need co to be a column vector
aco = size(co);
if aco(1) == 1
  co = co';
end
obj = @(x,co) sum((x-co).^2);
options=optimset('TolFun',1e-4,'Display','off','MaxFunEvals',30000 );

tic
[c,error,exitflag,output] = fmincon(@(x) obj(x,co),...
    cs,A,b,Aeq,beq,[],[],[],options);
fprintf('error %5.2e and flag %g\n',error,exitflag);
toc

% plot the convex spline
[xg,yg,g] = valspgrid(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng,xmin,xmax,ymin,ymax);
figure; surfl(xg,yg,g');  colormap(copper);

% Check convexity
ckmin = conck0d(d,x,y,v1,v2,v3,e1,e2,e3,ie1,ie2,tril,trir,c);
fprintf('ckmin %e\n',ckmin);
[dxmin,dymin,dermin] = conck(d,x,y,v1,v2,v3,e1,e2,e3,ie1,c,ng);
fprintf('dx,dy,dermin %5.2e, %5.2e, %5.2e\n',dxmin,dymin,dermin);

% Calculate the errors
e = errg(xg,yg,g,@conf);
fprintf('emax =%5.2e, RMS = %5.2e\n',norm(e,inf),erms(e));
