function [z] = plot_ellipse(m,S,pltopt)
%
% plot an elliptical contour of a bivariate Gaussian density
%
% m = mean (column vector), S = covariance matrix
% z = ellipse boundary points for plotting, using 
% >> plot(z(1,:),z(2,:))
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2015.09.15      ak          moved to lib repo

d = length(m);
if d~=2
    error('plot_ellipse assumes 2-d data')
end

% beta = theoretical mass enclosed
beta = .9;
r = sqrt(chi2inv(beta,2));

% plot circle
N = 500;
t = linspace(0,2*pi,N+1); t(end) = [];
u = r*[cos(t); sin(t)]; % circle

% transform to ellipse
[V,D]=eig(S);
z = repmat(m,1,N)+ V*sqrt(D)*u; 

plot(z(1,:),z(2,:),pltopt);
