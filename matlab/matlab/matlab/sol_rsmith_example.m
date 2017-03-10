% CE554 lec 14 Mobile robot
% Example in R. Smith 1990 paper

clear all; close all;

DTOR = pi/180;
RTOD = 180/pi;

% robot starts at the origin = world frame
xr = [0 0 0]'; Sr = zeros(3,3);

% robot sees the object #1
x1 = [-1 1 0]'; R = diag([0.1^2, 0.2^2, (1*DTOR)^2]);

X = [xr; x1];
Cov = [Sr zeros(3,3);
       zeros(3,3) R];

% robot moves
u = [1 1 -pi/4]'; Q = diag([0.15^2, 0.05^2, (1*DTOR)^2]);
[y_r, J] = head2tail_2d(xr, u);
Sy = J*[Sr zeros(3,3); zeros(3,3) Q]*J';

X = [y_r; x1];
Cov = [Sy zeros(3,3);
       zeros(3,3) R];
   
% robot sees #2
z2 = [-1.41 0.0 0]'; R = diag([0.1^2, 0.2^2, (1*DTOR)^2]);
[x2,J] = head2tail_2d(y_r, z2);
S2 = J*[Sy zeros(3,3); zeros(3,3) R]*J';

X = [y_r; x1; x2];
Cov = [Cov zeros(6,3);
       zeros(3,6) R];
   
plot_pose_with_cov_ellipse (X, Cov)