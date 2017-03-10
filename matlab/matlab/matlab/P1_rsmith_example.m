% CE554 lec 14 Mobile robot
% Example in R. Smith 1990 paper

clear all; close all;

DTOR = pi/180;
RTOD = 180/pi;

% robot starts at the origin = world frame
xr = [0 0 0]'; Sr = zeros(3,3);

% robot sees the object #1
x1 = [-1 1 0]'; R = diag([0.2^2, 0.2^2, (1*DTOR)^2]);
X = [xr; x1];
Cov = [Sr zeros(3,3);
       zeros(3,3) R];

% robot moves
u = [1 1 -pi/4]'; Q = diag([0.2^2, 0.2^2, (1*DTOR)^2]);
[yr, J] = % hint: you can use head2tail_2d func
Sy =

X = 
Cov = 

% robot sees #2
z2 = [-1 1 0]'; R = diag([0.2^2, 0.2^2, (1*DTOR)^2]);
[x2, J] = 
S2 = 

X = 
Cov = 