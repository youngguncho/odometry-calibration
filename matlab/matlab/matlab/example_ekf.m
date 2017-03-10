% CE554 lec 16 EKF

clear all; close all;

DTOR = pi/180;
RTOD = 180/pi;

% robot starts at the origin = world frame
x0 = [0 0 0]'; S0=diag([0.1^2, 0.1^2, (0.01*DTOR)^2]);

% robot moves 
Q = diag([0.1,0.1,1*DTOR].^2);
w = mvnrnd([0,0 0]',Q,1)';
u0 = [1.9 4.1 -pi/4]';

% make a prediction
[x1_p,J] = head2tail_2d(x0, u0);
x1_true = x1_p + w;
S1_p = J(:,1:3)*S0*J(:,1:3)'+Q;

X_estim = [x0; x1_p];
X_true = [x0; x1_true];

figure(1);
plot_mobile_robot (X_estim, '2d')
hold on
plot_ellipse (x0(1:2),S0(1:2,1:2),'r');
plot_ellipse (x1_p(1:2),S1_p(1:2,1:2),'r');
plot(x1_true(1),x1_true(2),'rx');
hold off;

%% update with meas 1
% measurement #1 z = x1
z1 = 2; R1 = 1;
H = [1 0 0];

K = S1_p*H'*inv(H*S1_p*H' +R1);
x1 = x1_p + K*(z1 - x1_p(1));
S1 = S1_p - K*H*S1_p;

%% plot
figure(2);
X_estim = [x0; x1];
plot_mobile_robot (X_estim, '2d')
hold on
plot_ellipse (x0(1:2),S0(1:2,1:2),'r');
plot_ellipse (x1_p(1:2),S1_p(1:2,1:2),'r');
plot(x1_true(1),x1_true(2),'rx');

x1_p
x1