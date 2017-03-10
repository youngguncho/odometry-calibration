function [X_ji,Jminous] = inverse_2d(X_ij)
% 2D version of head2tail operation

X_ji = zeros(3,1);

x_ij = X_ij(1); y_ij = X_ij(2); t_ij = X_ij(3);

X_ji(1) = -x_ij*cos(t_ij) - y_ij*sin(t_ij);
X_ji(2) = x_ij*sin(t_ij) - y_ij*cos(t_ij);
X_ji(3) = -t_ij;

Jminous = [-cos(t_ij), -sin(t_ij), X_ji(2);
           sin(t_ij),  -cos(t_ij), -X_ji(1);
           0        ,       0    , -1];