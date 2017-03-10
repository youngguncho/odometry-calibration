function [X_ik,Jplus] = head2tail_2d(X_ij,X_jk)
% 2D version of head2tail operation

X_ik = zeros(3,1);

x_ij = X_ij(1); y_ij = X_ij(2); t_ij = X_ij(3);
x_jk = X_jk(1); y_jk = X_jk(2); t_jk = X_jk(3);

X_ik(1) = x_jk*cos(t_ij) - y_jk*sin(t_ij) +x_ij;
X_ik(2) = x_jk*sin(t_ij) + y_jk*cos(t_ij) +y_ij;
X_ik(3) = t_ij + t_jk;

Jplus = [1 0 -(y_jk -y_ij) cos(t_ij) -sin(t_ij) 0;
         0 1 (x_jk - x_ij) sin(t_ij) cos(t_ij)  0;
         0 0 1             0         0          1];