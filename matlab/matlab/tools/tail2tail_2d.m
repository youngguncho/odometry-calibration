function [X_jk,Jcomp] = tail2tail_2d(X_ij,X_ik)
% 2D version of head2tail operation

[X_ji, Jminus] = inverse_2d(X_ij);
[X_jk, Jplus] = head2tail_2d(X_ji, X_ik);

Jcomp = [Jplus(:,1:3)*Jminus, Jplus(:,4:end)];
