function [X_jk,J] = ssc_tail2tail(X_ij,X_ik)
%SSC_TAIL2TAIL  6-DOF coordinate frame composition.
%   [X_jk,J] = SSC_TAIL2TAIL(X_ij,X_ik) returns the Smith, Self, and
%   Cheeseman tail-to-tail coordinate frame composition:
%     X_jk = (-)X_ij (+) X_ik
%          = [x_jk,y_jk,z_jk,r_jk,p_jk,h_jk]'
%   where
%     X_ij is the 6-DOF representation of frame j w.r.t. frame i, X_ik is
%     the 6-DOF representation of frame k w.r.t. frame i, X_jk is the 6-DOF
%     representation of frame k w.r.t. frame j, J is the Jacobian of the
%     tail-to-tail operation, i.e. J = d(X_jk)/d(X_ij,X_ik) evaluated at
%     X_ij, X_ik.
%
%   The above notation and associated Jacobian are based upon*: R. Smith,
%   M. Self, and P. Cheeseman.  "Estimating Uncertain Spatial Relationships
%   in Robotics".
%
%   *Note 1: If X_ij and X_ik are each 6xN arrays, then SSC_TAIL2TAIL can 
%   operate on the whole array at once returning the result as a 6xN array, 
%   X_jk, and a 6x12xN array, J.  Furthermore, if only one of the arguments
%   is a 6xN, and the other a 6x1, then the 6x1 will be internally expanded
%   to a 6xN array.
%   
%   *Note: my XYZ Euler angle convention follows Fossen's convention which
%   is different than the SSC's.  My RPH are HPR in SSC's notation,
%   therefore the Jacobian I return is a permutation of the Jacobian given
%   in the SSC appendix.
%
%   Ex:  relative pose of vehicle frame vj w.r.t vehicle frame vi
%   % x_lvi is the vehicle pose in the local-level frame at time t_i
%   % with covariance P_lvi
%   x_lvi = rand(6,1);
%   P_lvi = randcov(6);
%
%   % x_lvj is the vehicle pose in the local-level frame at time t_j
%   % with covariance P_lvj
%   x_lvj = rand(6,1);
%   P_lvj = randcov(6);
%
%   % x_vivj is the pose of the vehicle i w.r.t. vehicle frame j and 
%   % P_vivj is the first order covariance
%   [x_vivj,J] = ssc_tail2tail(x_lvi,x_lvj);
%   P_vivj = J*[P_lvi, zeros(6); zeros(6), P_lvj]*J';
%
%   See also SSC_HEAD2TAIL, SSC_INVERSE, ROTXYZ.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-10-2003      rme         Created and written.
%    04-27-2006      rme         Added capability to stack into 3rd dimension.
%    04-27-2006      rme         Renamed tail2tail.m to ssc_tail2tail.m
%    04-30-2006      rme         Added internal repmat capability.

% internally expand a 6x1 to match other arguments dim
n = size(X_ij,2);
m = size(X_ik,2);
if (n > m) && (m == 1);
  X_ik = repmat(X_ik,[1 n]);
elseif (m > n) && (n == 1);
  X_ij = repmat(X_ij,[1 m]);
elseif (m ~= n);
  error('X_ij and X_ik must each be 6x1 or 6xN arrays.');
end;
n = size(X_ij,2);


[X_ji,Jminus] = ssc_inverse(X_ij);
[X_jk,Jplus] = ssc_head2tail(X_ji,X_ik);
if (n > 1);
  J = [multiprod(Jplus(:,1:6,:),Jminus), Jplus(:,7:12,:)];
else; % faster if (n == 1)
  J = [Jplus(:,1:6,:)*Jminus, Jplus(:,7:12,:)];
end;
