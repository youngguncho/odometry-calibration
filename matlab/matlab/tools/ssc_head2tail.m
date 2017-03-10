function [X_ik,Jplus] = ssc_head2tail(X_ij,X_jk)
%SSC_HEAD2TAIL  6-DOF coordinate frame composition.
%   [X_ik,J_PLUS] = SSC_HEAD2TAIL(X_ij,X_jk) returns the Smith, Self, and
%   Cheesman head-to-tail coordinate frame composition:
%     X_ik = X_ij (+) X_jk
%          = [x_ik,y_ik,z_ik,r_ik,p_ik,h_ik]'  
%   where
%     X_ij is the 6-DOF representation of frame j w.r.t. frame i, X_jk is
%     the 6-DOF representation of frame k w.r.t. frame j, X_ik is the 6-DOF
%     representation of frame k w.r.t. frame i, J_PLUS is the Jacobian of
%     the composition operation, i.e. J_PLUS = d(X_ik)/d(X_ij,X_jk)
%     evaluated at X_ij, X_jk.
%
%   The above notation and associated Jacobian are based upon*: R. Smith,
%   M. Self, and P. Cheeseman.  "Estimating Uncertain Spatial Relationships
%   in Robotics".
%
%   *Note 1: If X_ij and X_jk are each 6xN arrays, then SSC_HEAD2TAIL can 
%   operate on the whole array at once returning the result as a 6xN array, 
%   X_ik, and a 6x12xN array, J_PLUS.  Furthermore, if only one of the 
%   arguments is a 6xN, and the other a 6x1, then the 6x1 will be
%   internally expanded to a 6xN array.
%   
%   *Note 2: my XYZ Euler angle convention follows Fossen's convention
%   which is different than the SSC's.  My RPH are HPR in SSC's notation,
%   therefore the Jacobian I return is a permutation of the Jacobian given
%   in the SSC appendix.
%
%   Ex:  transformation from vehicle pose to camera pose
%   % x_lv is the vehicle pose in the local-level frame with covariance P_lv
%   x_lv = rand(6,1);
%   P_lv = randcov(6);
%
%   % x_vc is the static pose of the camera in the vehicle frame 
%   % with covariance P_vc
%   x_vc = [1.4,0,0,0,0,pi/2]';
%   P_vc = zeros(6);
%
%   % x_lc is the pose of the camera in the local-level frame and 
%   % P_lc is the first order covariance
%   [x_lc,Jplus] = ssc_head2tail(x_lv,x_vc);
%   P_lc = Jplus*[P_lv, zeros(6); zeros(6), P_vc]*Jplus';
%
%   See also SSC_INVERSE, SSC_TAIL2TAIL, ROTXYZ.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-10-2003      rme         Created and written.
%    11-15-2003      rme         Changed K_1(3,2) & K_2(3,2) to use tan(p_3)
%    04-27-2006      rme         Added capability to stack into 3rd dimension.
%    04-27-2006      rme         Renamed head2tail.m to ssc_head2tail.m
%    04-30-2006      rme         Added internal repmat capability.

% internally expand a 6x1 to match other arguments dim
n = size(X_ij,2);
m = size(X_jk,2);
if (n > m) && (m == 1);
  X_jk = repmat(X_jk,[1 n]);
elseif (m > n) && (n == 1);
  X_ij = repmat(X_ij,[1 m]);
elseif (m ~= n);
  error('X_ij and X_jk must each be 6x1 or 6xN arrays.');
end;
n = size(X_ij,2);

% reshape each array from a 6xn to a 6x1xn
X_ij = reshape(X_ij,[6 1 n]);
X_jk = reshape(X_jk,[6 1 n]);

% extract pose elements
x_ij = X_ij(1,1,:);
y_ij = X_ij(2,1,:);
z_ij = X_ij(3,1,:);
r_ij = X_ij(4,1,:);
p_ij = X_ij(5,1,:);
h_ij = X_ij(6,1,:);
R_ij = rotxyz(r_ij,p_ij,h_ij);
t_ij = [x_ij; y_ij; z_ij];

x_jk = X_jk(1,1,:);
y_jk = X_jk(2,1,:);
z_jk = X_jk(3,1,:);
r_jk = X_jk(4,1,:);
p_jk = X_jk(5,1,:);
h_jk = X_jk(6,1,:);
R_jk = rotxyz(r_jk,p_jk,h_jk);
t_jk = [x_jk; y_jk; z_jk];

% translation component of X_ik
if (n > 1);
  t_ik = multiprod(R_ij,t_jk) + t_ij;
else;
  t_ik = R_ij*t_jk + t_ij; % faster if (n == 1)
end;
x_ik = t_ik(1,1,:);
y_ik = t_ik(2,1,:);
z_ik = t_ik(3,1,:);

% angle component of X_ik
if (n > 1);
  R_ik = multiprod(R_ij,R_jk);
else;
  R_ik = R_ij*R_jk; % faster if (n == 1)
end;
h_ik = atan2(R_ik(2,1,:), R_ik(1,1,:));

ch_ik = cos(h_ik); sh_ik = sin(h_ik);

p_ik = atan2(-R_ik(3,1,:), R_ik(1,1,:).*ch_ik + R_ik(2,1,:).*sh_ik);
r_ik = atan2(R_ik(1,3,:).*sh_ik - R_ik(2,3,:).*ch_ik, ...
	    -R_ik(1,2,:).*sh_ik + R_ik(2,2,:).*ch_ik);

X_ik = [x_ik(:), y_ik(:), z_ik(:), r_ik(:), p_ik(:), h_ik(:)]';

% compute the Jacobian
if nargout > 1
  cr_ij = cos(r_ij); sr_ij = sin(r_ij);
  cp_ij = cos(p_ij); sp_ij = sin(p_ij);
  ch_ij = cos(h_ij); sh_ij = sin(h_ij);
  
  cr_ik = cos(r_ik); sr_ik = sin(r_ik);
  cp_ik = cos(p_ik); tp_ik = tan(p_ik);
  
  cp_jk = cos(p_jk);
  
  cr_ikmjk = cos(r_ik-r_jk); sr_ikmjk = sin(r_ik-r_jk);
  ch_ikmij = cos(h_ik-h_ij); sh_ikmij = sin(h_ik-h_ij);
  
  % M
  M = zeros(3,3,n);
  M(1,1,:) =  R_ij(1,3,:).*y_jk - R_ij(1,2,:).*z_jk;
  M(1,2,:) =  (z_ik-z_ij).*ch_ij;
  M(1,3,:) = -(y_ik-y_ij);
  M(2,1,:) =  R_ij(2,3,:).*y_jk - R_ij(2,2,:).*z_jk;
  M(2,2,:) =  (z_ik-z_ij).*sh_ij;
  M(2,3,:) =  x_ik-x_ij;
  M(3,1,:) =  R_ij(3,3,:).*y_jk - R_ij(3,2,:).*z_jk;
  M(3,2,:) = -x_jk.*cp_ij - y_jk.*sp_ij.*sr_ij - z_jk.*sp_ij.*cr_ij;
  M(3,3,:) =  0;

  % K_1
  K_1 = zeros(3,3,n);
  K_1(1,1,:) =  cp_ij.*ch_ikmij./cp_ik;
  K_1(1,2,:) =  sh_ikmij./cp_ik;
  K_1(1,3,:) =  0;
  K_1(2,1,:) = -cp_ij.*sh_ikmij;
  K_1(2,2,:) =  ch_ikmij;
  K_1(2,3,:) =  0;
  K_1(3,1,:) =  ( R_jk(1,2,:).*sr_ik + R_jk(1,3,:).*cr_ik )./cp_ik;
  K_1(3,2,:) =  tp_ik.*sh_ikmij;
  K_1(3,3,:) =  1;

  % K_2
  K_2 = zeros(3,3,n);
  K_2(1,1,:) =  1;
  K_2(1,2,:) =  tp_ik.*sr_ikmjk;
  K_2(1,3,:) =  ( R_ij(1,3,:).*ch_ik + R_ij(2,3,:).*sh_ik )./cp_ik;
  K_2(2,1,:) =  0;
  K_2(2,2,:) =  cos(r_ik-r_jk);
  K_2(2,3,:) = -cp_jk.*sr_ikmjk;
  K_2(3,1,:) =  0;
  K_2(3,2,:) =  sr_ikmjk./cp_ik;
  K_2(3,3,:) =  cp_jk.*cr_ikmjk./cp_ik;

  % Jacobian
  if (n > 1);
    I = repmat(eye(3),[1 1 n]);
  else;
    I = eye(3); % faster if (n == 1)
  end;
  O = zeros(3,3,n);
  Jplus = [I,  M,    R_ij,  O;
           O,  K_1,  O,     K_2];
end

