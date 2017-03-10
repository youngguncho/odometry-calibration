function [X_prime,Jminus] = ssc_inverse(X_ij)
%SSC_INVERSE  6-DOF coordinate frame relationship.
%   [X_ji,J_MINUS] = SSC_INVERSE(X_ij) returns the Smith, Self, and
%   Cheeseman inverse coordinate frame relationship:
%     X_ji = (-)X_ij
%          = [x_ji,y_ji,z_ji,r_ji,p_ji,h_ji]'
%   where
%     X_ij is the 6-DOF representation of frame j w.r.t. frame i, X_ji is
%     the 6-DOF representation of frame i w.r.t. frame j, J_MINUS is the
%     Jacobian of the inverse operation, i.e. J_MINUS = d(X_ji)/d(X_ij)
%     evaluated at X_ij.
%
%   The above notation and associated Jacobian are based upon*: R. Smith,
%   M. Self, and P. Cheeseman.  "Estimating Uncertain Spatial Relationships
%   in Robotics".
%
%   *Note 1: If X_ij is a 6xN array, then SSC_INVERSE can operate on the
%   whole array at once returning the result as a 6xN array, X_ji, and a
%   6x6xN array, J_MINUS.
%   
%   *Note 2: my XYZ Euler angle convention follows Fossen's convention
%   which is different than the SSC's.  My RPH are HPR in SSC's notation,
%   therefore the Jacobian I return is a permutation of the Jacobian given
%   in the SSC appendix.
%
%   Ex:  express local-level w.r.t. vehicle frame
%   % x_lv is the vehicle pose in the local-level frame with covariance P_lv
%   x_lv = rand(6,1);
%   P_lv = randcov(6);
%
%   % x_vl is the pose of the local-level in the vehicle frame and 
%   % P_vl is the first order covariance
%   [x_vl,Jminus] = ssc_inverse(x_lv);
%   P_vl = Jminus*P_lv*Jminus';
%
%   See also SSC_HEAD2TAIL, SSC_TAIL2TAIL, ROTXYZ.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-10-2003      rme         Created and written.
%    04-27-2006      rme         Added capability to stack into 3rd dimension.
%    04-27-2006      rme         Renamed inverse.m to ssc_inverse.m

n = size(X_ij,2);
X_ij = reshape(X_ij,6,1,n);
x_ij = X_ij(1,1,:);
y_ij = X_ij(2,1,:);
z_ij = X_ij(3,1,:);
r_ij = X_ij(4,1,:);
p_ij = X_ij(5,1,:);
h_ij = X_ij(6,1,:);
R_ij = rotxyz(r_ij,p_ij,h_ij);

x_ji = -( R_ij(1,1,:).*x_ij + R_ij(2,1,:).*y_ij + R_ij(3,1,:).*z_ij );
y_ji = -( R_ij(1,2,:).*x_ij + R_ij(2,2,:).*y_ij + R_ij(3,2,:).*z_ij );
z_ji = -( R_ij(1,3,:).*x_ij + R_ij(2,3,:).*y_ij + R_ij(3,3,:).*z_ij );
h_ji = atan2(R_ij(1,2,:), R_ij(1,1,:));
ch_ji = cos(h_ji); sh_ji = sin(h_ji);
p_ji = atan2(-R_ij(1,3,:), R_ij(1,1,:).*ch_ji + R_ij(1,2,:).*sh_ji);
r_ji = atan2(R_ij(3,1,:).*sh_ji - R_ij(3,2,:).*ch_ji, ...
             -R_ij(2,1,:).*sh_ji + R_ij(2,2,:).*ch_ji);

X_prime = [x_ji(:), y_ji(:), z_ji(:), r_ji(:), p_ji(:), h_ji(:)]';

% compute Jacobian
if nargout > 1

  cr_ij = cos(r_ij); sr_ij = sin(r_ij);
  cp_ij = cos(p_ij); sp_ij = sin(p_ij);
  ch_ij = cos(h_ij); sh_ij = sin(h_ij);
  alpha = 1-R_ij(1,3,:).^2;   sqrt_alpha = sqrt(alpha);
  
  % N
  N = zeros(3,3,n);
  N(1,1,:) =  0;
  N(1,2,:) = -R_ij(3,1,:).*x_ij.*ch_ij - R_ij(3,1,:).*y_ij.*sh_ij + z_ij.*cp_ij;
  N(1,3,:) =  R_ij(2,1,:).*x_ij - R_ij(1,1,:).*y_ij;
  N(2,1,:) =  z_ji;
  N(2,2,:) = -R_ij(3,2,:).*x_ij.*ch_ij - R_ij(3,2,:).*y_ij.*sh_ij + z_ij.*sp_ij.*sr_ij;
  N(2,3,:) =  R_ij(2,2,:).*x_ij - R_ij(1,2,:).*y_ij;
  N(3,1,:) = -y_ji;
  N(3,2,:) = -R_ij(3,3,:).*x_ij.*ch_ij - R_ij(3,3,:).*y_ij.*sh_ij + z_ij.*sp_ij.*cr_ij;
  N(3,3,:) =  R_ij(2,3,:).*x_ij - R_ij(1,3,:).*y_ij;

  
  % Q
  Q = zeros(3,3,n);
  Q(1,1,:) = -R_ij(1,1,:)./alpha;
  Q(1,2,:) = -R_ij(1,2,:).*cr_ij./alpha;
  Q(1,3,:) =  R_ij(3,3,:).*R_ij(1,3,:)./alpha;
  Q(2,1,:) =  R_ij(1,2,:)./sqrt_alpha;
  Q(2,2,:) = -R_ij(3,3,:).*ch_ij./sqrt_alpha;
  Q(2,3,:) =  R_ij(2,3,:)./sqrt_alpha;
  Q(3,1,:) =  R_ij(1,1,:).*R_ij(1,3,:)./alpha;
  Q(3,2,:) = -R_ij(2,3,:).*ch_ij./alpha;
  Q(3,3,:) = -R_ij(3,3,:)./alpha;

  % Jacobian
  if (n > 1);
    Jminus = [-multitransp(R_ij),  N;
              zeros(3,3,n),        Q];
  else; % faster if (n == 1)
    Jminus = [-R_ij',    N;
              zeros(3),  Q];
  end;
end
