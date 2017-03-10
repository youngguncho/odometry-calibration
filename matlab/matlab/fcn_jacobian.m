function [ J ] = fcn_jacobian( l_avg, c_r, c_l, w_b, dth )
%FCN_JACOBIAN Summary of this function goes here
%   Detailed explanation goes here

J = [c_l*(l_avg*sin(dth)/w_b+0.5*cos(dth)), c_r*(-l_avg*sin(dth)/w_b+0.5*cos(dth)), dth*l_avg*sin(dth)/w_b; ...
     c_l*(-l_avg*cos(dth)/w_b+0.5*sin(dth)), c_r*(l_avg*cos(dth)/w_b+0.5*sin(dth)), -dth*l_avg*cos(dth)/w_b; ...
     -c_l/w_b                              , c_r/w_b                              , -dth/w_b];

end

