clear; close all;

% addpath of tools
addpath('tools');

% data load
load('../data/encoder.mat');
load('../data/gps_and_truth.mat');
enc_t = enc_cnt(:,2);
enc_l = enc_cnt(:,3);
enc_r = enc_cnt(:,4);

% const input
ENC_RES = 2048;
DTOR = pi/180;

% estimated variable
d_l = 0.65;
d_r = 0.65;
w_b = 1.67;

% init pose and encoder count
p0 = [0, 0, 0]';
S0 = diag([0.01^2, 0.01^2, (0.1*DTOR)^2]);
enc_l0 = 0;
enc_r0 = 0;
pre_match_enc = 1;

th = 0.05;
S_en = diag([0.01^2, 0.01^2, 0.215^2]);
err = 10000;
step_size = 1;
% fcn_drawPath(enc_cnt, d_l, d_r, w_b);
%
while (err > th)
    err = 0;
    H = zeros(3);
    b = zeros(1,3);
    pre_match_enc = 1;
    for i = 2:size(gps_pose,1)
        match_enc = knnsearch(enc_t, gps_pose(i,1));   
        c_l = (enc_l(match_enc)-enc_l(pre_match_enc))*pi/ENC_RES;
        c_r = (enc_r(match_enc)-enc_r(pre_match_enc))*pi/ENC_RES;
        l_avg = (c_l*d_l + c_r*d_r)/2;
        l_diff = (c_r*d_r - c_l*d_l);

        dth = l_diff / w_b;
        dx = l_avg*cos(dth);
        dy = l_avg*sin(dth);

        X_dk = [dx, dy, dth];
        [Z_dk, J_z] = tail2tail_2d(gps_pose(i-1, 2:end), gps_pose(i, 2:end));
        [e, Je] = tail2tail_2d(X_dk, Z_dk);
        J_delta = fcn_jacobian(l_avg, c_r, c_l, w_b, dth);
        J = Je(:,1:3)*J_delta;
        
        err = err+norm(e);
        
        
        H = H + J'*inv(S_en)*J;
        b = b + e'*inv(S_en)*J;
        pre_match_enc = match_enc;
    end
    dX = H\-b';
    dX = step_size*dX;
    d_l = d_l+dX(1);
    d_r = d_r+dX(2);
    w_b = w_b+dX(3);
    err
    fcn_drawPath(enc_cnt, d_l, d_r, w_b);
    hold on;
    plot(gps_pose(:,2), gps_pose(:,3), 'r.');

end



