clc; clear; close all;

addpath('tools');
% system config
load('../data/encoder.mat');
enc_cnt_L = enc_cnt(:,3);
enc_cnt_R = enc_cnt(:,4);

VEHICLE_TREAD = 1.65;
WHEEL_DIAMETER = 0.670;           
ENCODER_RESOLUTION = 2048;
DTOR = pi/180;

%% initial value
p0 = [0, 0, 0]';
S0 = diag([0.01^2, 0.01^2, (0.1*DTOR)^2]);

% we use cell for convinience
pose_global = cell(size(enc_cnt,1),1);
S_global = cell(size(enc_cnt,1),1);

pose_global{1} = p0;
S_global{1} = S0;

% Encoder covariance
Enc_cov = diag([0.0005, 0.0005]).^2;

for k = 2:size(enc_cnt,1)
    % calculate relative motion from wheel odometry
    dL = ((enc_cnt_L(k)-enc_cnt_L(k-1)) * WHEEL_DIAMETER * pi / ENCODER_RESOLUTION);
    dR = ((enc_cnt_R(k)-enc_cnt_R(k-1)) * WHEEL_DIAMETER * pi / ENCODER_RESOLUTION);
    dist = (dL+dR)/2;
    
    dth = (dR - dL) / VEHICLE_TREAD;
    dx = dist * cos (dth);
    dy = dist * sin (dth); 
      
    % robot starts from initial pose (global coordinate = initial coordinate)
    % (a) First compute the control input mean and covariance
    u = [dx, dy, dth];      % control input u
    
    J_odo = [1/2*cos(dth) 1/2*cos(dth); 1/2*sin(dth) 1/2*sin(dth); 1/2+1/VEHICLE_TREAD 1/2-1/VEHICLE_TREAD];
    Q = J_odo*Enc_cov*J_odo';
    
    % (b) plot entire trajectory using mean
    X_previous = pose_global{k-1};
    [X_current, J] = head2tail_2d (X_previous, u);
    pose_global{k} = X_current;
    
    % (c) covariance
    S_previous = S_global{k-1};
    S_current = J*[S_previous zeros(3,3); zeros(3,3) Q]*J';
    S_global{k} = S_current;
    
    U{k} = u;
end

% plot
figure(1); 
for k = 1:length(pose_global)
    scale = 2;
    plot_triangle(pose_global{k}, scale);  hold on;% plot sample position
    
%     if k == size(enc_cnt,1)
%         plot_ellipse (pose_global{k}(1:2),S_global{k}(1:2,1:2),'b');
%     else
%         plot_ellipse (pose_global{k}(1:2),S_global{k}(1:2,1:2),'r');
%     end
    
    axis equal; 
    xlabel('x'); ylabel('y');
    axis([-120 120 -10 225])
    grid on
%     drawnow;
    % pause(0.02);
end


for i = 1:length(pose_global)
    pose_global{i} = pose_global{i}';
end
gps_pose = [enc_cnt(:,2), cell2mat(pose_global)];
gps_pose = gps_pose(1:1:end, :);

% data for matlab
true_diameter = WHEEL_DIAMETER;
true_base = VEHICLE_TREAD;
disp (['Groundtruth [d_l, d_r, w_b] = [' num2str(WHEEL_DIAMETER) ', ' num2str(VEHICLE_TREAD) ']']);
save ('../data/gps_and_truth.mat', 'true_diameter', 'true_base', 'gps_pose');

% data for C++
fid = fopen('calib_data.txt', 'w');
fprintf(fid, '%d %d\n', size(enc_cnt,1), size(gps_pose,1));
fprintf(fid, '%ld %ld %ld\n', [enc_cnt(:,2)'; enc_cnt(:,3)'; enc_cnt(:,4)']);
fprintf(fid, '%ld %f %f %f\n', [gps_pose(:,1)'; gps_pose(:,2)'; gps_pose(:,3)'; gps_pose(:,4)']);
fclose(fid);