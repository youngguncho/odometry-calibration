function plot_pose_with_cov_ellipse (X, Cov)

figure(1), hold on;
scale = 0.1;
for k=1:3:length(X) 
    plot_triangle (X(k:k+2), scale);
    plot_ellipse (X(k:k+1), Cov(k:k+1,k:k+1), 'r');
end

% generate 3xn or 6xn matrix
dim = 3
X_mat = reshape(X,3,[]);
x_min = min(X_mat(1,:)); x_max = max(X_mat(1,:));
y_min = min(X_mat(2,:)); y_max = max(X_mat(2,:));

axis([x_min-1,x_max+1,y_min-1,y_max+1]);
grid on;
hold off;
axis equal;
