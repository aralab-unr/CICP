clc;close all;
m = 50; % width of grid
n = m^2; % number of points
[X,Y] = meshgrid(linspace(-2,2,m), linspace(-2,2,m));
X = reshape(X,1,[]);
Y = reshape(Y,1,[]);
Z = sin(X).*cos(Y);
% Create the data point-matrix
D = [X; Y; Z];
% Translation values (a.u.):
Tx = 0.5;
Ty = -0.3;
Tz = 0.2;
% Translation vector
T = [Tx; Ty; Tz];
% Rotation values (rad.):
rx = 0.3;
ry = -0.2;
rz = 0.05;
Rx = [1 0 0;
      0 cos(rx) -sin(rx);
      0 sin(rx) cos(rx)];
  
Ry = [cos(ry) 0 sin(ry);
      0 1 0;
      -sin(ry) 0 cos(ry)];
  
Rz = [cos(rz) -sin(rz) 0;
      sin(rz) cos(rz) 0;
      0 0 1];
% Rotation matrix
R = Rx*Ry*Rz;
% Transform data-matrix plus noise into model-matrix 
M = R * D + repmat(T, 1, n);
% Add noise to model and data
rng(2912673);
M = M + 0.01*randn(3,n);
D = D + 0.01*randn(3,n);
%%Implement ICP ....
[Ricp Ticp ER t] = icp_test(M, D, 15);
% Transform data-matrix using ICP result
Dicp = Ricp * D + repmat(Ticp, 1, n);
figure;
plot3(M(1,:),M(2,:),M(3,:),'bo',D(1,:),D(2,:),D(3,:),'r.');
figure;
plot3(M(1,:),M(2,:),M(3,:),'bo',Dicp(1,:),Dicp(2,:),Dicp(3,:),'r.');
