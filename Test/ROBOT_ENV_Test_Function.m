clf
clear
clc

m_env_setup
%% Drop off container locations 
p1 = [-0.6, 2.4, 1];
plot3(p1(1), p1(2), p1(3),'*','Color', 'b' )

p2 = [-0.6, 1.85, 1];
plot3(p2(1), p2(2), p2(3),'*','Color', 'r' )

p3 = [0.55, 2.4, 1];
plot3(p3(1), p3(2), p3(3),'*','Color', 'g' )

p4 = [0.55, 1.85, 1];
plot3(p4(1), p4(2), p4(3),'*','Color', 'y' )

%% Trajectory to points from default/home position
%These are manually configured
qBlueF = [ 0.3777 0.7294 0.3277 1.6391 -0.0242 2.6180 -0.5993];
qRedF = [ 1.4336   -0.4312    0.2666    0.8083   -0.0242    2.6180   -0.5993];

% Rot = [ -0.0002, 1, 0.0002; 0.0008,   -0.0002,    1; 1,    0.0002,   -0.0008 ];

% qM = sBot.genTrajRMRC(p1);

qM = jtraj(sBot.model.getpos(), qRedF, 50);

sBot.model.plot3d(qM)
