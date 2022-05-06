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

%% Trajectory to points
qBlue =  [0 0 0 0 1.5708 -1.5708 0];
tr = sBot.model.fkine(qBlue);

% Rot = [ -0.0002, 1, 0.0002; 0.0008,   -0.0002,    1; 1,    0.0002,   -0.0008 ];

qM = sBot.genTrajRMRC(p1);

sBot.model.plot3d(qM)
