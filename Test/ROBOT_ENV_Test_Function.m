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

p5 = [0, 2.6, 1];
plot3(p5(1), p5(2), p5(3),'*','Color', 'c' )

%% Reaching for Cup position
%These are manually configure
sBot.model.plot3d(sBot.qOp)
% qC = sBot.model.getpos;
% qBlueF = [ 0.3777 0.7294 0.3277 1.6391 -0.0242 2.6180 -0.5993];
% qRedF = [ 1.4336   -0.4312    0.2666    0.8083   -0.0242    2.6180   -0.5993];
% qCup = [1.0385   -1.9437         0    1.5429         0    2.0320         0];

% trCup = transl(0,2.8,1) *  troty(pi/2)
% qM = sBot.genTraj(trCup);
% qM = jtraj(sBot.model.getpos, qCup,50);

qM = sBot.GoToCupTrajectory;
sBot.model.plot3d(qM)

%% RMRC
