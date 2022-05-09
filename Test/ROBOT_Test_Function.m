clear
clf
clc

bot = Sawyer(0,0,0.2);
hold on
brick = PlyObject('Brick.ply', [-0.5 0.5 0.2], 0);
%% Robot set up robot
q0 = bot.model.getpos();

% qr = [0 -pi -pi/2 pi/2 pi/2 -pi/2 0];
qr = [0 0 0 0 -pi/2 pi/2 0];

qM = jtraj(q0,qr,30);

bot.model.plot3d(qM);

q0 = bot.model.getpos();

%% Robot Trajectory Generation

% mask = [1,1,1,0,0,0];
% % % Right
% tr = transl(0,-1,0);
% qM = bot.genTraj(tr);
% bot.model.plot3d(qM)
% % Left
% tr = transl(0,-1,0);
% qM = bot.genTraj(tr);
% bot.model.plot3d(qM)

%% Test RMRC
bot.model.plot3d(qr)
q0 = bot.model.getpos();
tr = bot.model.fkine(q0);
x0 = tr(1:3, 4)';
xf = brick.pose();
xf = xf(1:3, 4)';
qM = bot.genTrajRMRC(xf);
bot.model.plot3d(qM);

%% 

