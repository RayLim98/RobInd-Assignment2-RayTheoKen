clear
clf
clc

bot = Sawyer(0,0,0);
bot.model.teach();
hold on
%% Robot set up robot
q0 = bot.model.getpos();
qr = [0 -pi -pi/2 pi/2 0 -pi/2 0];

qM = jtraj(q0,qr,30);

bot.model.plot(qM);
q0 = bot.model.getpos();
bot.model.teach();

%% Robot Trajectory Generation
mask = [1,1,1,0,0,0];
% Right
tr = transl(0,-1,0);
qM = bot.genTraj(tr);
bot.model.plot(qM)
% Left
tr = transl(0,-1,0);
qM = bot.genTraj(tr);
bot.model.plot(qM)
% Behind

% Above 

%% Stops on Collision

