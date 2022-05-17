clear
clf
clc
%% GEN ARR
mdl_planar2
hold on
axis([-2 4 -2 2 0 2])
p2.base = p2.base * transl(2,0,0);
q1 =[-0.6283         0];
q2 = [0.6283         0];
qM = jtraj(q1,q2,20);

%% GEN Barrier is breached
dlc = DigitalLightCurtain();
state = dlc.isBreached(p2, qM);

%% GEN Barrier is not breached
p2.base = p2.base * transl(-2,0,0);
dlc = DigitalLightCurtain();
state = dlc.isBreached(p2, qM);
