clf
clear
clc

m_env_setup
%% Drop off container locations 
m_container_locations

%% Reaching for Cup position
%These are manually configure
sBot.model.plot3d(sBot.qOp)
% qC = sBot.model.getpos;
qBlueF = [ 0.3777 0.7294 0.3277 1.6391 -0.0242 2.6180 -0.5993];
qRedF = [ 1.4336   -0.4312    0.2666    0.8083   -0.0242    2.6180   -0.5993];
qCup = [1.0385   -1.9437         0    1.5429         0    2.0320         0];

trBlue = transl(-0.6,2.4,1);

qM = sBot.GoToCupTrajectory();
sBot.model.plot3d(qM);

%%
sBot.model.plot3d(sBot.qOp);
% Premium pearl when clciked
order1.name = 'Premium Pearl Milk tea';
order1.containerLocations = [p1;p2]; 

% Matcha when clciked
order2.name = 'Matcha Green Tea';
order2.containerLocations = [p3;p2]; 

% soemthe else when clciked
order3.name = 'Jasmine tea';
order3.containerLocations = [p4;p3]; 

sBot.StartOrderTrajectory(Bottle, order1)
