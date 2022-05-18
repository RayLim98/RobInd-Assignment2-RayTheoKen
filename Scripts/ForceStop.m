% % % % % %%
% % % % % classdef P2 < handle
% % % % %     
% % % % %     
% % % % % properties
% % % % %         model;
% % % % %         workspace = [-2 2 -2 2 0 2];   
% % % % % 
% % % % %     
% % % % %  % Bot state
% % % % %         
% % % % %         canOperate = true;
% % % % %         isHolding = false;
% % % % %         holdingIndex = 0;
% % % % %         releaseIndex = 0;
% % % % %         qMResume = [];
% % % % %         idxResume = 0;
% % % % %     
% % % % %         
% % % % % 
% % % % % a1 = 0.5; %Link 1 length 
% % % % % a2 = 0.4; %Link 2 length
% % % % % 
% % % % % 
% % % % % p2 = SerialLink([
% % % % %     Revolute('d', 0, 'a', a1, 'alpha', 0, 'standard')
% % % % %     Revolute('d', 0, 'a', a2, 'alpha', 0, 'standard')
% % % % %     ], ...
% % % % %     'name', 'two link'); 
% % % % % 
% % % % % q = zeros(1,2);%use this instead
% % % % % scale = 0.5; 
% % % % % workspace = [-2 2 -2 2 -0.1 2];                                       % Set the size of the workspace when drawing the robot
% % % % % p2.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
% % % % % 
% % % % % 
% % % % %         
% % % % % %% 
% % % % %         
% % % % % sBot = Sawyer(0,2,1); %generate sawyer model 
% % % % % hold on
% % % % % Bottle = PlyObject('CupHeadLess.ply', [0,2.9,0.9], pi/2);
% % % % % Lip = PlyObject('Lid.ply', [0,2.9,0.9], pi/2);
% % % % % PlyObject('CupHeadLess.ply', [0,2.9,0.9], pi/2);
% % % % % PlyObject('SideTable.ply', [0 1.25 0], 0);
% % % % % PlyObject('BobaBar.ply', [0,0,0], 0);
% % % % % PlyObject('BotStand.ply', [0,2,0], 0);
% % % % % Customer = PlyObject('Sonic.ply', [2,1,0], -pi/2);
% % % % % Employee = PlyObject('Sonic.ply', [0,0,0], 0);
% % % % % Container1 = PlyObject('BrnContainer.ply', [-0.89,2.3,0.9], pi/2);
% % % % % Container2 = PlyObject('LavContainer.ply', [-0.89,1.75,0.9], pi/2);
% % % % % Container3 = PlyObject('GrnContainer.ply', [0.8,2,0.9], 3/2*pi);
% % % % % Container4 = PlyObject('BlkContainer.ply', [0.8,2.55,0.9], 3/2*pi);
% % % % % Light_Curtain1 = PlyObject('Light_curtain_pair.ply', [1.3,1.75,-0.1], pi);
% % % % % % Light_Curtain2 = PlyObject('Light_curtain_pair.ply', [1,1,-0.1], 0);
% % % % % 
% % % % % 
% % % % % % Set view
% % % % % %camlight
% % % % % axis([-3 3 -2 4 -0.01 2])
% % % % % view(45, 45)
% % % % % 
% % % % % % Render floor as image
% % % % % img = imread('floor.jpeg');     
% % % % % xImage = [2 2; -2 -2];              % The x data for the image corners
% % % % % yImage = [-2 3.5; -2 3.5];              % The y data for the image corners
% % % % % zImage = [0 0; 0 0];                % The z data for the image corners
% % % % % surf(xImage,yImage,zImage,...       % Plot the surface
% % % % %      'CData',img,...
% % % % %      'FaceColor','texturemap');
% % % % %  
% % % % %  % Render background as image
% % % % % img = imread('mall.jpeg');     
% % % % % xImage = [2 -2; -2 2];              % The x data for the image corners
% % % % % yImage = [-2 -2; -2 -2];              % The y data for the image corners
% % % % % zImage = [1.5 1.5; 0 0];                % The z data for the image corners
% % % % % surf(xImage,yImage,zImage,...       % Plot the surface
% % % % %      'CData',img,...
% % % % %      'FaceColor','texturemap');
% % % % % 
% % % % % q = zeros(1,2);%use this instead
% % % % % scale = 0.5; 
% % % % % workspace = [-2 2 -2 2 -0.1 2];                                       % Set the size of the workspace when drawing the robot
% % % % % 
% % % % % p2.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
% % % % % 
% % % % % BREACH_TESTING;
% % % % % p2Collide; %USE THIS ALTERNATIVE METHOD
%% Collision Detection
%  Render Objects
sBot = Sawyer(0,2,1); %generate sawyer model 
hold on
Bottle = PlyObject('CupHeadLess.ply', [0,2.9,0.9], pi/2);
Lip = PlyObject('Lid.ply', [0,2.9,0.9], pi/2);
PlyObject('CupHeadLess.ply', [0,2.9,0.9], pi/2);
PlyObject('SideTable.ply', [0 1.25 0], 0);
PlyObject('BobaBar.ply', [0,0,0], 0);
PlyObject('BotStand.ply', [0,2,0], 0);
Customer = PlyObject('Sonic.ply', [2,1,0], -pi/2);
Employee = PlyObject('Sonic.ply', [0,0,0], 0);
Container1 = PlyObject('BrnContainer.ply', [-0.89,2.3,0.9], pi/2);
Container2 = PlyObject('LavContainer.ply', [-0.89,1.75,0.9], pi/2);
Container3 = PlyObject('GrnContainer.ply', [0.8,2,0.9], 3/2*pi);
Container4 = PlyObject('BlkContainer.ply', [0.8,2.55,0.9], 3/2*pi);
Light_Curtain1 = PlyObject('Light_curtain_pair.ply', [1.3,1.75,-0.1], pi);
Light_Curtain2 = PlyObject('Light_curtain_pair2.ply', [0.6,0.4,-0.1], pi/2);


% Set view
%camlight
axis([-3 3 -2 4 -0.01 2])
view(45, 45)

% Render floor as image
img = imread('floor.jpeg');     
xImage = [2 2; -2 -2];              % The x data for the image corners
yImage = [-2 3.5; -2 3.5];              % The y data for the image corners
zImage = [0 0; 0 0];                % The z data for the image corners
surf(xImage,yImage,zImage,...       % Plot the surface
     'CData',img,...
     'FaceColor','texturemap');
 
 % Render background as image
img = imread('mall.jpeg');     
xImage = [2 -2; -2 2];              % The x data for the image corners
yImage = [-2 -2; -2 -2];              % The y data for the image corners
zImage = [1.5 1.5; 0 0];                % The z data for the image corners
surf(xImage,yImage,zImage,...       % Plot the surface
     'CData',img,...
     'FaceColor','texturemap');
 
 
% Collision Test
q = zeros(1,2);%use this instead
scale = 0.5; 
workspace = [-2 2 -2 2 -0.1 2];                                       % Set the size of the workspace when drawing the robot
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
%BREACH_TESTING;
p2Collide; %USE THIS ALTERNATIVE METHOD





