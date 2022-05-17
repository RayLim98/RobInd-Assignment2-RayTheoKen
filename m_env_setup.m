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
Light_Curtain = PlyObject('Light_curtain_pair.ply', [1.3,1.75,-0.1], pi);

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
xImage = [2 2; -2 -2];              % The x data for the image corners
yImage = [-0.5 -0.5; -0.5 -0.5];              % The y data for the image corners
zImage = [0 1.5; 0 1.5];                % The z data for the image corners
surf(xImage,yImage,zImage,...       % Plot the surface
     'CData',img,...
     'FaceColor','texturemap');
 
 
% Establish workspace cube

%//////////////////////////////////////
a1 = 0.4; %Link 1 length 
a2 = 0.4; %Link 2 length
p2 = SerialLink([
    Revolute('d', 0, 'a', a1, 'alpha', 0, 'standard')
    Revolute('d', 0, 'a', a2, 'alpha', 0, 'standard')
    ], ...
    'name', 'two link'); 
% % % % % L1 = Link('d', 0, 'a', a1, 'alpha', 0,'offset',0.5,'qlim',[deg2rad(-360),deg2rad(360)]);
% % % % % L2 = Link('d', 0, 'a', a2, 'alpha', 0,'offset',2,'qlim',[deg2rad(-90),deg2rad(90)]);
% % % % % % p2Bot = p2(1,1,1); %generate sawyer model 
% % % hold on
% % % p2Bot = p2;
% % % p2Bot = [0,0,0];
q = zeros(1,2);%use this instead
scale = 0.5; 
workspace = [-2 2 -2 2 -0.1 2];                                       % Set the size of the workspace when drawing the robot
p2.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
p2.teach;
%/////////////////////////////////////////////////

% % % % % mdl_planar2; 
% % % % % hold on
% % % % % p2.base = p2.base * transl(2,0,0);
% % % % % hold on
BREACH_TESTING;
%dlc = DigitalLightCurtain();
p2Collide; %USE THIS ALTERNATIVE METHOD


