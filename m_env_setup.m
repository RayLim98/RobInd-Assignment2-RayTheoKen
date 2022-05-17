%  Render Objects
sBot = Sawyer(0,2,1);
hold on
Bottle = PlyObject('CupHeadLess.ply', [0,2.9,0.9], pi/2);
PlyObject('Lid.ply', [0,2.9,0.9], pi/2);

axis([-2 2 -2 3.5 0 2])
Table = PlyObject('BobaBar.ply', [0,0,0], 0);
Container1 = PlyObject('BrnContainer.ply', [-0.89,2.3,0.9], pi/2);
Container2 = PlyObject('LavContainer.ply', [-0.89,1.75,0.9], pi/2);
Container3 = PlyObject('GrnContainer.ply', [0.8,2,0.9], 3/2*pi);
Container4 = PlyObject('BlkContainer.ply', [0.8,2.55,0.9], 3/2*pi);

% Render floor as image
img = imread('floor.jpeg');     
xImage = [2 2; -2 -2];              % The x data for the image corners
yImage = [-2 3.5; -2 3.5];              % The y data for the image corners
zImage = [0 0; 0 0];                % The z data for the image corners
surf(xImage,yImage,zImage,...       % Plot the surface
     'CData',img,...
     'FaceColor','texturemap');
