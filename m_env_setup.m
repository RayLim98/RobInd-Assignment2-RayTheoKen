sBot = Sawyer(0,2,1);
hold on
axis([-2 2 -2 3.5 0 2])
sBot.model.teach

Table = PlyObject('Bar_EventsHire1.ply', [0,0,0], 0);

Container1 = PlyObject('Container.ply', [-0.89,2.3,0.9], pi/2);
Container2 = PlyObject('Container.ply', [-0.89,1.75,0.9], pi/2);
Container3 = PlyObject('Container.ply', [0.8,2,0.9], 3/2*pi);
Container4 = PlyObject('Container.ply', [0.8,2.55,0.9], 3/2*pi);

Bottle = PlyObject('Bottle.ply', [0,2.9,0.9], pi/2);