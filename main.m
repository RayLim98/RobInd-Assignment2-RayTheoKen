clear
clf
clc

wall = GlassWallY(2, 5, [0 -2]);
hold on
wall2 = CreateYWall(2, 5, [0 2]);
wall3 = CreateXWall(2, 5, [0 -2]);
wall4 = CreateXWall(2, 5, [0 2]);