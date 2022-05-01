%% Initialise CAM Function

clear
clf
clc

bot = Sawyer(0,0,0);
bot.model.teach();
hold on

%% Initialise Robot 
% 2.1: Make Sawyer model 
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]); 
L4 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]); 
L5 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]); 
L6 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]); 
robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','Robot');                     
q = zeros(1,6);                                                     % Create a vector of initial joint angles        
scale = 0.5;
workspace = [-2 2 -2 2 -0.05 2];                                       % Set the size of the workspace when drawing the robot
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
        
%% Class for Workspace Interaction as an Object
        classdef  WorkspaceObjectClass
            properties
%            assign x of mesh
%            assign y of mesh
%            assign z of mesh
%            assign Pp arbitrary of plane for detection (check line method)
%   Wall_ = newWall_1;
%     Wall_2 = newWall_2(X,Y);
%       Wall_3 = newWall_3(X,Y);
%         Wall_4 = newWall_4(X,Y);
            end      
            
         
%% CAM Object Class

shopWallIntersect = shopWallInt;

q; %joint states

% % %% Robot set up robot
% % q0 = bot.model.getpos();
% % qr = [0 -pi -pi/2 pi/2 0 -pi/2 0];
% % 
% % qM = jtraj(q0,qr,30);
% % 
% % bot.model.plot(qM);
% % q0 = bot.model.getpos();
% % bot.model.teach();

%% Test to see line collision with wall 
% [This section will return with a boolean expression if there is a
% collision with end-effector]

% (forward kinematic for for each joint)
% From Lab 5 
% % % % 2.4: Get the transform of every joint (i.e. start and end of every link)
% % % tr = zeros(4,4,robot.n+1);
% % % tr(:,:,1) = robot.base;
% % % L = robot.links;
% % % for i = 1 : robot.n
% % %     tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
% % % end

function [tr] = GetLinkPointMatrixes(robot)
%GETLINKLINEMATRIXS 
    tr(:,:,:,1) = robot.base;
    for i = 1: robot.n
        link = robot.links(i);
        tr(:,:,:, i+1) = tr(:,:,:,i) * trotz(q(i)+link.offset) * transl(0,0,link.d) * transl(link.a,0,0) * trotx(link.alpha);
    end
end


%Tr(1-7) for 6 joints including one transformation from base;
 









