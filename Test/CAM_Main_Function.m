%% Initialise CAM Function

clear
clf
clc


L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);

robot = SerialLink([L1 L2 ],'name','myRobot');                     
q = zeros(1,2);                                                     % Create a vector of initial joint angles        
scale = 0.5;
workspace = [-4 4 -4 4 -0.05 3];                                       % Set the size of the workspace when drawing the robot
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot

%% Create Light Curtain Wall

wall = LightCurtainWall(1, 2, [0 1.9 -0.5]);
hold on


%% Creating a cube and use teach to see collision detection
centerpnt = [2,0,-0.5];
side = 1.5;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal
camlight

    pPoints = [1.25,0,-0.5 ...
        ;2,0.75,-0.5 ...
        ;2,-0.75,-0.5 ...
        ;2.75,0,-0.5];
    pNormals = [-1,0,0 ...
            ; 0,1,0 ...
            ; 0,-1,0 ...
            ;1,0,0];

robot.teach;
hold on


%% IsCollision

% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end




% % % % Get the transform of every joint (i.e. start and end of every link)
% % % tr = zeros(4,4,robot.n+1);
% % % tr(:,:,1) = robot.base;
% % % L = robot.links;
% % % for i = 1 : robot.n
% % %     tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
% % % end
% % % 
% % % %Go through each link and each triangle face
% % % for i = 1 : size(tr,2)-1    
% % %     for faceIndex = 1:size(faces,1)
% % %         vertOnPlane = vertex(faces(faceIndex,1)',:);
% % %         [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
% % %         if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
% % %             plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
% % %             display('Intersection');
% % %         end
% % %     end    
% % % end
% % % 
% % % %Go through each steps 
% % % q1 = [-pi/4,0,0];
% % % q2 = [pi/4,0,0];
% % % steps = 10;
% % % while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
% % %     steps = steps + 1;
% % % end
% % % qMatrix = jtraj(q1,q2,steps);


%% Confirm Collision

%check each joint states in the trajectory to work out which ones are in
%collisions
% 0 = no collision
% 1 = yes collision (unsafe)

result = true(steps,1)
for i = 1:steps
    result(i) = CollisionCheck(robot,q1,q2);
end

