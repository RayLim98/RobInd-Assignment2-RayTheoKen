%% Testing mdl_planar2 colliding workspace
function [  ] = p2SPawn( )



%clf
%close all;


a1 = 1.0; %Link 1 length 
a2 = 0.8; %Link 2 length


p2 = SerialLink([
    Revolute('d', 0, 'a', a1, 'alpha', 0, 'standard')
    Revolute('d', 0, 'a', a2, 'alpha', 0, 'standard')
    ], ...
    'name', 'two link'); 

q = zeros(1,2);%use this instead
scale = 0.5; 
workspace = [-2 2 -2 2 -0.1 2];                                       % Set the size of the workspace when drawing the robot
p2.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
%%
% % % % qz = [0 0];

% 2.1: Make a 2DOF model
% % % L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
% % % L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
% % % 
% % % robot = SerialLink([L1 L2 ],'name','myRobot');                     
% % % q = zeros(1,2);                                                     % Create a vector of initial joint angles        
% % % scale = 0.5;
% % % workspace = [-2 2 -2 2 -0.2 2];                                       % Set the size of the workspace when drawing the robot
% % % robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot

% 2.2 and 2.3
centerpnt = [0,2,0.5];
side = 2.5;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = WorkSpaceCube(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal
%camlight


% 2.4: Get the transform of every joint (i.e. start and end of every link)
tr = zeros(4,4,p2.n+1);
tr(:,:,1) = p2.base;
L = p2.links;
for i = 1 : p2.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

% 2.5: Go through each link and also each triangle face
for i = 1 : size(tr,3)-1    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = vertex(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            display('Get Back Fool!');
        else
            display('Ur Good');
        end
    end    
end

%Go through until there are no step sizes larger than 1 degree
q1 = [0,0,0];
q2 = [pi/2,0,0];
steps = 100;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);

result = true(steps,1);
for i = 1: steps
    result(i) = IsCollision(p2,qMatrix(i,:),faces,vertex,faceNormals,false);
    p2.animate(qMatrix(i,:));
end

% p2.teach; %UNCOMMENT THIS TO MANUALLY CONTROL
end


%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(p2,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), p2);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Workplace Breach!');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, p2)

links = p2.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = p2.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end

%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end