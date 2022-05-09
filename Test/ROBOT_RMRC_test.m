clc
clear
clf
display('Running Bot RMRC test: ')

qr = [0 0 0 0 -pi/2 pi/2 0]
% Gen assets
bot = Sawyer(0,0,0.2, true);
axis([-1 1 -1 1 -0.2 1])
hold on
bot.model.plot(qr, 'scale', 0.7);
bot.model.teach();

% Get current joint position and end effector
qCurrent = bot.model.getpos;
tr = bot.model.fkine(qCurrent);
x = tr(1:3,4);
theta0 = tr2rpy(tr)';

% Final Data
R = eye(4);
R(1:3,1:3) = tr(1:3, 1:3) * rotx(-pi/2);
trf = transl(-0.6,0.2,0.6) * R;
xf = trf(1:3,4);
thetaF =  tr2rpy(tr * trotx(-pi/2))';


% Plot data
goal1 = plot3(x(1),x(2),x(3), '*', 'Color', 'r');
goal2 = plot3(xf(1),xf(2),xf(3), '*', 'Color', 'b');

% End effector position
% qM = bot.genTrajRMRC(xf);
% bot.model.plot(qM);

%% RMRC
steps = 30;
deltaT = 0.05;

% Current Joint config
qCurrent = bot.model.getpos;
% Current pose
tr = bot.model.fkine(qCurrent);

% Interpolate from point to point
x1 = tr(1:3,4);
x = zeros(3,steps);
theta = zeros(3,steps);
qdot = zeros(steps,7);          % Array for joint velocities       
s = lspb(0,1,steps);                                 % Create interpolation scalar
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*xf;                  % Create trajectory in x-y plane
    theta(:,i) = theta0*(1-s(i)) + s(i)*thetaF;
end

% Gen trajectory
qM = nan(steps, 7);
qM(1,:) = bot.model.ikcon(tr,qCurrent);

for i = 1:steps-1
    T = bot.model.fkine(qM(i,:));
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);
    S = Rdot*Ra';
    lin_vel = (x(:,i+1) - x(:,i))/deltaT;
    ang_vel = [S(3,2);S(1,3);S(2,1)];
    xdot = [lin_vel; ang_vel]; 
    J = bot.model.jacob0(qM(i,:));              
    
    % Cal inv J for redundant machine
    invJ = J'*inv(J*J');
    % Calc joint speed
    qdot(i,:) = invJ*xdot;       
    for j = 1:7                                                             % Loop through joints 1 to 6
        if qM(i,j) + deltaT*qdot(i,j) < bot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qM(i,j) + deltaT*qdot(i,j) > bot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qM(i+1,:) =  qM(i,:) + deltaT*qdot(i,:);  
end


bot.model.plot(qM)
bot.model.teach
