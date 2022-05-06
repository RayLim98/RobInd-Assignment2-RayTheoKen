classdef Sawyer < handle
    properties
        model;
        workspace = [-2 2 -2 2 0 2];   
        useGripper = false;
        CanOperate = true;
        % Ready Pose
        qr = [0 -pi -pi/2 pi/2 -pi/2 -pi/2 0];
        % Base position         
        x = 0;
        y = 0;
        z = 0;
    end
    
    methods
        %% Class for UR5 robot simulation
        function self = Sawyer(x, y, z)
            self.x = x;
            self.y = y;
            self.z = z;
            self.GetSawyerRobot();
            self.PlotAndColourRobot();
        end

        %% GetUR3dRobot
        function GetSawyerRobot(self)
        %     if nargin < 1
                % Create a unique name (ms timestamp after 1ms pause)
                pause(0.001);
                name = ['SawyerBot',datestr(now,'yyyymmddTHHMMSSFFF')];
        %     end
%         qr = [0 -pi -pi/2 pi/2 -pi/2 0 0]
            L(1) = Link('d',0.237,  'a',0.081,   'alpha',-1.571, 'offset',0, 'qlim',[-175 175]*pi/180);
            L(2) = Link('d',0.1925, 'a',0,       'alpha',-1.571, 'offset',0, 'qlim',[-175 175]*pi/180);    
            L(3) = Link('d',0.4,    'a',0,       'alpha',-1.571, 'offset',0, 'qlim',[-175 175]*pi/180);
            L(4) = Link('d',-0.1685,'a',0,       'alpha',-1.571, 'offset',0, 'qlim',[-170 170]*pi/180);
            L(5) = Link('d',0.4,    'a',0,       'alpha',-1.571, 'offset',0, 'qlim',[-170 170]*pi/180);
            L(6) = Link('d',0.1363, 'a',0,       'alpha',-1.571, 'offset',0, 'qlim',[-170 170]*pi/180);
            L(7) = Link('d',0.11,   'a',0,       'alpha',0,      'offset',0, 'qlim',[-270 270]*pi/180);
        
%             L(1) = Link('d',0.317,  'a',0.081,  'alpha', -1.571,  'offset',0);
%             L(2) = Link('d',0.1925, 'a',0,      'alpha',-1.571,     'offset',0);    
%             L(3) = Link('d',0.4,    'a',0,        'alpha',-1.571,     'offset',0);
%             L(4) = Link('d',0.1685,'a',0,       'alpha',-1.571,  'offset',0);
%             L(5) = Link('d',0.4,    'a',0,       'alpha',-1.571, 'offset',0);
%             L(6) = Link('d',0.1363, 'a',0,       'alpha',-1.571,     'offset',0);
%             L(7) = Link('d',0.13375,   'a',0,       'alpha',0,     'offset',0);

            self.model = SerialLink(L,'name',name);
            self.model.base = self.model.base * transl(self.x, self.y, self.z);
        end
        %% PlotAndColourRobot
        function PlotAndColourRobot(self)%robot,workspace)
            tic
            display('Coloring Robot Sequence started')
            for linkIndex = 0:self.model.n
                display(['Getting Link Data ', num2str(linkIndex + 1)])      
                if self.useGripper && linkIndex == self.model.n
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['sawyer_part_',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
                else
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['sawyer_part_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                end
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
                display(['Acquired face and vertex data for Link ', num2str(linkIndex + 1),'...',num2str(toc)]);
                display(['Facedata size: ', num2str(size(faceData))...
                    ,' ,Vertexx size: ', num2str(size(vertexData))])
            end
            self.model.plot3d(zeros(1,self.model.n),'workspace',self.workspace);
%             self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);

            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
            display(['Finished coloring ', num2str(toc), ' seconds']);
        end       
        %% Generate Trajectory
        function [qM] = genTraj(self,tr)
            mask = [1,1,1,0,0,0];
            q0 = self.model.getpos();
            qf = self.model.ikine(tr,q0,mask);
            qM = jtraj(q0,qf,50);
        end
        %% Generate Trajectory RMRC
        function [qMatrix] = genTrajRMRC(self, xf)
            qCurrent = self.model.getpos;
            trCurrent = self.model.fkine(qCurrent);
            x0 = trCurrent(1:3,4);
            display('Generating RMRC Trajectory')
            t = 3;              % Total time (s)
            deltaT = 0.05;      % Control frequency
            steps = t/deltaT;   % No. of steps for simulation
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.2;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
            
            % 1.2) Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,7);       % Array for joint anglesR
            qdot = zeros(steps,7);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory
%             positionError = zeros(3,steps); % For plotting trajectory error
%             angleError = zeros(3,steps);    % For plotting trajectory error
            
            % 1.3) Set up trajectory, initial pose
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i=1:steps
                x(1,i) = (1-s(i))*x0(1) + s(i)*xf(1); % Points in x
                x(2,i) = (1-s(i))*x0(2) + s(i)*xf(2); % Points in y
                x(3,i) = (1-s(i))*x0(3) + s(i)*xf(3); % Points in z
                theta(1,i) = 0;                % Roll angle 
                theta(2,i) = 0;                % Pitch angle
                theta(3,i) = 0;                 % Yaw angle
            end
            % set current joint position as the first row
            q0 = self.model.getpos();                                                     % Initial guess for joint angles
            T = self.model.fkine(q0);
            qMatrix(1,:) = self.model.ikcon(T,q0);                                        % Solve joint angles to achieve first waypoint            
            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                T = self.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                     % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                             % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = self.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)           % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
%                 positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
%                 angleError(:,i) = deltaTheta;                                          % For plotting
            end
        end

         %% Default position
         function resetJoints(self)
            self.model.plot(self.qr);
        end
    end
end