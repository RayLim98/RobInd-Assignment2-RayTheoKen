classdef Sawyer < handle
    properties
        model;
        workspace = [-2 2 -2 2 0 2];   

        % Bot state
        useGripper = false;
        canOperate = true;
        isHolding = false;

        % Positions
        % Manually found for waypoints
        qr = [0 0 0 0 pi/2 -pi/2 0];            % Readsy position
        qOp = [0 -3.0543 0 0.1187 0 1.7453 0];  % Operation Position
        qOpPosition1 = [1.9388   -3.0543   -1.2217    0.1187   -2.3504    1.4486   -0.0942]
        qOpPosition2 = [ -1.0996   -3.0543   -1.2217    0.1187   -2.3504    1.4486   -0.0942]
        qCup = [1.0385 -1.9437 0 1.5429 0 2.0320 0];      % Cup Position

        % Interpolation
        rStep = 5*pi/180 % radian steps
    end
    
    methods
        %% Class for UR5 robot simulation
        function self = Sawyer(x,y,z, test);
            tic;
            if nargin < 4
                self.GetSawyerRobot(x,y,z);
                self.PlotAndColourRobot();
            else
                if test == true
                    self.GetSawyerRobot(x,y,z);
                else
                   self.GetSawyerRobot(x,y,z);
                   self.PlotAndColourRobot();
                end
            end
            display(['Sawyer Robot rendered: ', 'Duration: ',num2str(toc)]);
            
        end

        %% GetUR3dRobot
        function GetSawyerRobot(self, x, y, z)
        %     if nargin < 1
                % Create a unique name (ms timestamp after 1ms pause)
                pause(0.001);
                name = ['SawyerBot',datestr(now,'yyyymmddTHHMMSSFFF')];
        %     end

            L(1) = Link('d',0.237,  'a',0.081,   'alpha',-1.571, 'offset',0, 'qlim',[-175 175]*pi/180);
            L(2) = Link('d',0.1925, 'a',0,       'alpha',-1.571, 'offset',0, 'qlim',[-175 175]*pi/180);    
            L(3) = Link('d',0.4,    'a',0,       'alpha',-1.571, 'offset',0, 'qlim',[-175 175]*pi/180);
            L(4) = Link('d',-0.1685,'a',0,       'alpha',-1.571, 'offset',0, 'qlim',[-170 170]*pi/180);
            L(5) = Link('d',0.4,    'a',0,       'alpha',-1.571, 'offset',0, 'qlim',[-170 170]*pi/180);
            L(6) = Link('d',0.1363, 'a',0,       'alpha',-1.571, 'offset',0, 'qlim',[-170 170]*pi/180);
            L(7) = Link('d',0.11,   'a',0,       'alpha',0,      'offset',0, 'qlim',[-270 270]*pi/180);

            self.model = SerialLink(L,'name',name);
            self.model.base = self.model.base * transl(x, y, z);
        end
        %% PlotAndColourRobot
        function PlotAndColourRobot(self) %robot,workspace)
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
            mask = [1,1,1,1,1,1];
            q0 = self.model.getpos();
            qf = self.model.ikcon(tr,q0,mask);
            qM = jtraj(q0,qf,50);
        end
        %% Generate Trajectory through final Joint config
        function [qM] = genTrajQ(self,qf)
            q0 = self.model.getpos();
            qM = jtraj(q0,qf,50);
        end
        %% Generate Trajectory RMRC
        function [qM] = GenTrajRMRC(self, xf)
            steps = 30;
            deltaT = 0.05;
            % Current Joint config
            % Current pose
            qCurrent = self.model.getpos;
            tr = self.model.fkine(qCurrent);

            % Interpolate from point to point
            x1 = tr(1:3,4);
            x = zeros(3,steps);
%             theta = zeros(3,steps);
            qdot = zeros(steps,7);                                          % Array for joint velocities       
            s = lspb(0,1,steps);                                            % Create interpolation scalar
            for i = 1:steps
                x(:,i) = x1*(1-s(i)) + s(i)*xf;                             % Create trajectory in x-y plane
%                 theta(:,i) = theta0*(1-s(i)) + s(i)*thetaF;
            end

            % Gen trajectory
            qM = nan(steps, 7);
            qM(1,:) = self.model.ikcon(tr,qCurrent);
            
            for i = 1:steps-1
%                 T = bot.model.fkine(qM(i,:));
%                 Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));         % Get next RPY angles, convert to rotation matrix
%                 Ra = T(1:3,1:3);                                            % Current end-effector rotation matrix
%                 Rdot = (1/deltaT)*(Rd - Ra);
%                 S = Rdot*Ra';
                lin_vel = (x(:,i+1) - x(:,i))/deltaT;
                ang_vel = [0;0;0];
                xdot = [lin_vel; ang_vel]; 
                J = self.model.jacob0(qM(i,:));              
                
                % Cal inv J for redundant machine
                invJ = J'*inv(J*J');
                % Calc joint speed
                qdot(i,:) = invJ*xdot;       
                for j = 1:7                                                 % Loop through joints 1 to 6
                    if qM(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0;                                      % Stop the motor
                    elseif qM(i,j) + deltaT*qdot(i,j) > ...
                        self.model.qlim(j,2) % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0;                                      % Stop the motor
                    end
                end
                qM(i+1,:) =  qM(i,:) + deltaT*qdot(i,:);  
            end
        end
        %% Go to ready pose 
        function [qM] = GoToReadyPose(self)
            display('Return to waiting position')
            qCurrent = self.model.getpos;
            qReady = self.qOp;    
            qReady(1) = qCurrent(1);
            qM = jtraj(qCurrent, qReady, 20);
        end  
        %% Go to Cup pose 
        function [qM] = GoToCupTrajectory(self)
            display('Reaching for cup')
            % q0 to take into account of the current pose
            q0 = self.model.getpos;
            
            % q1 to ready it for the working position
            q1 = self.qOp;
            % Value for bot to face foward
            q1(1)= 1.2828; 

            % q2 to get predetermined position of cup
            q2 = self.qCup;

            % Concatonate way points
            qWayPoints = [q0;q1;q2];
            
            % Generate way points
            qM = InterpolateWaypointsRadians(qWayPoints, self.rStep);

            % Check qM forCollision detection
            % TODO: Colision detection 

            % Commence trajectory
%             self.model.plot3d(qM)
        end  
        %% First order way point 
        function StartOrderTrajectory(self, object)
            tic
            display('Reaching for Cup')
            qM1 = self.GoToCupTrajectory();
            self.model.plot3d(qM1)

            display('Returning to home position')
            % Now holding Cup 
            qM2 = self.GoToReadyPose();
            self.AnimateTrajectoryWObject(qM2,object);

            display('Fufilling order 1, Premium pearl')
            qM3 = InterpolateWaypointsRadians([self.model.getpos;self.qOpPosition1], self.rStep);
            self.AnimateTrajectoryWObject(qM3, object);

            display('Adding stuff..')
            p1 = [-0.6, 2.4, 1]; % to be deleted 
            qM4 =  self.GenTrajRMRC(p1');
            self.AnimateTrajectoryWObject(qM4, object);
            pause(2);
            qM4Return = flip(qM4);
            self.AnimateTrajectoryWObject(qM4Return, object);
    
            p2 = [-0.6, 1.85, 1]; % to be deleted
            qM5 =  self.GenTrajRMRC(p2');
            self.AnimateTrajectoryWObject(qM5, object);
            pause(2);
            qM5Return = flip(qM5);
            self.AnimateTrajectoryWObject(qM5Return, object);
            display('Completed Order')

        end

        %% Animate 
        function AnimateTrajectoryWObject(self, qM, object)
            for i = 1: size(qM,1)
                % set current pose
                q = qM(i,:);
                tr = self.model.fkine(q);
                % animate
                self.model.animate(q);
                object.trObject(tr);
            end
        end
         %% Default position
         function resetJoints(self)
            self.model.plot(self.qr);
        end
    end
end