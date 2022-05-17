            SawyerRobot = Sawyer(0,2,1);
            xf = [-0.6, 2.4, 1]';
            steps = 30;
            deltaT = 0.05;
            % Current Joint config
            % Current pose
            qCurrent = SawyerRobot.model.getpos()
            tr = SawyerRobot.model.fkine(qCurrent)

            % Interpolate from point to point
            x1 = tr(1:3,4)
            x = zeros(3,steps)
            qdot = zeros(steps,7)                                          % Array for joint velocities       
            s = lspb(0,1,steps)                                            % Create interpolation scalar
            for i = 1:steps
                x(:,i) = x1*(1-s(i)) + s(i)*xf                             % Create trajectory in x-y plane
            end

            % Gen trajectory
            qM = nan(steps, 7)
            qM(1,:) = SawyerRobot.model.ikcon(tr,qCurrent)
            
            for i = 1:steps-1
                lin_vel = (x(:,i+1) - x(:,i))/deltaT
                ang_vel = [0;0;0];
                xdot = [lin_vel; ang_vel] 
                J = SawyerRobot.model.jacob0(qM(i,:))              
                
                % Cal inv J for redundant machine
                invJ = J'*inv(J*J')
                % Calc joint speed
                qdot(i,:) = invJ*xdot       
                for j = 1:7                                                 % Loop through joints 1 to 6
                    if qM(i,j) + deltaT*qdot(i,j) < SawyerRobot.model.qlim(j,1)     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0;                                      % Stop the motor
                    elseif qM(i,j) + deltaT*qdot(i,j) > ...
                        SawyerRobot.model.qlim(j,2) % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0;                                      % Stop the motor
                    end
                end
                qM(i+1,:) =  qM(i,:) + deltaT*qdot(i,:);  
            end