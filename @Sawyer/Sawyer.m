classdef Sawyer < handle
    properties
        model;
        workspace = [-2 2 -2 2 0 2];   
        useGripper = false;
        x = 0;
        y = 0;
        z = 0;
    end
    
    methods%% Class for UR5 robot simulation
        function self = Sawyer(x, y, z)
            self.x = x;
            self.y = y;
            self.z = z;
            self.GetSawyerRobot();
%             self.PlotAndColourRobot();%
        end

%% GetUR3dRobot
        function GetSawyerRobot(self)
        %     if nargin < 1
                % Create a unique name (ms timestamp after 1ms pause)
                pause(0.001);
                name = ['SawyerBot',datestr(now,'yyyymmddTHHMMSSFFF')];
        %     end
%         qr = [0 -pi -pi/2 pi/2 -pi/2 0 0]
            L(1) = Link('d',0.237,  'a',0.081,  'alpha', -1.571,  'offset',0);
            L(2) = Link('d',0.1925, 'a',0,      'alpha',-1.571,     'offset',0);    
            L(3) = Link('d',0.4,    'a',0,        'alpha',-1.571,     'offset',0);
            L(4) = Link('d',-0.1685,'a',0,       'alpha',-1.571,  'offset',0);
            L(5) = Link('d',0.4,    'a',0,       'alpha',-1.571, 'offset',0);
            L(6) = Link('d',0.1363, 'a',0,       'alpha',-1.571,     'offset',0);
            L(7) = Link('d',0.11,   'a',0,       'alpha',0,     'offset',0);
        
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
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
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
    end
end