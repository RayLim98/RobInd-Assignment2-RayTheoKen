classdef DigitalLightCurtain
    %CAM Summary of this class goes here
    %Detailed explanation goes here
    
    properties
        vertex
        faces
        faceNormals
    end
    
    methods
        % Called when i'm created
        function self = DigitalLightCurtain()
            centerpnt = [0,2.6,0.5];
            side = 1.7;
            plotOptions.plotFaces = true; 
            [vertex,faces,faceNormals] = WorkSpaceCube(centerpnt-side/2, centerpnt+side/2,plotOptions);
            self.vertex = vertex;
            self.faces = faces;
            self.faceNormals = faceNormals;
        end
        
        function isBreached = isBreached(self, p2, qM)
            state = CollisionExist(p2, qM, self.vertex, self.faces, self.faceNormals) == true;
            if ~state
                display('UR GOOD')
            else
                display('Breached')
            end
            isBreached = state;
        end
    end
end

