classdef PlyObject < handle
    properties
        vertices;
        mesh_h;
        pose;
    end
    methods
        % xyz = [ x y z ] input
        function self = PlyObject(strName, coor, rad)
            self.mesh_h = PlaceObject(strName, [0 0 0]);
            self.vertices = get(self.mesh_h,'Vertices');        
            % transform point pose
            self.pose = transl(coor(1), coor(2), coor(3));
            % transform ply
            transformedVertices = [self.vertices,ones(size(self.vertices,1),1)] * (transl(coor(1), coor(2), coor(3))* trotz(rad))';
            set(self.mesh_h,'Vertices',transformedVertices(:,1:3));
        end
        % transMatrix = standard 4by4 transformation matrix 
        function trObject(self, tr)
             self.pose = self.pose * tr;
             transformedVertices = [self.vertices,ones(size(self.vertices,1),1)] * tr';
             set(self.mesh_h,'Vertices',transformedVertices(:,1:3));
        end
    end
end
