function [newWall] = CreateXWall(width, height, location)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    [y, z] = meshgrid((-width + location(1)):0.2:(width + location(1)), 0:0.2:height);
    x = zeros(size(y)) + location(2);
    T = delaunay(y,z);
    % Create new wall
    s = trisurf(T,x,y,z);
    vertex = s.Vertices;
    faces = s.Faces;
    % need to create face normals
    faceNormals = zeros(size(faces,1),3);
    for faceIndex = 1:size(faces,1)
        v1 = vertex(faces(faceIndex,1)',:);
        v2 = vertex(faces(faceIndex,2)',:);
        v3 = vertex(faces(faceIndex,3)',:);
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end
    s.FaceNormals = faceNormals;
    newWall = s;
end

