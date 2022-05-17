function [collided] = CollisionExist(p2,qM, vertex,faces,faceNormals)
%COLLISIONEXIST Summary of this function goes here
%   Detailed explanation goes here
    collided = false; %start with initial no collision
    for qIdx = 1: size(qM,1)    %creating a matrix for all q values of the robot
        q = qM(qIdx,:);         % 
        tr(:,:,:,1) = p2.base;
        %Getting the transform of every link
        for i = 1: p2.n  %allocating an index for the number of link 
            link = p2.links(i);  %declaring the number of links
            tr(:,:,:, i+1) = tr(:,:,:,i) * trotz(q(i)+link.offset) * transl(0,0,link.d) * transl(link.a,0,0) * trotx(link.alpha); %scrolling through each links available
        end 
        for linkIndex = 1: p2.n
            for faceIndex = 1:size(faces,1)
                if collided == true
                    break
                end
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,linkIndex)',tr(1:3,4,linkIndex+1)'); 
                if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                    plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                    display('Intersection');
                    collided = true;
                    p2.plot(q)
                end
            end
        end
    end
end

