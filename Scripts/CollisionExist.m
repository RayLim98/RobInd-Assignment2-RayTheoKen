function [collided] = CollisionExist(robot,qM, vertex,faces,faceNormals)
%COLLISIONEXIST Summary of this function goes here
%   Detailed explanation goes here
    collided = false;
    for qIdx = 1: size(qM,1)
        q = qM(qIdx,:);
        tr(:,:,:,1) = robot.base;
        for i = 1: robot.n
            link = robot.links(i);
            tr(:,:,:, i+1) = tr(:,:,:,i) * trotz(q(i)+link.offset) * transl(0,0,link.d) * transl(link.a,0,0) * trotx(link.alpha);
        end 
        for linkIndex = 1: robot.n
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
                    robot.plot(q)
                end
            end
        end
    end
end

