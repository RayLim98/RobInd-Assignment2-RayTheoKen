function [tr] = GetLinkPointMatrixes(robot)
%GETLINKLINEMATRIXS 
%   
    tr(:,:,:,1) = robot.base;
    for i = 1: robot.n
        link = robot.links(i);
        tr(:,:,:, i+1) = tr(:,:,:,i) * trotz(q(i)+link.offset) * transl(0,0,link.d) * transl(link.a,0,0) * trotx(link.alpha);
    end
end

