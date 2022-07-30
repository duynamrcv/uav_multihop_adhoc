% get map data from current position

function obstacles = GetMap(point,model,range)
    
    obstacles = [];

    threats = model.threats;
    threat_num = size(threats,1);
    
    for i=1:threat_num
        threat.x = threats(i,1);
        threat.y = threats(i,2);
        threat.z = threats(i,3);
        threat.radius = threats(i,4);
        
%         segment = [point.x; point.y; point.z]-[threat.x; threat.y; threat.z];
        segment = [point.x; point.y]-[threat.x; threat.y];
        distance = norm(segment);
        if  distance<(range+threat.radius)
           obstacles = [obstacles threat];
        end
    end
    % return obstacles;
end