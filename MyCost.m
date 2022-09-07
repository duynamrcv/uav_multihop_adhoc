%
% Calculate path cost
%

function cost=MyCost(sol,start,currentState,target,model,varmax,varmin)
%MyCost(x,currentState,target,model,VarMax); 
    
    J_inf = inf;
    H = model.H; % H is the map
    
    % Input solution
    x=sol.x;
    y=sol.y;
    z=sol.z;
    
    % Altitude wrt sea level = z_relative + ground_level
    z_abs = z + H(round(y),round(x));
    

    %==============================================
    %% F1 - Obstacles avoidance  

    % Threats/Obstacles
    threats = currentState.Threats;
    threat_num = size(threats,2);
    
    drone_size = 1.0;
    offset = 10.0;
    danger_dist = 10*drone_size;
    
    F1 = 0;
    for i = 1:threat_num
        threat_x = threats(i).x;
        threat_y = threats(i).y;
        threat_radius = threats(i).radius;
        dist = norm([x y]-[threat_x threat_y]);
        if (dist>=threat_radius + drone_size + danger_dist + offset)
            threat_cost = 0;
        elseif dist <= (threat_radius + drone_size + offset)  % Collision
            threat_cost = J_inf;
        else  % danger
            threat_cost = (threat_radius + drone_size + danger_dist + offset) - dist;
        end

        F1 = F1 + threat_cost;
    end

    %==============================================
    %% F2 - Angle cost
%     anglemax = varmax.alpha;
    
    currentState.Position.zabs = currentState.Position.z + H(round(currentState.Position.y),round(currentState.Position.x));
    target.zabs = target.z + H(round(target.y),round(target.x));
    
    %P(ij)P(i,j+1)
    segment1 = [x; y; z_abs] - [currentState.Position.x; currentState.Position.y; currentState.Position.z];

    % P(ij)E

    segment2 = [target.x; target.y; target.zabs] - [currentState.Position.x; currentState.Position.y; currentState.Position.zabs];

    angle = atan2(norm(cross(segment1,segment2)),dot(segment1,segment2));

%     F2 = 180*abs(angle);
% %     if abs(angle) < varmax.alpha
        F2 = 180*abs(angle)/pi;
%     else
%         F2 = J_inf;
%     end

    %==============================================
    %% F3 farthest in sensing range 
    r = norm(segment1);
    F3 = varmax.sen - r;
    if F3 < 0
        F3 = J_inf;
    end

    %==============================================
    %% F4 farthest in communication range
    segment3 = [x; y; z_abs] - start';
    r = norm(segment3);
    F4 = varmax.com - r;
    
    if F4 < 0
        F4 = J_inf;
    end
    
    %============================================
    % Weight coeffcients
    b1 = 10;
    b2 = 1;
    b3 = 1;
    b4 = 1;
    % Overall cost
    cost = b1*F1 + b2*F2 + b3*F3 + b4*F4;
end