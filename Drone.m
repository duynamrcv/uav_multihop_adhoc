classdef Drone < handle
    %% Drone properties
    properties
       com_range = 300.0;
       sen_range = 50.0;
       
       id
       connect_id

       distance
       time
       
       position
       heading
       radius
       path
       
       best_cost = []
       
       state    % 0 = unassigned, 1 = assigned, 2 = occupied
       
       target = []      % store the PSO target
       nearest_target   % the target follow
       next_target      % the next target to assign new uav
    end
    
    %% Control parameters
    properties
        % Moving to goal parameters
        a_m2g = 30.0;
        b_m2g = 20.0;
        
        % Avoiding threats parameters
        a_ath = 30.0;
        b_ath = 15.0;
        
        % Avoiding drones parameters
        a_adr = 30.0;
        b_adr = 20.0;
    end
    
    %% Methods
    methods
        % Constructor
        function obj = Drone(id, position, heading, radius)
            obj.id = id;
            obj.connect_id = 1;
            obj.position = position;
            obj.heading = heading;
            obj.radius = radius;
            obj.path = [position];
            
            obj.state = 0;
            
            obj.distance = 0;
            obj.time = 0;
        end
        
        % Search next target
        function obj = search_next_target(obj,model,start,VarMax,VarMin)
            % Run PSO
            disp('Searching ...');
            tic();
            % obj.next_target = model.goals(obj.id,:);
            if norm(obj.position - model.goal) > obj.sen_range
                currentState.Position.x = obj.position(1);
                currentState.Position.y = obj.position(2);
                currentState.Position.z = obj.position(3);

                currentState.Threats = GetMap(currentState.Position,model,obj.sen_range);
                [obj.next_target, cost] = PSO(start,currentState,model,VarMax,VarMin);
                obj.best_cost = [obj.best_cost;cost];
                obj.next_target(3) = obj.next_target(3) + model.H(round(obj.next_target(2)),...
                                                                round(obj.next_target(1)));
            else
                obj.next_target = model.goal;
            end
            obj.target = [obj.target; obj.next_target];
            obj.time = obj.time + toc();
        end
        
        % Search nearest target
        function obj = search_nearest_target(obj, uavs)
            obj.nearest_target = uavs(obj.connect_id).target;
        end
        
        % Control signal
        function vel = control_signal(obj, model, uavs, target)
            % Move to goal
            d_m2g = norm(target - obj.position);
            v_m2g = (target - obj.position)/d_m2g;
            f_m2g = obj.a_m2g;
            if d_m2g <= obj.b_m2g
                f_m2g = obj.a_m2g*d_m2g/obj.b_m2g;
            end
            vel = f_m2g*v_m2g;
            
            % Avoid threats
            for i = 1: size(model.threats, 1)
                d_ath = norm(model.threats(i,1:2) - obj.position(1:2))...
                      - model.threats(i,4) - obj.radius;
                v_ath = (model.threats(i,1:2) - obj.position(1:2))/d_ath;
                
                sig = -sign(v_ath(1)*sin(obj.heading)-v_ath(2)*cos(obj.heading));
                rot = [0    -sig    0;...
                       sig  0       0;...
                       0    0       1];
                
                v_ath = [v_ath(1), v_ath(2), 0];
                f_ath = 0;
                if d_ath <= obj.b_ath
                    f_ath = obj.a_ath*(1-d_ath/obj.b_ath);
                end
                vel = vel + f_ath*v_ath*rot;
            end
            
            % Avoid UAVs
            for i = 1:size(uavs, 1)
                if obj.id ~= uavs(i).id
                    d_adr = norm(uavs(i).position - obj.position)...
                          - uavs(i).radius - obj.radius;
                    v_adr = (uavs(i).position - obj.position)/d_adr;

                    f_adr = 0;
                    if d_adr <= obj.b_adr
                        f_adr = obj.a_adr*(1-d_adr/obj.b_adr);
                    end
                    vel = vel - f_adr*v_adr;
                end
            end
        end
        
        % Update position
        function obj = update_position(obj, vel, dt)
            obj.position = obj.position + vel*dt;   % Udpate position
            obj.heading = atan2(vel(2), vel(1));    % Update heading
            obj.path = [obj.path; obj.position];    % Restore path
            
%             obj.time = obj.time + dt;
            obj.distance = obj.distance + norm(vel*dt);
        end
        
        % Multi target tracking
        function obj = multi_target_tracking(obj,model,uavs,dt,VarMax,VarMin)
            id = 1;
            % Moving to farthest drone
            while id < obj.id
                disp(['Moveing to Drone ', num2str(id)]);
                while norm(obj.position - uavs(id).position) > 20
                    vel = obj.control_signal(model, uavs, uavs(id).position);
                    obj = obj.update_position(vel, dt);
                end
                id = id + 1;
            end
            
            % Get start adhoc node
            if id == 1
                start = model.start;
            else
                start = uavs(id-1).position;
            end
            
            while norm(obj.position - start) < obj.com_range - 10
                % Searching next temporary target
                obj = obj.search_next_target(model,start,VarMax,VarMin);
                % Moving to the next temporary target
                while norm(obj.position - obj.next_target) > 5
                    vel = obj.control_signal(model, uavs, obj.next_target);
                    obj = obj.update_position(vel, dt);
                end
                if all(obj.next_target == model.goal)
                    break;
                end
            end
            
        end
        
        % Draw function
        function draw_drone(obj)
            % Draw path
            plot3(obj.path(:,1), obj.path(:,2), obj.path(:,3), '-b', 'LineWidth', 2);
            
            % Draw drone
            t = 0:pi/12:2*pi;
            x = obj.position(1) + obj.radius*cos(t);
            y = obj.position(2) + obj.radius*sin(t);
            plot3(x, y, ones(size(t))*obj.position(3), '-r', 'LineWidth', 2);
        end
    end
end