clc;
clear;
close all;

%% Problem definition
model = CreateModel1();  % Environment model
dt = 0.05;               % Time step

% Lower and upper Bounds of particles (Variables)
VarMin.x=model.xmin;           
VarMax.x=model.xmax;           
VarMin.y=model.ymin;           
VarMax.y=model.ymax;           
VarMin.z=model.zmin;           
VarMax.z=model.zmax;  

VarMax.r = 300; % sensing range
VarMin.r = 200;
% restriction of searching angle
VarMax.alpha = pi/4;

uavs = [];  % Init the empty uavs list
% start position
start = model.start;
goal = start;
i = 1;
%% Searching
% for i = 1:size(model.goals,1)+1 
while goal ~= model.goal
    if i == 1
        goal = start + [0,0,30];
    else
        % goal position
        % goal = model.goals(i-1,:);
        goal = uavs(i-1).next_target;
    end
    disp(['Next target: ', num2str(goal)])
    
    % init uav
    drone = Drone(i, start, 0, 1.0, goal);
    uavs = [uavs; drone];
    drone = drone.multi_target_tracking(model,uavs,dt,VarMax,VarMin);
    i = i+1;
end
% 
% start = model.start;
% start(3) = start(3) + model.H(round(start(2)),round(start(1)));
% goal = model.goals(1,:);
% goal(3) = goal(3)+ model.H(round(goal(2)),round(goal(1)));
% 
% drone = Drone(1, start, 0, 2.0, goal);
% drone.nearest_target = drone.target;
% figure();
% drone.draw_drone();

% while norm(drone.position - drone.target) > 0.1
%     
%     vel = drone.control_signal(model, []);
%     drone.update_position(vel, dt);
% 
% end


% figure();
% drone.draw_drone();
% PlotHistogram(uavs, model)
% save('drone.mat', 'uavs');
% Plot3DView(uavs, model);
PlotTopView(uavs, model);
PlotIteration(uavs);
ViewResults(uavs, model);