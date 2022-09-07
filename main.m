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

VarMax.sen = 50;    % sensing range
VarMax.com = 300;   % communication range
% restriction of searching angle
VarMax.alpha = pi/4;
VarMin.alpha = -VarMax.alpha;

uavs = [];  % Init the empty uavs list
% start position
start = model.start;
goal = start;
i = 0;
is_reach = false;
%% Searching
% for i = 1:size(model.goals,1)+1 
while ~is_reach
    i = i+1;
    drone = Drone(i, start, 0, 1.0);    % init uav
    uavs = [uavs; drone];               % Store results
    
    drone = drone.multi_target_tracking(model,uavs,dt,VarMax,VarMin);
    if norm(drone.position - model.goal) < 5
        is_reach = true;
    end
end

% figure();
% drone.draw_drone();
% PlotHistogram(uavs, model)
save('drone.mat', 'uavs');
% Plot3DView(uavs, model);
PlotTopView(uavs, model);
PlotIteration(uavs);
ViewResults(uavs, model);