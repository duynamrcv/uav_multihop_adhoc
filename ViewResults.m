function ViewResults(uavs, model) 
%% Number of threats
num_threat = size(model.threats, 1);
disp(['Number of threats: ', num2str(num_threat)]);

%% Number of UAVs
num_uavs = size(uavs,1);
disp(['Number of UAVs: ', num2str(num_uavs)]);

%% Distance
dis_ideal = norm(model.goal - model.start);

for j = 1:num_uavs
    disp(['The distance of uav ', num2str(j), ': ', num2str(uavs(j).distance)]);
    disp(['The PSO search time of uav ', num2str(j), ': ', num2str(uavs(j).time)]);
end

disp(['The ideal distance from start to goal: ', num2str(dis_ideal)]);

%% The ideal position of drone
start = model.start;
goal = model.goal;
for i = 1:num_uavs
    pos = start + (goal-start)/num_uavs*i;
    disp(['The ideal position of uav ', num2str(i) ': ', num2str(pos)]);
end

%% The actual position of drone
for i=1:num_uavs
    disp(['The actual position of UAV ', num2str(i) ': ', num2str(uavs(i).position);])
end

%% The ideal angle
angle = atan2(goal(2)-start(2), goal(1)-start(1));
disp(['The ideal angle is ', num2str(angle)]);

%% The actual angle
for i= 1:num_uavs
    if i == 1
        angle = atan2(uavs(i).position(2)-model.start(2),...
                    uavs(i).position(1)-model.start(1));
    else
        angle = atan2(uavs(i).position(2)-uavs(i-1).position(2),...
                        uavs(i).position(1)-uavs(i-1).position(1));
    end
    disp(['The actual angle of UAV ', num2str(i) ': ', num2str(angle);])
end

%% The link length
len = 0;
for i= 1:num_uavs
    if i == 1
        link = norm(uavs(i).position-model.start);
    else
        link = norm(uavs(i).position-uavs(i-1).position);
    end
    len = len + link;
    disp(['The actual link of UAV ', num2str(i) ': ', num2str(link)])
end
disp(['The total link of UAVs: ', num2str(len)]);
disp(['The ideal link of UAVs: ', num2str(norm(goal-start-[0,0,30]))]);
end