function PlotHistogram(uavs, model)
CostMap = -ones(size(model.H))*100;

for i = 1:size(uavs,1)
    drone = uavs(i);
    
    length = size(drone.best_cost,2);
    if length ~= 0
        for j = 1:length
            x = round(drone.best_cost(j).Position.x);
            y = round(drone.best_cost(j).Position.y);
            CostMap(x,y) = drone.best_cost(j).Cost;
        end
    end
end

% Plot Histogram map
mesh(model.X, model.Y, CostMap);
colormap summer;
set(gca, 'Position', [0 0 1 1]); % Fill the figure window.
axis equal vis3d on;             % Set aspect ratio and turn off axis.
view([0 90]);
end