function PlotIteration(uavs)
figure();
grid on;
hold on;
p = [];
for i = 1:size(uavs, 1)
    iter = size(uavs(i).best_cost, 2);
    
    cost = [];
    if iter ~= 0
        for j = 1:iter
            cost = [cost uavs(i).best_cost(j).Cost];
        end
        if i == 1
            p(i) = plot(cost,...
                'LineWidth', 2, 'DisplayName', 'Base station');
        else
            p(i) = plot(cost,...
                'LineWidth', 2, 'DisplayName', ['UAV', num2str(i-1)]);
        end
    end
end
xlabel('Number of Iterations');
ylabel('Fitness value');
legend(p,'Location', 'best');
hold off;
end