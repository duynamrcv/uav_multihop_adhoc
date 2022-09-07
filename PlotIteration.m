function PlotIteration(uavs)
figure();

for i = 1:size(uavs, 1)
    p = [];
    subplot(ceil(size(uavs, 1)/2),2,i);
    grid on;
    hold on;
    for j = 1:size(uavs(i).best_cost, 1)
        p(j) = plot(uavs(i).best_cost(j,:),...
            'LineWidth', 2, 'DisplayName', [num2str(j)]);
    end
    xlabel('Number of Iterations');
    ylabel('Fitness value');
    xlim([0,150])
    title(['UAV',num2str(i)]);
    lgd = legend(p);
%     lgd.NumColumns = 2;
end


end