function PlotTopView(sol,model)
figure()
mesh(model.X,model.Y,model.H);   % Plot the data
colormap pink;                 % Default color map.
set(gca, 'Position', [0 0 1 1]); % Fill the figure window.
axis equal vis3d on;             % Set aspect ratio and turn off axis.
shading interp;                  % Interpolate color across faces.
material dull;                   % Mountains aren't shiny.
camlight left;                   % Add a light over to the left somewhere.
lighting gouraud;                % Use decent lighting.
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
hold on


% Threats as cylinders
threats = model.threats;
threat_num = size(threats,1);

for i = 1:threat_num
    threat = threats(i,:);
    threat_x = threat(1);
    threat_y = threat(2);
    threat_z = max(max(model.H))+1;  % choose z to be the highest peak
    threat_radius = threat(4);

    for j=1:3 
    % Define circle parameters:
    % Make an array for all the angles:
    theta = linspace(0, 2 * pi, 2000);
    % Create the x and y locations at each angle:
    x = threat_radius * cos(theta) + threat_x;
    y = threat_radius * sin(theta) + threat_y;
    % Need to make a z value for every (x,y) pair:
    z = zeros(1, numel(x)) + threat_z;
    % Do the plot:
    % First plot the center:
    plot3(threat_x, threat_y, threat_z, 'o', 'color', 'red', 'MarkerSize', 3, 'MarkerFaceColor','red');
    % Next plot the circle:
    plot3(x, y, z, '-', 'color', 'red', 'LineWidth', 1);

    % Repeat for a smaller radius
    threat_radius = threat_radius - 10;
    end
end

p = [];
% Plot paths
for i = size(sol, 1):-1:1
    p(i) = plot3(sol(i).path(:,1), sol(i).path(:,2), sol(i).path(:,3),...
        '-', 'LineWidth', 2, 'DisplayName', ['UAV', num2str(i)]);
end

% plot target
all_marks = {'s','d','h','>','^','v','o','p','<'};
for j = 1:size(sol, 1)
    k = size(sol(j).target,1);
    scatter3(sol(j).target(1:k-1,1),...
            sol(j).target(1:k-1,2),...
            sol(j).target(1:k-1,3),...
            40,'k','fill', 'Marker', all_marks{mod(j,9)});
    
    % Final location
    xf=sol(j).target(k,1);
    yf=sol(j).target(k,2);
    zf=sol(j).target(k,3);
    p(size(sol,1)+1) = plot3(xf,yf,zf,'bo','MarkerSize',8,...
                            'MarkerFaceColor','b',...
                            'DisplayName', 'Ad-hoc node');
end

% Start location
xs=model.start(1);
ys=model.start(2);
zs=model.start(3);

p(size(sol,1)+2)=plot3(xs,ys,zs,'bs','MarkerSize',10,...
                        'MarkerFaceColor','b', 'DisplayName', 'Base station');

% Start location
xf=model.goal(1);
yf=model.goal(2);
zf=model.goal(3);
p(size(sol,1)+3)=plot3(xf,yf,zf,'rp','MarkerSize',10,...
                        'MarkerFaceColor','r', 'DisplayName', 'Target');

legend(p,'Location', 'best');
% title('Search and Rescue');
view([0 90]);
hold off;
end