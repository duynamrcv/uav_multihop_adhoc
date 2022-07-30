function Plot3DView(sol,model)
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
h=250; % Height

for i = 1:threat_num
    threat = threats(i,:);
    threat_x = threat(1);
    threat_y = threat(2);
    threat_z = threat(3);
    threat_radius = threat(4);


    [xc,yc,zc]=cylinder(threat_radius); % create a unit cylinder
    % set the center and height 
    xc=xc+threat_x;  
    yc=yc+threat_y;
    zc=zc*h+threat_z;
    c = mesh(xc,yc,zc); % plot the cylinder 
    set(c,'Edgecolor','none','Facecolor','red','FaceAlpha',.3); % set color and transparency
end

% Start location
xs=model.start(1);
ys=model.start(2);
zs=model.start(3);

% plot target
for j = 1:size(sol, 1)
    % Final location
    xf=sol(j).target(1);
    yf=sol(j).target(2);
    zf=sol(j).target(3);
   
    % plot target point
    plot3(xf,yf,zf,'ko','MarkerSize',7,'MarkerFaceColor','k');
end

% plot start point
plot3(xs,ys,zs,'ks','MarkerSize',10,'MarkerFaceColor','k');
% plot3(xs,ys,zs+50,'ko','MarkerSize',7,'MarkerFaceColor','k');

% Plot paths
p = [];
for i = size(sol, 1):-1:1
    p(i) = plot3(sol(i).path(:,1), sol(i).path(:,2), sol(i).path(:,3),...
        '-', 'LineWidth', 2, 'DisplayName', ['UAV', num2str(i)]);
end
legend(p,'Location', 'best');
% title('Search and Rescue');
hold off;
end