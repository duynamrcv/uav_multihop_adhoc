function PlotTest(model)
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
plot3(xs,ys,zs,'ks','MarkerSize',10,'MarkerFaceColor','k');

plot3(2.541482205754813e+02,1.798824864400457e+02,3.388946228027344e+02,'rd','MarkerSize',10,'MarkerFaceColor','r');

% Goal location
xg=model.goal(1);
yg=model.goal(2);
zg=model.goal(3);
plot3(xg,yg,zg,'ko','MarkerSize',7,'MarkerFaceColor','k');

end