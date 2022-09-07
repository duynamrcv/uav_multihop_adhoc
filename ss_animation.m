clc;
clear;
close all;

model = CreateModel2();
file = load('drone.mat');
uavs = file.uavs;

figure();
set(gcf, 'Position', get(0, 'Screensize'));
iter = 0;

% Plot model
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
% Plot threats
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
    set(c,'Edgecolor','none','Facecolor','black','FaceAlpha',.5); % set color and transparency
end
% Start location
xs=model.start(1);
ys=model.start(2);
zs=model.start(3);
plot3(xs,ys,zs,'bs','MarkerSize',10,'MarkerFaceColor','b');

% End location
xf=model.goal(1);
yf=model.goal(2);
zf=model.goal(3);
plot3(xf,yf,zf,'rp','MarkerSize',10,'MarkerFaceColor','r');
        
step = 4;
% Plot UAVS path
for it = 1:size(uavs)
    k = 0;
    for j = 1:step:size(uavs(it).path,1)
        iter = iter + 1;
%         hold on

        %% Plot previous UAVS
        for i = 1:it-1
%             % Path
%             plot3(uavs(i).path(:,1), uavs(i).path(:,2), uavs(i).path(:,3),...
%                     '-', 'LineWidth', 3);
%                 
%             % Target
%             scatter3(uavs(i).target(1:end-1,1),...
%                     uavs(i).target(1:end-1,2),...
%                     uavs(i).target(1:end-1,3),...
%                     70,'k','fill', 'Marker', 'h');
            scatter3(uavs(i).target(end,1),...
                    uavs(i).target(end,2),...
                    uavs(i).target(end,3),...
                    70,'b','fill', 'Marker', 'o');
            
        end
        
        %% Curent UAV
        % Path
        plot3(uavs(it).path(1:j,1), uavs(it).path(1:j,2), uavs(it).path(1:j,3),...
                '-', 'LineWidth', 3);
        % Target
        if it == 1
            if norm(uavs(it).path(j,:) - uavs(it).path(1,:)) < 2.0
                k = 1;
            end
        else
            if norm(uavs(it).path(j,:) - uavs(it-1).path(end,:)) < 15.0
                k = 1;
            end
        end
        if k > 0
            if norm(uavs(it).path(j,:) - uavs(it).target(k,:)) < 6.0
                if k < size(uavs(it).target,1)
                    k = k+1;
                end
            end
            scatter3(uavs(it).target(k,1),...
                    uavs(it).target(k,2),...
                    uavs(it).target(k,3),...
                    70,'k','fill', 'Marker', 'h');
        end
        pltx = scatter3(uavs(it).path(j,1),...
                        uavs(it).path(j,2),...
                        uavs(it).path(j,3),...
                        70,'r','fill', 'Marker', 'o');
%         hold off;
        view([5 70]);
        drawnow
        F(iter) = getframe(gcf);
        delete(pltx);
    end
end

%% End frame
iter = iter + 1;

% Plot previous UAVS
for i = 1:size(uavs)
%             % Path
%             plot3(uavs(i).path(:,1), uavs(i).path(:,2), uavs(i).path(:,3),...
%                     '-', 'LineWidth', 3);
%                 
%             % Target
%             scatter3(uavs(i).target(1:end-1,1),...
%                     uavs(i).target(1:end-1,2),...
%                     uavs(i).target(1:end-1,3),...
%                     70,'k','fill', 'Marker', 'h');
    scatter3(uavs(i).target(end,1),...
            uavs(i).target(end,2),...
            uavs(i).target(end,3),...
            70,'b','fill', 'Marker', 'o');

end
view([5 70]);
drawnow
F(iter) = getframe(gcf);

video = VideoWriter('result.avi','Motion JPEG AVI');
video.FrameRate = 20;  % (frames per second) this number depends on the sampling time and the number of frames you have
open(video);
writeVideo(video,F);
close(video);