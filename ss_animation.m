clc;
clear;
close all;

model = CreateModel3();
file = load('drone.mat');
uavs = file.uavs;

figure();
set(gcf, 'Position', get(0, 'Screensize'));
iter = 0;

for it = 1:size(uavs)
    for j = 1:size(uavs(it).path,1)
        iter = iter + 1;
        
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
        
        % plot start point
        plot3(xs,ys,zs,'ks','MarkerSize',10,'MarkerFaceColor','k');

        for i = 1:it
            % Final location
            xf=uavs(i).target(1);
            yf=uavs(i).target(2);
            zf=uavs(i).target(3);

            % plot target point
            plot3(xf,yf,zf,'ko','MarkerSize',7,'MarkerFaceColor','k');
            if i ~= it
                plot3(uavs(i).path(:,1), uavs(i).path(:,2), uavs(i).path(:,3),...
                        '-', 'LineWidth', 3);
            end
        end

        plot3(uavs(it).path(1:j,1), uavs(it).path(1:j,2), uavs(it).path(1:j,3),...
                '-', 'LineWidth', 3);
        hold off;
        view([5 70])
        drawnow
        F(iter) = getframe(gcf);
    end
end

video = VideoWriter('result.avi','Motion JPEG AVI');
video.FrameRate = 20;  % (frames per second) this number depends on the sampling time and the number of frames you have
open(video);
writeVideo(video,F);
close(video);