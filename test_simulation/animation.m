function animation(c,joint,path,movie,speed, filename, pen_down)

d3 = c(1);
d4 = c(2);
px = path(1,:);
py = path(2,:);
pz = path(3,:);

h = 0; % base height
persistent strokes stroke_color current_stroke last_pen_state;
% Initialize once
if isempty(strokes)
    strokes = {}; 
    stroke_color = {};
    current_stroke = []; 
    last_pen_state = 0;
end
traced_path = [];   % reset every call

%% Create Movie
if movie == 1
    v = VideoWriter(filename + ".mp4", "MPEG-4");
    open(v);
end
%% Drawing loop
for i = 1:speed:length(joint)
    
    [fx,fy,fz,T] = FK(c,joint(:,i));
%     fz = fz + 0.1;
    
    %% Base
    plot3([0 fx(1)],[0 fy(1)],[-h fz(1)],'k','linewidth',8);
    hold on;
    %% Manipulator
    plot3(fx(1:end-1),fy(1:end-1),fz(1:end-1),'k','linewidth',4);

    %% Tool
    plot3(fx(end-1:end),fy(end-1:end),fz(end-1:end),'m-','linewidth',3);

    %% Frames
    for j = 1:5
        Rj = T{j}(1:3,1:3);
        mag = 0.025;
        plot3(fx(j)+[0 Rj(1,1)]*mag,fy(j)+[0 Rj(2,1)]*mag,fz(j)+[0 Rj(3,1)]*mag,'r','linewidth',2); % x
        plot3(fx(j)+[0 Rj(1,2)]*mag,fy(j)+[0 Rj(2,2)]*mag,fz(j)+[0 Rj(3,2)]*mag,'g','linewidth',2); % y
        plot3(fx(j)+[0 Rj(1,3)]*mag,fy(j)+[0 Rj(2,3)]*mag,fz(j)+[0 Rj(3,3)]*mag,'b','linewidth',2); % z
    end

    %% Trajectory
    % plot3(px,py,pz,'b');
    
    % Detect pen going DOWN (start of a new stroke)
    if pen_down(i) && ~last_pen_state
        current_stroke = [];   % reset stroke buffer
    end
    
    % Record stroke points when pen is DOWN
    if pen_down(i)
        current_stroke = [current_stroke, path(:,i)];
    end
    
    % Detect pen going UP → finish current stroke
    if ~pen_down(i) && last_pen_state
        if ~isempty(current_stroke)
            strokes{end+1} = current_stroke;
    
            % Assign stroke color
            if length(strokes) == 1
                stroke_color{end+1} = [1 0 0];   % red = big triangle
            else
                stroke_color{end+1} = [0 0 1];   % blue = small triangle
            end
    
            current_stroke = []; % clear for next stroke
        end
    end
    
    % If last frame AND pen still down → save stroke
    if i == length(pen_down) && pen_down(i)
        if ~isempty(current_stroke)
            strokes{end+1} = current_stroke;
    
            if length(strokes) == 1
                stroke_color{end+1} = [1 0 0];
            else
                stroke_color{end+1} = [0 0 1];
            end
        end
    end
    
    % ---- Draw all completed strokes ----
    for s = 1:length(strokes)
        stroke = strokes{s};
        color  = stroke_color{s};
        plot3(stroke(1,:), stroke(2,:), stroke(3,:), ...
              'Color', color, 'LineWidth', 2);
    end
    
    % ---- Draw current active stroke (not finished yet) ----
    if pen_down(i) && ~isempty(current_stroke)
        % Determine this stroke's color:
        if isempty(strokes)
            this_color = [1 0 0];   % big triangle
        else
            this_color = [0 0 1];   % small triangle
        end
    
        plot3(current_stroke(1,:), current_stroke(2,:), current_stroke(3,:), ...
              'Color', this_color, 'LineWidth', 2);
    end
    
    % Update pen state
    last_pen_state = pen_down(i);

    %% Ground
    X = [1 -1;1 -1]*0.2;
    Y = [1 1;-1 -1]*0.2;
    Z = [1 1;1 1]*-h;
    surf(X,Y,Z,'FaceColor',[0.9 0.9 0.9],'edgecolor','none'); hold on;
    % Label
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    axis([-0.2 0.2 -0.2 0.2 -0.2 0.2]);
    pbaspect([1 1 1]);
    grid on;
%     view(0,0);
    view(40,30);
    hold off;
    drawnow;
    
    if movie == 1
    frame = getframe(gcf);
    writeVideo(v,frame);
    end

end

if movie == 1
    close(v);
    fprintf('Video saved as filename.mp4\n');

end

end