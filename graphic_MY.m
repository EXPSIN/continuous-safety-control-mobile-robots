function H = graphic_MY(H, t, p, R, dx, env, D_B, ith_controller)

clr         = [[92 200 92]; [245 194 65];  [92 92 200]; [200 92 92]]/255;
line_style  = {'-.'; ':'; '-'};
% legend_name = {'Usual', 'MY', 'Proposed'};
legend_name = {'UC', 'MYC', 'PC'};

[r, l] = measurements_2D(p, R, env, D_B);


if(ith_controller == 2)    
    r_L = regularization_MY(l', r, l)';
else
    n_L = 9;  c_A = cos(2*pi/n_L);
    r_L = regularization(l', r, l,c_A)';
end

p_obs  = p + R*l.*r;
p_re   = p + R*l.*r_L;
D    = evalin('base', 'D');
Time = evalin('base', 'Time');

if(isempty(H))
    figure(1); 
    hold on; 
    axis equal;
    box on;
    grid on;
    set(gcf, 'color', 'w', 'position', [50,50,800,350]);
    set(gca, 'fontsize', 20, 'position', [0.05, 0.2, 0.93,0.79]);
    xlabel('$[p]_1$ (m)', 'Interpreter','latex');
    ylabel('$[p]_2$ (m)', 'Interpreter','latex');
    % axis off;
    range = [env.p_e, env.p_e + [0, 1; -1, 0] * size(env.obs)'/env.N];
    axis([min(range(1, :))-1.5, max(range(1, :))+4, min(range(2, :)), max(range(2, :))]);
    axis([-4, 10, -2.5, 3.5]);

    % set(gcf, 'position', [50,50,800,350]); axis([-6,8,-2.5,3.5]); set(gca, 'position', [0.05, 0.2, 0.93,0.79])
    
    H.obs_map = patch_img(env.obs, env.p_e, env.N);
    for i = 1:length(legend_name)
        H.robot(i) = graphic_plant_2D([], p, R, 0.5, clr(i, :), line_style{i});
        H.robot(i).robot_path.HandleVisibility = 'on';
        H.robot(i).robot_path.DisplayName = legend_name{i};
        H.obs(i)   = patch(p_obs(1,:), p_obs(2,:), -ones(1, size(p_obs,2)), clr(i, :), 'facealpha', 0.1, 'handlevisibility','off');
        H.obs_re(i)= patch( p_re(1,:),  p_re(2,:), -0.9*ones(1, size( p_re,2)), clr(i, :), 'facealpha', 0.2, 'handlevisibility','off', 'Visible', 'off');
    end

    % H.fig1_feasible_set = half_space_2D([],    [1,0], 0, [], [], [], [], [0; 0]);

    H.fig1_feasible_set = half_plane([],  [0,0], 0, [0;0], [-6,8,-2.5,3.5], 'y', '');

    figure(11); 
    set(gcf, 'color', 'w', 'position', [50,350,800,600]);
        
    for i = 1:length(legend_name)
        subplot(length(legend_name),1,i);
        hold on; axis equal; box on; grid on;
        set(gca, 'fontsize', 20, 'XAxisLocation', 'origin');
        range = [env.p_e, env.p_e + [0, 1; -1, 0] * size(env.obs)'/env.N];
        axis([min(range(1, :))-2, max(range(1, :))+7, -2.1, 3.1]);
        % axis([-8, 7, -2.5, 3.5]);
        H.fig11_gca(i) = gca;
        H.fig11_obs_map = patch_img(env.obs, env.p_e, env.N);
        H.fig11_robot(i) = graphic_plant_2D([], p, R, 0.5, clr(i, :), line_style{i});
        H.fig11_robot(i).robot_path.HandleVisibility = 'on';
        H.fig11_robot(i).robot_path.DisplayName = legend_name{i};
        % H.fig11_robot(i).robot_cir.EdgeColor = clr(i, :);
        H.fig11_robot(i).robot_cir.EdgeColor = [0;0;0];
        H.fig11_axes(i) = gca;
        H.fig11_obs_or_re(i)= patch( p_re(1,:),  p_re(2,:), -0.9*ones(1, size( p_re,2)), clr(i, :), 'facealpha', 0.2, 'handlevisibility','off', 'Visible', 'on');
        
        xlabel('$[p]_1$ (m)', 'Interpreter','latex');
        ylabel({legend_name{i},'$[p]_2$ (m)'}, 'Interpreter','latex');
    end
    fullScreenSubplots(gcf, 0.13, 0.01, 0.08, 0.02, 0.01);

    figure(2); 
    hold on; 
    box on;
    grid on;
    set(gcf, 'color', 'w', 'position', [850,50,800,300]);
    set(gca, 'fontsize', 20, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
    xlabel('$t$ (sec)', 'Interpreter','latex');
    ylabel('$[v]_1$ (m/s)', 'Interpreter','latex');
    xlim([0,Time]);

    for i = 1:length(legend_name)
        H.fig2_traj_vx(i) = animatedline('Color', clr(i, :), 'LineWidth', 2, 'LineStyle', line_style{i}, 'displayname', legend_name{i});
    end
    H.fig2_legend = legend('location', 'southeast');
    H.fig2_legend.AutoUpdate = "off";

    figure(3); 
    hold on; 
    box on;
    grid on;
    set(gcf, 'color', 'w', 'position', [850,500,800,300]);
    set(gca, 'fontsize', 20, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin', 'Position', [0.16,0.25,0.8,0.74]);
    xlabel('$t$ (sec)', 'Interpreter','latex');
    ylabel({'Minimum', 'Distance (m)'}, 'Interpreter','latex');
    xlim([0, Time]);
    ylim([0, D*4]);
    
    H.fig3_traj_expected_distance = plot([0, Time], [D, D], 'Color', [0.5,0.5,0.5], 'LineWidth', 1, 'linestyle', '--', 'HandleVisibility','off');
    for i = 1:length(legend_name)
        H.fig3_traj_dist(i) = animatedline('Color', clr(i, :), 'LineWidth', 2, 'LineStyle', line_style{i}, 'displayname', legend_name{i});
    end
    H.fig3_legend = legend('location', 'southeast');
    H.fig3_legend.AutoUpdate = "off";


    figure(4); 
    set(gcf, 'color', 'w', 'position', [950,400,800,400]);
    xlabel('$t$ (sec)', 'Interpreter','latex');
    xlim([0,Time]);
    for i = 1:length(legend_name)
        subplot(length(legend_name),1,i); 
        hold on;  box on; grid on;
        set(gca, 'fontsize', 16);
        ylabel({legend_name{i}, '(m/s)'}, 'Interpreter','latex');
        xlim([0,Time]);
        H.fig4_traj_v_x(i)  = animatedline('Color', clr(i, :), 'LineWidth', 2, 'LineStyle', '--', 'displayname', 'Velocity Reference');
        H.fig4_traj_va_x(i) = animatedline('Color', clr(i, :), 'LineWidth', 2, 'LineStyle', '-', 'displayname', 'Actual Velocity');
        H.fig4_legend(i) = legend();
        H.fig4_legend(i).AutoUpdate = "off";
    end
    xlabel('$t$ (sec)', 'Interpreter','latex');

    figure(5); 
    set(gcf, 'color', 'w', 'position', [950,50,800,400]);
    for i = 1:length(legend_name)
        subplot(length(legend_name),1,i);
        hold on;  box on; grid on;
        set(gca, 'fontsize', 16);
        ylabel({legend_name{i}, '(m/s)'}, 'Interpreter','latex');
        xlim([0,Time]);
        H.fig5_traj_v_y(i)  = animatedline('Color', clr(i, :), 'LineWidth', 2, 'LineStyle', '--', 'displayname', 'Velocity Reference');
        H.fig5_traj_va_y(i) = animatedline('Color', clr(i, :), 'LineWidth', 2, 'LineStyle', '-', 'displayname', 'Actual Velocity');
        H.fig5_legend(i) = legend();
        H.fig5_legend(i).AutoUpdate = "off";
    end
    xlabel('$t$ (sec)', 'Interpreter','latex');
    
else
    addpoints(H.fig2_traj_vx(ith_controller),   t, dx(1));
    addpoints(H.fig3_traj_dist(ith_controller), t, min(r));
    try
        v = evalin('base', 'v');
        addpoints(H.fig4_traj_v_x(ith_controller),  t, v(1));
        addpoints(H.fig5_traj_v_y(ith_controller),  t, v(2));
    catch
        
    end
    
    addpoints(H.fig4_traj_va_x(ith_controller), t, dx(1));
    addpoints(H.fig5_traj_va_y(ith_controller), t, dx(2));
    
    for i = 1:length(legend_name)
        


        if( i == ith_controller)
            H.obs(i).Visible = true;
            H.obs_re(i).Visible = true;
            H.robot(i).robot_cir.Visible = true;
            H.robot(i).robot_dir.Visible = true;
        else
            H.obs(i).Visible = false;
            H.obs_re(i).Visible = false;
            H.robot(i).robot_cir.Visible = false;
            H.robot(i).robot_dir.Visible = false;
            continue;
        end

        H.robot(ith_controller)       = graphic_plant_2D(H.robot(ith_controller), p, R, 0.5, clr(ith_controller, :), line_style{ith_controller});
        H.fig11_robot(ith_controller) = graphic_plant_2D(H.fig11_robot(ith_controller), p, R, 0.5, clr(ith_controller, :), line_style{ith_controller});
        
        % if(mod(round(t, 1), 0.5) == 0 || round(t, 1) == 1.9 || round(t, 1) == 4.8)
        if(mod(round(t, 2), 0.05) == 0)
            transparent_val = min(0.3+0.7*t/10, 1.0);
            if(i == 3)
                transparent_val = min(0.9+0.1*t/10, 1.0);
            end
            H_new = copyobj(H.fig11_robot(ith_controller).robot_cir, H.fig11_axes(ith_controller));
            H_new.FaceAlpha = transparent_val*0.5;
            H_new.EdgeAlpha = 0;
            H_new = copyobj(H.fig11_robot(ith_controller).robot_dir, H.fig11_axes(ith_controller));
            H_new.FaceAlpha = transparent_val*0;
            H_new.EdgeAlpha = transparent_val*0;
        end

        H.obs(ith_controller).XData = p_obs(1, :);
        H.obs(ith_controller).YData = p_obs(2, :);
        H.obs_re(ith_controller).XData = p_re(1, :);
        H.obs_re(ith_controller).YData = p_re(2, :);

        if(ith_controller == 1)
            H.fig11_obs_or_re(ith_controller).XData = p_obs(1, :);
            H.fig11_obs_or_re(ith_controller).YData = p_obs(2, :);
        else
            H.fig11_obs_or_re(ith_controller).XData = p_re(1, :);
            H.fig11_obs_or_re(ith_controller).YData = p_re(2, :);
        end
    end
    
end


drawnow limitrate;
end




function H = graphic_plant_2D(H, p, R, radius, clr, linestyle)

if(isempty(H))
    H.dir = [1, -1/1.7937, -1/1.7937; 0, 0.4151, -0.4151]*0.618*radius;
    H.cir = [cos(linspace(0,2*pi,20)); sin(linspace(0,2*pi,20))]*radius;

    H.robot_cir  = patch(H.cir(1, :), H.cir(2, :), clr, 'HandleVisibility', 'off', 'FaceAlpha', 0.2, 'edgecolor', 'k', 'linewidth', 1);
    H.robot_dir  = patch(H.dir(1, :), H.dir(2, :), clr, 'HandleVisibility', 'off', 'FaceAlpha', 0.5, 'edgecolor', 'k', 'linewidth', 1);
    H.robot_path = animatedline(p(1), p(2), 'Color', clr, 'linewidth', 2, 'Linestyle', linestyle, 'HandleVisibility', 'off');
end 

H.robot_cir.XData = p(1) + H.cir(1, :);
H.robot_cir.YData = p(2) + H.cir(2, :);

dir_realtime = R*H.dir;
H.robot_dir.XData = p(1) + dir_realtime(1, :);
H.robot_dir.YData = p(2) + dir_realtime(2, :);

addpoints(H.robot_path, p(1), p(2));

end


%{
    draw the map
    pic         -- map
    p_m         -- the origin of the map
    resolution  -- specific pix per meter
%}
function h = patch_img(obs, p_m, resolution)
x =  (1:size(obs, 2))/resolution + p_m(1);  %  w/N + p_e(1)
y = -(1:size(obs, 1))/resolution + p_m(2);  % -h/N + p_e(2)


[X, Y] = meshgrid(x, y);

Z       = double(obs);
Z(Z<=0) = nan;
Z(Z> 0) =  -1;  % true means obstacle

h = surf('XData',X,'YData',Y,'ZData',Z, 'EdgeColor', 'none', 'displayname', 'obstacle', 'HandleVisibility', 'off');
colorres = [ones(1,3); 0,0,0];
colormap(colorres);

end





function fullScreenSubplots(figHandle, leftMargin, rightMargin, bottomMargin, topMargin, verticalSpacing)
    if nargin < 1
        figHandle = gcf;
    end
    if nargin < 2
        leftMargin = 0.1;
    end
    if nargin < 3
        rightMargin = 0.1;
    end
    if nargin < 4
        topMargin = 0.1;
    end
    if nargin < 5
        bottomMargin = 0.1;
    end
    if nargin < 6
        verticalSpacing = 0.05;
    end
    
    axHandles = findall(figHandle, 'Type', 'axes');
    axHandles = flipud(axHandles);
    numSubplots = numel(axHandles);
    
    totalVerticalSpace = topMargin + bottomMargin + (numSubplots - 1) * verticalSpacing;
    subplotHeight = (1 - totalVerticalSpace) / numSubplots;
    subplotWidth = 1 - leftMargin - rightMargin;
    
    for i = 1:numSubplots
        newPosition = [leftMargin, ...
                       bottomMargin + (numSubplots - i) * (subplotHeight + verticalSpacing), ...
                       subplotWidth, ...
                       subplotHeight];
        set(axHandles(i), 'Position', newPosition);
    end
end
