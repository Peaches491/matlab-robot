function [f] = simple_gui2(robot, t, traj, t_step)
% SIMPLE_GUI2 Select a data set from the pop-up menu, then
% click one of the plot-type push buttons. Clicking the button
% plots the selected data in the axes.

%  Create and then hide the UI as it is being constructed.
f = figure('Visible','off','Position',[360,500,450,285]);

% Construct the components.

ha = axes('Units','pixels','Position',[50,60,200,185]);

% Initialize the UI.
% Change units to normalized so components resize automatically.
f.Units = 'normalized';
ha.Units = 'normalized';


% Create a plot in the axes.
current_data = zeros(1, robot.num_joints());
current_data = traj(1, :);
tf_scale = 0.10;
plotSetup(1.5, 135, 45, 'perspective');
plotArm(robot, current_data);

for link = 0 : robot.num_links()
    plotCoordTrans(robot.TF('end_link', link, 'config', current_data), tf_scale);
end



reset_btn = uicontrol('Style', 'pushbutton', ...
    'String', 'Reset', ...
    'Position',[315,0,70,25], ...
    'Callback', @surfbutton_Callback);
reset_btn.Units = 'normalized';


replay_btn = uicontrol('Style', 'pushbutton', ...
    'String', 'Replay', ...
    'Position',[115,0,70,25], ...
    'Callback', @replay);
replay_btn.Units = 'normalized';

sliders = [];
for link = 1:robot.num_joints()
    link_slider = uicontrol('Style','slider',...
        'String', strcat('Joint ', num2str(link)), ...
        'Position',[315, link*35, 100, 16], ...
        'Value', 0.5, ...
        'Callback', {@slider_callback, link});
    link_slider.Units = 'normalized';
    
    hProp = findprop(link_slider,'Value');
    addlistener(link_slider, 'Value', 'PostSet', ...
        @(x, y) update_plot(link, y.AffectedObject.Value));
    sliders = [sliders; link_slider];
end

align(sliders,'Center','None');
align([replay_btn; reset_btn],'Middle','None');



% Assign the a name to appear in the window title.
f.Name = 'Simple GUI';

% Move the window to the center of the screen.
movegui(f,'center');

% Make the window visible.
f.Visible = 'on';
f = flip(findobj(gcf()));

function redraw()
    plotArm_update(f(1:2), robot, current_data);

    for link = 0 : robot.num_links()
        T = robot.TF('end_link', link, 'config', current_data);
        [p, R] = TF_Pos_Rot(T);

        plotCoord_update(f(link+3), p, R, tf_scale);
    end
end
    
function update_plot(link, value)
    current_data(link) = value*2*pi - pi;
    redraw();
end

function slider_callback(source, eventdata, link)
    update_plot(link, source.Value);
end

function surfbutton_Callback(source,eventdata)
    current_data = zeros(1, robot.num_links());
    update_plot(1, 0.5);
end

function replay(source,eventdata)
    current_data = traj(1, :);
    redraw();
    pause(1);
    traj_idx = 1;
    start_t = now;
    while traj_idx <= numel(traj)
        current_data = traj(traj_idx, :);
        redraw();
        pause(t_step);
        traj_idx = find(t > (now - start_t)*100000, 1, 'first');
    end
    disp 'Play complete.'
end

replay();

end