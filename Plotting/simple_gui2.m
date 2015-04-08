function [f] = simple_gui2(robot)
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
current_data = [0, 0, 0, 0, 0, 0];
tf_scale = 0.2;
plotSetup(0.90, 148, 15, 'perspective');
plotArm(robot, current_data);

for link = 1 : num_joints(robot)
    plotCoordTrans(TF(robot, 'end_link', link, 'config', current_data), tf_scale);
end



reset_btn = uicontrol('Style', 'pushbutton', ...
    'String', 'Reset', ...
    'Position',[315,0,70,25], ...
    'Callback', @surfbutton_Callback);

sliders = [];
for link = 1:size(robot, 2)
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

align([sliders; reset_btn],'Center','None');



% Assign the a name to appear in the window title.
f.Name = 'Simple GUI';

% Move the window to the center of the screen.
movegui(f,'center')

% Make the window visible.
f.Visible = 'on';


    
function update_plot(link, value)
    current_data(link) = value*2*pi - pi;

    f = flip(findobj(gcf()));

    plotArm_update(f(1:2), robot, current_data);

    for link = 1 : num_joints(robot)
        T = TF(robot, 'end_link', link, 'config', current_data);
        [p, R] = TF_Pos_Rot(T);

        plotCoord_update(f(link+2), p, R, tf_scale);
    end
end

function slider_callback(source, eventdata, link)
    update_plot(link, source.Value)
end

function surfbutton_Callback(source,eventdata)
    cla(ha);
    current_data = [0, 0, 0, 0, 0, 0];
    plotarm_replot(robot, current_data);
end

end