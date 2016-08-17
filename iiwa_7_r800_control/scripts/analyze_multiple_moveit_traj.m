clear
clc
format long g
clf

hold on
% ----------------------------------------------------------------------
% ----------------------------------------------------------------------

% Settings
show_moveit_pos = 0;
show_moveit_vel = 0;
show_moveit_acc = 0;
show_overlay    = 1;

horizon = 20; % how many points from each trajectory to display

% Limit time series to view
zoomed_in = 0;
if zoomed_in
    time_min = 3;
    time_max = 5;
    y_min = -inf;
    y_max = inf;
else
    time_min = -inf;
    time_max = inf;
    y_min = -inf;
    y_max = inf;
end
% ----------------------------------------------------------------------
% ----------------------------------------------------------------------

% Common Vars
joints = [1:1:7];
joint=3;
%joints=[joint:1:joint];

for n = 2:2:19
%for n = [2,18]
    file = sprintf('~/ros/current/ws_local/src/iiwa_trajectory_data/arm_moveit_trajectory_%i.csv',n);
    moveit_traj = csvread(file,1,0);
    i=1;
    moveit_timestamp=moveit_traj(:,i);i=i+1;

    % Preallocate memory
    moveit_pos= zeros(size(moveit_timestamp,1),7);
    moveit_vel= zeros(size(moveit_timestamp,1),7);
    moveit_acc = zeros(size(moveit_timestamp,1),7);

    % Convert input data to better structures
    for j = joints
        moveit_pos(:,j)=moveit_traj(:,i);i=i+1;
        moveit_vel(:,j)=moveit_traj(:,i);i=i+1;
        moveit_acc(:,j)=moveit_traj(:,i);i=i+1;
    end

    if (show_moveit_pos)
        plot(...
            moveit_timestamp, moveit_pos(:,1),'r', ...
            moveit_timestamp, moveit_pos(:,2),'y', ...
            moveit_timestamp, moveit_pos(:,3),'m', ...
            moveit_timestamp, moveit_pos(:,4),'c',...
            moveit_timestamp, moveit_pos(:,5),'g',...
            moveit_timestamp, moveit_pos(:,6),'b',...
            moveit_timestamp, moveit_pos(:,7),'k');
        ylabel('Position')
        title('Position Desired (dotted), Actual - All Joints')
    end

    if (show_moveit_vel)
        plot(...
            moveit_timestamp, moveit_vel(:,1),'--r', ...
            moveit_timestamp, moveit_vel(:,2),'--y', ...
            moveit_timestamp, moveit_vel(:,3),'--m', ...
            moveit_timestamp, moveit_vel(:,4),'--c',...
            moveit_timestamp, moveit_vel(:,5),'--g',...
            moveit_timestamp, moveit_vel(:,6),'--b',...
            moveit_timestamp, moveit_vel(:,7),'--k');
        ylabel('Velocity')
        title('Velocity Desired (dotted), Actual - All Joints')
    end

    if (show_moveit_acc)
        if (n==2)
            plot(...
                moveit_timestamp, moveit_acc(:,1),'-r',...
                moveit_timestamp, moveit_acc(:,2),'-y',...
                moveit_timestamp, moveit_acc(:,3),'-m',...
                moveit_timestamp, moveit_acc(:,4),'-c',...
                moveit_timestamp, moveit_acc(:,5),'-g',...
                moveit_timestamp, moveit_acc(:,6),'-b',...
                moveit_timestamp, moveit_acc(:,7),'-k');
        else
            plot(...
                moveit_timestamp, moveit_acc(:,1),':r',...
                moveit_timestamp, moveit_acc(:,2),':y',...
                moveit_timestamp, moveit_acc(:,3),':m',...
                moveit_timestamp, moveit_acc(:,4),':c',...
                moveit_timestamp, moveit_acc(:,5),':g',...
                moveit_timestamp, moveit_acc(:,6),':b',...
                moveit_timestamp, moveit_acc(:,7),':k');
        end
        ylabel('Acceleration')
        title('Acceleration Desired (dotted), Actual - All Joints')
    end

    if (show_overlay)
        plot(moveit_timestamp, moveit_acc(:,joint), 'LineWidth',2);
        ylabel('Acceleration')
        xlabel('Time')
        %title_str = sprintf('Joint %i Data',joint);
        %title(title_str)
        title('Affect of Iterative Parabolic Time Parameterization Iterations')

        legend('10',...
            '60',...
            '110',...
            '160',...
            '210',...
            '260',...
            '310',...
            '360',...
            '410',...
            '460',...
            'Location','northwest');
    end


    %add_legend()
    %axis([time_min time_max y_min y_max])

end
