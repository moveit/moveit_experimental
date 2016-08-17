clear
clc
format long g
clf

% ----------------------------------------------------------------------
% USER SETTINGS
% ----------------------------------------------------------------------
all_plots = false; % Show 4 plots in 1 window

% Offset moveit data
moveit_time_offset = 21.75;

% Mode
plot_position = 0;
plot_velocity = 0;
plot_acceration = 0;
plot_jerk = 0;
plot_joint = 1;

% Position
show_moveit_pos=0;

% Velocity
show_vel=0;
show_moveit_vel=1;
show_desired_vel=0;

% Acceleration
show_accel = 1;
show_moveit_accel = 0;

% Joint
joint = 6;
show_moveit_joint=0;

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

% ----------------------------------------------------------------------
% Read MoveIt Trajectory

have_moveit_traj=1;
if have_moveit_traj
    moveit_traj = csvread('~/ros/current/ws_local/src/iiwa_trajectory_data/arm_moveit_trajectory_1.csv',1,0);
    i=1;
    moveit_timestamp=moveit_traj(:,i);i=i+1;
    moveit_timestamp = moveit_timestamp + moveit_time_offset;

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
end
% ----------------------------------------------------------------------
% Read Controller State Trajecotry
controller_state = csvread('~/ros/current/ws_local/src/iiwa_trajectory_data/recorded_trajectory_11.csv',1,0);
i=1;
timestamp=controller_state(:,i);i=i+1;
t_delta=diff(timestamp);

% Preallocate memory
pos_desired = zeros(size(timestamp,1),7);
vel_desired = zeros(size(timestamp,1),7);
pos= zeros(size(timestamp,1),7);
vel= zeros(size(timestamp,1),7);
vel_delta= zeros(size(timestamp,1)-1,7);
acc = zeros(size(timestamp,1),7);
acc_delta = zeros(size(timestamp,1)-1,7);
jer = zeros(size(timestamp,1),7);

% Convert input data to better structures
for j = joints
    pos_desired(:,j)=controller_state(:,i);i=i+1;
    vel_desired(:,j)=controller_state(:,i);i=i+1;
    pos(:,j)=controller_state(:,i);i=i+1;
    vel(:,j)=controller_state(:,i);i=i+1;
    vel_delta(:,j)=diff(vel(:,j));
    acc(:,j)=[0; vel_delta(:,j) ./ t_delta]; % add a zero in front to make same size as other data structures
    acc_delta(:,j)=diff(acc(:,j));
    jer(:,j)=[0; acc_delta(:,j) ./ t_delta]; % add a zero in front to make same size as other data structures
end
% ----------------------------------------------------------------------
disp('Data Analysis:')
max_pos = max(pos);
max_vel = max(vel);
max_acc = max(acc);
max_jer = max(jer);
joint_names = {'Joint 1';'Joint 2';'Joint 3';'Joint 4';'Joint 5';'Joint 6';'Joint 7'};
var_names = {'Max_Pos','Max_Vel','Max_Acc', 'Max_Jerk'};
format short
disp(table(max_pos.', max_vel.', max_acc.', max_jer.',...
           'RowNames', joint_names, 'VariableNames', var_names))

% ----------------------------------------------------------------------
if all_plots
    subplot(2,2,1)
end
if plot_position
    plot(...
        timestamp, pos(:,1),'r', ...
        timestamp, pos(:,2),'y', ...
        timestamp, pos(:,3),'m', ...
        timestamp, pos(:,4),'c',...
        timestamp, pos(:,5),'g',...
        timestamp, pos(:,6),'b',...
        timestamp, pos(:,7),'k')
    if (show_moveit_pos)
        plot(...
            moveit_timestamp, moveit_pos(:,1),'--r', ...
            moveit_timestamp, moveit_pos(:,2),'--y', ...
            moveit_timestamp, moveit_pos(:,3),'--m', ...
            moveit_timestamp, moveit_pos(:,4),'--c',...
            moveit_timestamp, moveit_pos(:,5),'--g',...
            moveit_timestamp, moveit_pos(:,6),'--b',...
            moveit_timestamp, moveit_pos(:,7),'--k')
    end

    ylabel('Position')
    add_legend()
    title('Position: Actual (solid), MoveIt Desired (dotted) - All Joints')
    axis([time_min time_max y_min y_max])
end
% ----------------------------------------------------------------------
if all_plots
    subplot(2,2,2)
end
if plot_velocity
    hold on
    if (show_vel)
        plot(...
            timestamp, vel(:,1),'r',...
            timestamp, vel(:,2),'y',...
            timestamp, vel(:,3),'m',...
            timestamp, vel(:,4),'c',...
            timestamp, vel(:,5),'g',...
            timestamp, vel(:,6),'b',...
            timestamp, vel(:,7),'k');
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
    end
    if (show_desired_vel)
        plot(...
            timestamp, vel_desired(:,1),':r',...
            timestamp, vel_desired(:,2),':y',...
            timestamp, vel_desired(:,3),':m',...
            timestamp, vel_desired(:,4),':c',...
            timestamp, vel_desired(:,5),':g',...
            timestamp, vel_desired(:,6),':b',...
            timestamp, vel_desired(:,7),':k');
    end
    hold off
    ylabel('Velocity')
    add_legend()
    title('Velocity Desired (dotted), Actual - All Joints')
    axis([time_min time_max y_min y_max])
end
% ----------------------------------------------------------------------
if all_plots
    subplot(2,2,3)
end
if plot_acceration
    hold on
    if (show_accel)
        plot(...
            timestamp, acc(:,1),'r',...
            timestamp, acc(:,2),'y',...
            timestamp, acc(:,3),'m',...
            timestamp, acc(:,4),'c',...
            timestamp, acc(:,5),'g',...
            timestamp, acc(:,6),'b',...
            timestamp, acc(:,7),'k')
    end
    if (show_moveit_accel)
        plot(...
            moveit_timestamp, moveit_acc(:,1),'--r', ...
            moveit_timestamp, moveit_acc(:,2),'--y', ...
            moveit_timestamp, moveit_acc(:,3),'--m', ...
            moveit_timestamp, moveit_acc(:,4),'--c',...
            moveit_timestamp, moveit_acc(:,5),'--g',...
            moveit_timestamp, moveit_acc(:,6),'--b',...
            moveit_timestamp, moveit_acc(:,7),'--k')
    end
    hold off
    ylabel('acceration')
    add_legend()
    title('acceration Actual - All Joints')
    axis([time_min time_max y_min y_max])
end
% ----------------------------------------------------------------------
if plot_jerk
    hold on
    plot(...
        timestamp, jer(:,1),'r',...
        timestamp, jer(:,2),'y',...
        timestamp, jer(:,3),'m',...
        timestamp, jer(:,4),'c',...
        timestamp, jer(:,5),'g',...
        timestamp, jer(:,6),'b',...
        timestamp, jer(:,7),'k')
    hold off
    ylabel('jerk')
    add_legend()
    title('jerk Actual - All Joints')
    axis([time_min time_max y_min y_max])
end
% ----------------------------------------------------------------------
if all_plots
    subplot(2,2,4)
end
if plot_joint
    acc_scale = 0.05;
    jer_scale = 0.001;
    plot(...
        timestamp, pos(:,joint),'r',...
        timestamp, vel_desired(:,joint),'--k',...
        timestamp, vel(:,joint),'g',...
        timestamp, acc(:,joint) * acc_scale,'b',...
        timestamp, jer(:,joint) * jer_scale,'m')
    if (show_moveit_joint)
        plot(...
            moveit_timestamp, moveit_vel(:,joint),'y')
    end

    xlabel('Time')
    ylabel('Value')
    legend('Position',...
           'Desired Velocity',...
           'Velocity',...
           'Acceration',...
           'Jerk',...
           'MoveIt Velocity',...
           'Location','northwest')

    title_str = sprintf('Joint %i Data',joint);
    title(title_str)
    axis([time_min time_max y_min y_max])
end
