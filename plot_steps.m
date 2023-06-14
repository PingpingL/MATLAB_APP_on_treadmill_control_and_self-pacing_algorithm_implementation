% load('walkingTest_lateral.mat');
% load('lateral_test_model2_1.mat');
% load('lateral_test_model2_2.mat');
load('lateral_test_model2_3.mat');

% data extraction
cop_left = data.allData(:,29);
cop_right = data.allData(:,30);
com_dcom_forward = data.allData(:,25);
com_dcom_lateral = data.allData(:,51);
n_ts = data.allData(:,4);
stance_L = data.allData(:,5);
stance_R = data.allData(:,6);
dT_steps = data.allData(:,2);

% convert to array
cop_left = table2array(cop_left);
cop_right = table2array(cop_right);
com_dcom_forward = table2array(com_dcom_forward);
com_dcom_lateral = table2array(com_dcom_lateral);
stance_L = table2array(stance_L);
stance_R = table2array(stance_R);
step_detector = table2array(n_ts);
dT_steps = table2array(dT_steps);

% parameters preparation(in c coordinate)
% core parameter
cop_left_forward = cop_left(:,1);
cop_left_lateral = -cop_left(:,2);

cop_right_forward = cop_right(:,1);
cop_right_lateral = -cop_right(:,2);

com_forward = com_dcom_forward(:,1);
com_lateral = com_dcom_lateral(:,1);

dcom_forward = com_dcom_forward(:,2);
dcom_lateral = com_dcom_lateral(:,2);

% distinguish of each step
index_new_step = find(step_detector==0);

%slice
l = 1;
r = 1;
for i = 1:length(index_new_step)-1
    if stance_L(index_new_step(i)) == 1 && stance_L(index_new_step(i)-1) == 0
        step_left_index(l,1) = index_new_step(i);
        step_left_index(l,2) = index_new_step(i+1)-index_new_step(i);
        l = l+1;
    else 
        step_right_index(r,1) = index_new_step(i);
        step_right_index(r,2) = index_new_step(i+1)-index_new_step(i);
        r = r+1;
    end
end

num_left = size(step_left_index);
for i = 1:num_left(1)
    left_y= cop_left_forward(step_left_index(i,1):step_left_index(i,1)+step_left_index(i,2));
    left_x= cop_left_lateral(step_left_index(i,1):step_left_index(i,1)+step_left_index(i,2));

    com_forward_cut = com_forward(step_left_index(i,1):step_left_index(i,1)+step_left_index(i,2));
    com_lateral_cut = com_lateral(step_left_index(i,1):step_left_index(i,1)+step_left_index(i,2));

    dcom_forward_cut = dcom_forward(step_left_index(i,1):step_left_index(i,1)+step_left_index(i,2));
    dcom_lateral_cut = dcom_lateral(step_left_index(i,1):step_left_index(i,1)+step_left_index(i,2));
    
    dt_cut_l = dT_steps(step_left_index(i,1):step_left_index(i,1)+step_left_index(i,2));

    field = append("step", num2str(2*i));
    step.(field)= [left_x,left_y,dt_cut_l];
    com_step.(field) = [com_lateral_cut,com_forward_cut];
    dcom_step.(field) = [dcom_lateral_cut,dcom_forward_cut];
end

num_right = size(step_right_index);
for i = 1:num_right(1)
    right_y= cop_right_forward(step_right_index(i,1):step_right_index(i,1)+step_right_index(i,2));
    right_x= cop_right_lateral(step_right_index(i,1):step_right_index(i,1)+step_right_index(i,2));

    com_forward_cut = com_forward(step_right_index(i,1):step_right_index(i,1)+step_right_index(i,2));
    com_lateral_cut = com_lateral(step_right_index(i,1):step_right_index(i,1)+step_right_index(i,2));

    dcom_forward_cut = dcom_forward(step_right_index(i,1):step_right_index(i,1)+step_right_index(i,2));
    dcom_lateral_cut = dcom_lateral(step_right_index(i,1):step_right_index(i,1)+step_right_index(i,2));

    dt_cut_r = dT_steps(step_right_index(i,1):step_right_index(i,1)+step_right_index(i,2));

    field = append("step", num2str(2*i-1));
    step.(field)= [right_x,right_y,dt_cut_r];
    com_step.(field) = [com_lateral_cut,com_forward_cut];
    dcom_step.(field) = [dcom_lateral_cut,dcom_forward_cut];
end


% move cop and com
dy = 0;
for i = 1:length(index_new_step)-2
    field = append("step", num2str(i+1));
    cop = step.(field);
    com_points = com_step.(field);
    dcom_lines = dcom_step.(field);
    
    y = cop(:,2);
    dt = cop(:,3);
    y_com = com_points(:,2);

    for k = 1:length(cop)
        y(k) = y(k)+dy+(sum(dt(1:k))-dt(1));
        y_com(k) = y_com(k)+dy+(sum(dt(1:k))-dt(1));
    end
    dy = dy+sum(dt(1:k))-dt(1);

    dcom_vec = dcom_lines(1,:);
    cop(:,2) = y;
    com_points(:,2) = y_com;
    step.(field) = cop;
    com_step.(field) = com_points;
    dcom_step.(field) = dcom_vec;
end

% plot
figure(1)
for i = 1:20
    field = append("step", num2str(i));
    cop_plot = step.(field);
    com_plot = com_step.(field);
    dcom_plot = dcom_step.(field); 
    hold on;
    scatter(cop_plot(:,1),cop_plot(:,2),[],"blue");
    scatter(com_plot(:,1),com_plot(:,2),"black",'Marker','.');
    plot([com_plot(1,1),com_plot(1,1)+dcom_plot(1)],[com_plot(1,2),com_plot(1,2)],"Color",'g','LineStyle','-');% lateral
    plot([com_plot(1,1),com_plot(1,1)],[com_plot(1,2),com_plot(1,2)+dcom_plot(2)],"Color",'r','LineStyle','-');
end
legend('cop','com','lateral speed','forward speed');
axis([-0.5,0.5,0,1])
% axis([-0.5,0.5,2,13])
