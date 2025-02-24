% 初始化
numExperiments = 3;  % 总实验次数
obstacle_height = 90 + (200 - 90) * rand(1, numExperiments);  % 设定随机的大臂长范围 [90, 200]
arm_length = 120;
small_arm_length = 200;


% 用于存储所有实验的位置信息和theta轨迹
% 修改后的初始化部分
all_positions = cell(numExperiments, 1);  
all_thetas = cell(numExperiments, 1);  
all_target_thetas = cell(numExperiments, 1);  % 新增目标theta存储
all_phis = cell(numExperiments, 1);          % 新增phi存储
steps_at_break = zeros(numExperiments, 1);  
heights = zeros(numExperiments, 1); 

% 循环进行10次实验
for experiment = 1:numExperiments
    % 设置当前实验的参数
    % 创建自定义环境实例
    obstacle_height(experiment)
    myEnv = MyEnvironment();
    
    % 加载训练好的智能体（假设保存为'trainedAgent.mat'）
    load('Agent85.mat', 'agent');
    
    % 环境重置并获取初始观察
    InitialObs = reset(myEnv, obstacle_height(experiment),arm_length,small_arm_length);

    % 设置最大仿真步数
    maxSteps = 8000;
    numEpisodes = 20;
    
    for step = 1:maxSteps
        % 使用训练好的策略选择动作
        Action = getAction(agent, InitialObs);  % 根据观察选择动作
        
        % 执行动作并获取新状态
        obs = myEnv.step(Action{1});
        
        % 更新观察状态
        InitialObs = obs;

        if myEnv.Mydone == true
            % 记录break发生时的步数
            steps_at_break(experiment) = step;
            heights(experiment) = obstacle_height(experiment);  % 记录对应的障碍物高度
            break;
        end
    end
    
    % 提取当前位置数据
    data_map = myEnv.myMap_list;
    num_entries = numel(data_map);  % 获取数据条目数量
    % 预分配内存
    positions = zeros(num_entries, 2);  
    thetas = zeros(num_entries, 1);  
    target_thetas = zeros(num_entries, 1);  % 新增
    phis = zeros(num_entries, 1);           % 新增

    
    for n = 1:num_entries
        data = data_map{n}('data');  
        abs_pos = data('绝对位置');  
        positions(n, :) = abs_pos(:)';  
        
        % 角度数据提取
        thetas(n) = data('theta');
        target_thetas(n) = data('目标theta');  % 新增
        phis(n) = data('phi');                % 新增
    end
    
    % 存储实验结果
    all_positions{experiment} = positions;
    all_thetas{experiment} = thetas;
    all_target_thetas{experiment} = target_thetas;  % 新增
    all_phis{experiment} = phis;                    % 新增

end


% 绘制图形
% 绘制绝对位置分布图
figure;
hold on;
for experiment = 1:numExperiments
    positions = all_positions{experiment};
    plot(positions(:, 1), positions(:, 2), '-', 'MarkerSize', 8, 'LineWidth', 1.5);
end

% 创建 legend 标签
legend_labels = arrayfun(@(x) sprintf('障碍物高度=%.1f', obstacle_height(x)), ...
                         1:numExperiments, 'UniformOutput', false);

xlabel('X坐标');
ylabel('Y坐标');
title('不同参数下的绝对位置分布图');
grid on;
legend(legend_labels);  % 使用格式化后的标签
hold off;

% 绘制障碍物高度和步数的直方图
figure;
scatter(heights, steps_at_break, 'filled');
xlabel('障碍物高度');
ylabel('用掉的步数');
title('障碍物高度与用掉的步数的关系');
grid on;

% 绘制theta变化轨迹
figure;
hold on;

% 平滑因子，可以根据需要调整
smooth_factor = 70;

% 获取更多颜色
colors = lines(numExperiments); 

for experiment = 1:numExperiments
    thetas = all_thetas{experiment};
    
    % 对数据进行平滑
    smoothed_thetas = smooth(thetas, smooth_factor);
    
    % 绘制平滑后的曲线，指定颜色
    plot(smoothed_thetas, 'LineWidth', 1.5, 'Color', colors(experiment, :));
end

% 创建 legend 标签
legend_labels = arrayfun(@(x) sprintf('障碍物高度=%.1f', obstacle_height(x)), ...
                         1:numExperiments, 'UniformOutput', false);

xlabel('步数');
ylabel('theta值');
title('不同实验的theta变化轨迹');
grid on;
legend(legend_labels);  % 使用格式化后的标签
hold off;

figure;
hold on;

smooth_factor = 70;
colors = lines(numExperiments); 

for experiment = 1:numExperiments
    phis = all_phis{experiment};
    smoothed_phis = smooth(phis, smooth_factor);
    plot(smoothed_phis, 'LineWidth', 1.5, 'Color', colors(experiment, :));
end

legend_labels = arrayfun(@(x) sprintf('障碍物高度=%.1f', obstacle_height(x)),...
                         1:numExperiments, 'UniformOutput', false);

xlabel('步数');
ylabel('phi值');
title('不同实验的phi变化轨迹');
grid on;
legend(legend_labels);
hold off;

figure;
hold on;
colors = lines(numExperiments);  % 使用与之前一致的配色方案

% 设置线型参数
theta_line_width = 1.5;
target_line_width = 1.2;

for experiment = 1:numExperiments
    % 获取数据
    x_coords = all_positions{experiment}(:,1);      % X坐标数据
    theta_values = all_thetas{experiment};          % theta数据
    target_theta_values = all_target_thetas{experiment}; % 目标theta
    
    % 绘制theta曲线
    plot(x_coords, theta_values,...
        'LineWidth', theta_line_width,...
        'Color', colors(experiment,:),...
        'DisplayName', sprintf('theta (高度=%.1f)', heights(experiment)));
    
    % 绘制目标theta曲线（虚线）
    plot(x_coords, target_theta_values, '--',...
        'LineWidth', target_line_width,...
        'Color', colors(experiment,:),...
        'DisplayName', sprintf('目标theta (高度=%.1f)', heights(experiment)));
end

% 图形修饰
xlabel('绝对位置X坐标');
ylabel('角度值（弧度）');
title('theta与目标theta随X坐标变化对比');
legend('Location','bestoutside');  % 将图例放在外侧避免遮挡
grid on;
axis tight;
hold off;

% 新增部分4：phi随X坐标变化图
figure;
hold on;

% 设置绘图参数
phi_line_width = 1.5;
smooth_factor = 90;
for experiment = 1:numExperiments
    % 获取数据
    x_coords = all_positions{experiment}(:,1);  % X坐标数据
    phi_values = all_phis{experiment};          % phi数据
    
    % 绘制平滑曲线（使用与theta相同的平滑因子）
    smoothed_phi = smooth(phi_values, smooth_factor);
    
    plot(x_coords, smoothed_phi,...
        'LineWidth', phi_line_width,...
        'Color', colors(experiment,:),...
        'DisplayName', sprintf('phi (高度=%.1f)', heights(experiment)));
end

% 图形修饰
xlabel('绝对位置X坐标');
ylabel('phi值（弧度）');
title('phi随X坐标变化轨迹');
legend('Location','bestoutside');
grid on;
axis tight;
hold off;