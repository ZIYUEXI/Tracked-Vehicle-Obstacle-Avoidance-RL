classdef MyEnvironment < rl.env.MATLABEnvironment
    %% 属性 (根据需要设置属性的属性)
    properties
        carSimulator
        target_obstacle
        target_grad
        perturbed_change_amount = 1
        target_mirror_car
        State
        Mystep
        Myreward
        lastDistance

        target_angle

        change_angle_signal

        final_position

        success_count

        Mydone
        % 指定并初始化环境所需的属性

        myMap_list
    end

    properties(Access = protected)
        % 初始化内部标志，以指示回合是否结束
        IsDone = false
    end
    methods
        % 构造方法创建环境的实例
        % 根据需要修改类名和构造方法名称
        function this = MyEnvironment()
            % 初始化观察设置
            ObservationInfo = rlNumericSpec([25 1]);
            % 初始化动作设置
            ActionInfo = rlFiniteSetSpec([0 1 2 3 6]);  % 定义整数集合
            ActionInfo.Name = 'CartPole Action';

            % 以下行实现了强化学习环境的内建功能
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);
            this.success_count = 0;
            this.Mydone = false;
            figure;
            hold on;
            axis equal;
            grid on;
        end

        function DataLog(this)
            current_step = this.Mystep;
            myMap = containers.Map();
            myMap('step') = current_step;

            myData = this.storeData();
            myMap('data') = myData;

            this.myMap_list{end+1} = myMap;
        end

        function rand_num = random_0_to_2(this)
            rand_num = 2 * rand();  % rand()生成一个[0, 1)之间的随机数，乘以2使其范围在[0, 2)
            this.perturbed_change_amount = rand_num;
        end

        function [Observation,Reward,IsDone,Info] = step(this,Action)
            Info = [];
            add_reward_flag = 0;
            Reward = 0;

            IsDone = false;
            this.random_0_to_2();
            this.target_mirror_car = this.carSimulator.deepCopy();
            this.action_comparison_table(Action,this.target_mirror_car);
            this.target_mirror_car.update();
            if this.laws_physics_check(this.target_mirror_car,'physics_grand') && this.laws_physics_check(this.target_mirror_car,'physics_obstacle_point') || (this.change_angle_signal ~= 0 && Action == 6)
                add_reward_flag = 1;
                if this.laws_physics_check(this.target_mirror_car,'physics_obstacle')
                    this.action_comparison_table(Action,this.carSimulator);
                    this.carSimulator.update();
                end
            else
                Reward = -1000;
            end
            %Observation = this.get_state();
            if add_reward_flag == 1
                Reward = this.calculate_reward(Action);
            end
            this.Myreward = this.Myreward + Reward;
            Observation = this.normalize_f();
            %this.drawpic(true);
            if mod(this.Mystep, 50) == 0
                this.drawpic(true);
            end
            if this.Myreward < -30000
                IsDone = true;
            end
            %% 第一阶段停止
            % if this.check_touch(this.carSimulator.lc1_bottom,this.carSimulator.ssc1_bottom,this.target_obstacle.left_top_coordinate,2)
            %     IsDone = true;
            %     disp("任务完成停止");
            %     Reward = 1000 + 500/this.Mystep;
            %     this.Mydone = true;
            % end
            %%
            
            %% 第二阶段停止
            this.Mystep = this.Mystep + 1;
            this.DataLog();
            if this.euclidean_distance(this.final_position,this.carSimulator.com_coordinates) < 30.0
                Reward = 500000 + 500/this.Mystep;
                IsDone = true;
                this.success_count = this.success_count + 1;
                this.Mydone = true;
            end
            if this.success_count >= 10
                disp("训练完成！！！！！！！！！！！！！！！！！！！！");
            end
        end

        function normalize_data = normalize_f(this)
            raw_state = this.get_state();
            normalized_state = zeros(size(raw_state));
            x_max = 2700;   % 假设x坐标最大值
            y_max = 500;    % 假设y坐标最大值
            angle_max = 360;% 假设角度最大值
            distance_max = 3200;

            for i = 1:2:12
                normalized_state(i) = raw_state(i) / x_max;       % x坐标
                normalized_state(i+1) = raw_state(i+1) / y_max;  % y坐标
            end

            % 角度部分归一化（接下来6个元素）
            for i = 13:18
                normalized_state(i) = raw_state(i) / angle_max;
            end


            normalized_state(19) = raw_state(19) / distance_max;

            % 障碍物坐标归一化（最后4个元素）
            for i = 20:2:25
                normalized_state(i) = raw_state(i) / x_max;       % x坐标
                normalized_state(i+1) = raw_state(i+1) / y_max;  % y坐标
            end
            normalize_data = normalized_state;
        end

        function action_comparison_table(this, Action,target_car)
            switch Action
                case 0
                    target_car.change_deg(1,this.perturbed_change_amount);
                    target_car.change_deg(3,this.perturbed_change_amount);
                case 1
                    target_car.change_deg(-1,this.perturbed_change_amount);
                    target_car.change_deg(-3,this.perturbed_change_amount);
                case 2
                    target_car.change_deg(2,this.perturbed_change_amount);
                    target_car.change_deg(4,this.perturbed_change_amount);
                case 3
                    target_car.change_deg(-2,this.perturbed_change_amount);
                    target_car.change_deg(-4,this.perturbed_change_amount);
                    % case 4
                    %     target_car.change_deg(5,this.perturbed_change_amount);
                    % case 5
                    %     target_car.change_deg(-5,this.perturbed_change_amount);
                case 6
                    this.move(target_car,1)
                otherwise
                    % 如果 Action 不是 0 到 6 的值
                    disp('Invalid Action');
            end
        end

        % 重置环境到初始状态，并输出初始观察
        function InitialObservation = reset(this,obstacle_height, arm_length, small_arm_length, ...
                theta, phi, eta, zeta)
            if nargin < 2
                obstacle_height = 150;  %障碍物长度
            end
            if nargin < 3
                arm_length = 140;  %大臂长
            end
            if nargin < 4
                small_arm_length = 140;  %小臂长
            end
            if nargin < 5
                theta = 10;  %theta角度
            end
            if nargin < 6
                phi = 180;  %phi角度
            end
            if nargin < 7
                eta = 0;  %eta角度
            end
            if nargin < 8
                zeta = 185;  %zeta角度
            end

            this.initialize_object(obstacle_height, arm_length, small_arm_length, ...
                theta, phi, eta, zeta);
            InitialObservation = this.get_state;
            this.State = InitialObservation;
            this.drawpic(false);
            this.Mystep = 0;
            this.Myreward = 0;
            this.lastDistance = 20000;
            this.calculate_angle();
            this.change_angle_signal = 0;
            this.myMap_list = {};
        end

        function initialize_object(this, obstacle_height, arm_length, small_arm_length, ...
                theta, phi, eta, zeta)
            % Set default values if parameters are not provided

            large_radius = 50;
            small_radius = 30;
            small_small_radius = 15;
            length_car = 200;
            com_hight = 50;
            obstacle_x = 2000;
            com_coordinates = [0, com_hight];
            this.final_position = [2000, obstacle_height + com_hight];

            alpha = 0;
            beta = 0;

            this.carSimulator = newCarSimulator(large_radius, small_radius, small_small_radius, ...
                arm_length, small_arm_length, length_car, com_coordinates, theta, phi, eta, zeta, alpha, beta);

            this.target_obstacle = Obstacle([obstacle_x, 0], obstacle_height, 500);
            this.target_grad = [-100, 3000];
        end

        function myreward = calculate_reward(this,Action)
            % 计算欧式距离
            myreward = 0;
            distance = this.euclidean_distance(this.final_position,this.carSimulator.com_coordinates);


            % 计算目标角度与当前车角之间的差距
            angle_difference = abs(this.target_angle - this.carSimulator.theta);

            % 设定角度差距的奖励范围，可以根据需求进行调整
            angle_reward = max(0, 100 - angle_difference * 25);  % 假设每单位角度差减少10分

            % 如果当前距离比上次的小，则奖励增加100分
            if distance < this.lastDistance && this.change_angle_signal == 0
                myreward = 10 + angle_reward + 0.1*(this.lastDistance - distance)*100;  % 加上角度奖励
            else
                myreward = -100 + angle_reward;  % 加上角度奖励
            end
            if this.change_angle_signal == 0
                myreward = myreward - 20;
            end
            % 更新上次的距离
            if this.change_angle_signal == 1
                if Action == 3
                    myreward = 200;
                else
                    myreward = -50;
                end
            end

            if this.change_angle_signal == 2
                if Action == 2
                    myreward = 200;
                else
                    myreward = -50;
                end
            end

            myreward = myreward - 5;
            this.lastDistance = distance;
        end


        function is_valid = laws_physics_check(this, check_car,check_type)
            % 定义所有需要检查的部件坐标字段
            lc_fields = {'lc1_center', 'lc2_center', 'lc1_top', 'lc1_bottom', 'lc2_top', 'lc2_bottom'};
            sc_fields = {'sc1_center', 'sc2_center', 'sc1_top_1', 'sc1_bottom_1', 'sc2_top_1', 'sc2_bottom_1', ...
                'sc1_top_2', 'sc1_bottom_2', 'sc2_top_2', 'sc2_bottom_2'};
            ssc_fields = {'ssc1_center', 'ssc2_center', 'ssc1_top', 'ssc1_bottom', 'ssc2_top', 'ssc2_bottom'};
            all_fields = [lc_fields, sc_fields, ssc_fields];

            % 默认合法
            is_valid = true;

            % 根据检查类型决定检查内容
            if strcmp(check_type, 'physics_grand')
                % 遍历所有部件检查Y坐标是否不低于地面
                for i = 1:length(all_fields)
                    field = all_fields{i};
                    value = check_car.(field);
                    if length(value) >= 2 && value(2) < -2  % 地面Y坐标为-2
                        is_valid = false;
                        return;
                    end
                end
            elseif strcmp(check_type, 'physics_obstacle')
                % 获取障碍物边界坐标
                left = this.target_obstacle.left_top_coordinate(1);
                right = this.target_obstacle.right_top_coordinate(1);
                top = this.target_obstacle.left_top_coordinate(2);
                bottom = this.target_obstacle.left_bottom_coordinate(2);

                lc_fields = {'lc1_center', 'lc2_center', 'lc1_top', 'lc1_bottom', 'lc2_top', 'lc2_bottom'};
                sc_fields = {'sc1_center', 'sc2_center', 'sc1_top_1', 'sc1_bottom_1', 'sc2_top_1', 'sc2_bottom_1', ...
                    'sc1_top_2', 'sc1_bottom_2', 'sc2_top_2', 'sc2_bottom_2'};
                ssc_fields = {'ssc1_center', 'ssc2_center', 'ssc1_top', 'ssc1_bottom', 'ssc2_top', 'ssc2_bottom'};
                % 定义需要检查的部件中心点
                check_fields = [lc_fields, sc_fields, ssc_fields];

                % 检查各部件中心是否在障碍物区域内
                for i = 1:length(check_fields)
                    field = check_fields{i};
                    value = check_car.(field);
                    if length(value) >= 2
                        x = value(1);
                        y = value(2);
                        if x >= left && x <= right && y >= bottom && y <= top
                            is_valid = false;
                            return;
                        end
                    end
                end

            elseif strcmp(check_type, 'physics_obstacle_point')
                % 获取障碍物边界坐标
                left = this.target_obstacle.left_top_coordinate(1);
                right = this.target_obstacle.right_top_coordinate(1);
                top = this.target_obstacle.left_top_coordinate(2);
                bottom = this.target_obstacle.left_bottom_coordinate(2);
                lc_fields = {'lc1_center', 'lc2_center'};
                sc_fields = {'sc1_center', 'sc2_center'};
                ssc_fields = {'ssc1_center', 'ssc2_center'};
                % 定义需要检查的部件中心点
                check_fields = [lc_fields, sc_fields, ssc_fields];

                % 检查各部件中心是否在障碍物区域内
                for i = 1:length(check_fields)
                    field = check_fields{i};
                    value = check_car.(field);
                    if length(value) >= 2
                        x = value(1);
                        y = value(2);
                        if x >= left && x <= right && y >= bottom && y <= top
                            is_valid = false;
                            return;
                        end
                    end
                end
            else
                error('Invalid check type specified. Use "physics_grand" or "physics_obstacle".');
            end
        end

        function myData = storeData(this)
            % 存储数据到myData中
            myData = containers.Map();

            myData('大圆半径') = this.carSimulator.large_radius;
            myData('小圆半径') = this.carSimulator.small_radius;
            myData('二段小圆半径') = this.carSimulator.small_small_radius;
            myData('摆臂长度') = this.carSimulator.arm_length;
            myData('小摆臂长度') = this.carSimulator.small_arm_length;
            myData('小车长度') = this.carSimulator.length_car;

            myData('绝对位置') = this.carSimulator.com_coordinates;
            myData('质心位置') = this.carSimulator.XY_O;
            myData('前侧大圆中心A坐标') = this.carSimulator.lc1_center;
            myData('后侧大圆中心B坐标') = this.carSimulator.lc2_center;
            myData('前小轮坐标') = this.carSimulator.sc1_center;
            myData('后小轮坐标') = this.carSimulator.sc2_center;
            myData('前小小轮坐标') = this.carSimulator.ssc1_center;
            myData('后小小轮坐标') = this.carSimulator.ssc2_center;
            myData('中心线') = this.carSimulator.center_line;

            % 角度相关属性
            myData('theta') = this.carSimulator.theta;
            myData('phi') = this.carSimulator.phi;
            myData('eta') = this.carSimulator.eta;
            myData('zeta') = this.carSimulator.zeta;
            myData('alpha') = this.carSimulator.alpha;
            myData('beta') = this.carSimulator.beta;

            myData('完美theta') = this.carSimulator.per_theta;
            myData('完美phi') = this.carSimulator.per_phi;
            myData('完美eta') = this.carSimulator.per_eta;
            myData('完美zeta') = this.carSimulator.per_zeta;
            myData('完美alpha') = this.carSimulator.per_alpha;
            myData('完美beta') = this.carSimulator.per_beta;

            myData('扰动值') = this.perturbed_change_amount;
            myData('目标theta') = this.target_angle;

        end

        function calculate_angle(this)
            this.target_angle = asind(this.target_obstacle.height / (this.carSimulator.small_arm_length + this.carSimulator.arm_length+ this.carSimulator.large_radius));
        end

        function state = get_state(this)
            % 获取小车的位置信息
            lc1_center = this.carSimulator.lc1_center;
            lc2_center = this.carSimulator.lc2_center;
            sc1_center = this.carSimulator.sc1_center;
            sc2_center = this.carSimulator.sc2_center;
            ssc1_center = this.carSimulator.ssc1_center;
            ssc2_center = this.carSimulator.ssc2_center;

            com_c = this.carSimulator.com_coordinates;
            com_c_o = this.target_obstacle.coordinate;

            distance = this.euclidean_distance(this.final_position,this.carSimulator.com_coordinates);
            % 获取小车的角度信息
            theta = this.carSimulator.theta;
            phi = this.carSimulator.phi;
            eta = this.carSimulator.eta;
            zeta = this.carSimulator.zeta;
            alpha = this.carSimulator.alpha;
            beta = this.carSimulator.beta;

            % 获取障碍物位置
            obstacle_left_top = this.target_obstacle.left_top_coordinate;
            % 合并为状态向量
            state = [
                lc1_center(1), lc1_center(2), ...
                lc2_center(1), lc2_center(2), ...
                sc1_center(1), sc1_center(2), ...
                sc2_center(1), sc2_center(2), ...
                ssc1_center(1), ssc1_center(2), ...
                ssc2_center(1), ssc2_center(2), ...
                theta, phi, eta, zeta, alpha, beta, ...
                distance,...
                com_c(1),com_c(2),com_c_o(1),com_c_o(2),...
                obstacle_left_top(1), obstacle_left_top(2), ...
                ];
        end

        function drawpic(this,draw_mirral_flag)
            % 设置图形窗口大小（单位：像素）
            % set(gcf, 'Position', [100 100 1920 1080]);  % 1920x1080全屏分辨率

            clf;
            hold on;
            axis equal;
            grid on;

            this.drawCar(this.carSimulator,1);
            if draw_mirral_flag == true
                this.drawCar(this.target_mirror_car,2);
            end
            this.drawland();
            this.drawobstacle();


            drawnow;
            %pause(0.05);
        end

        function move(this,car,signal)
            % 根据是否接触障碍物决定移动模式
            if this.check_touch(car.lc1_bottom,car.ssc1_bottom,this.target_obstacle.left_top_coordinate,5) || this.check_touch(car.lc1_bottom,car.lc2_bottom,this.target_obstacle.left_top_coordinate,5) ||this.check_touch(car.lc1_center,car.ssc1_center,this.target_obstacle.left_top_coordinate,5)
                car.calculate_move(signal,0,1);  % 接触时移动模式0
                if car.com_coordinates(1) < this.target_obstacle.left_top_coordinate(1)
                    car.change_deg(5,1);
                    this.change_angle_signal = 2;
                else
                    car.change_deg(-5,1);
                    this.change_angle_signal = 1;
                end
            else
                car.calculate_move(signal,1,1);  % 非接触时移动模式1
                if car.com_coordinates(1) >= this.target_obstacle.left_top_coordinate(1) + 5 && (-6 > car.alpha || 6 < car.alpha)
                    car.change_deg(-5,1);
                    this.change_angle_signal = 1;
                end
            end
        end

        function touched = check_touch(this,A, B, P, tol)
            % 计算线段AB的向量
            AB = B - A;
            AP = P - A;
            t = dot(AP, AB) / dot(AB, AB);  % 计算投影参数

            % 确定最近点位置
            if t < 0
                closest = A;
            elseif t > 1
                closest = B;
            else
                closest = A + t * AB;
            end

            % 计算距离并判断是否小于容差
            dist = norm(P - closest);
            touched = dist <= tol;
        end

        function drawCar(this, car,colorScheme)
            % 处理可选参数
            if nargin < 2
                colorScheme = 1; % 默认颜色方案1
            end

            % 根据颜色方案设置颜色
            if colorScheme == 1
                edgeColor = 'k';        % 圆圈边缘颜色（黑色）
                mainLineColor = 'r';    % 主线条颜色（红色）
                secondaryLineColor = 'k'; % 次级线条颜色（黑色）
            elseif colorScheme == 2
                edgeColor = 'b';        % 圆圈边缘颜色（蓝色）
                mainLineColor = 'g';    % 主线条颜色（绿色）
                secondaryLineColor = 'm'; % 次级线条颜色（品红）
            else
                error('无效的颜色方案');
            end

            % 设置统一的线条宽度和类型
            lineWidth = 2;
            lineStyle = '--';

            % 绘制各个圆圈
            viscircles(car.lc1_center, car.large_radius, 'EdgeColor', edgeColor);
            viscircles(car.lc2_center, car.large_radius, 'EdgeColor', edgeColor);
            viscircles(car.sc1_center, car.small_radius, 'EdgeColor', edgeColor);
            viscircles(car.sc2_center, car.small_radius, 'EdgeColor', edgeColor);
            viscircles(car.ssc1_center, car.small_small_radius, 'EdgeColor', edgeColor);
            viscircles(car.ssc2_center, car.small_small_radius, 'EdgeColor', edgeColor);

            % 绘制连线
            line([car.sc1_bottom_1(1), car.lc1_bottom(1)], [car.sc1_bottom_1(2), car.lc1_bottom(2)],...
                'Color', mainLineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);
            line([car.sc1_top_1(1), car.lc1_top(1)], [car.sc1_top_1(2), car.lc1_top(2)],...
                'Color', mainLineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);

            line([car.lc1_bottom_abs(1), car.lc2_bottom_abs(1)], [car.lc1_bottom_abs(2), car.lc2_bottom_abs(2)],...
                'Color', secondaryLineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);
            line([car.lc1_top_abs(1), car.lc2_top_abs(1)], [car.lc1_top_abs(2), car.lc2_top_abs(2)],...
                'Color', secondaryLineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);

            line([car.sc2_bottom_1(1), car.lc2_bottom(1)], [car.sc2_bottom_1(2), car.lc2_bottom(2)],...
                'Color', mainLineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);
            line([car.sc2_top_1(1), car.lc2_top(1)], [car.sc2_top_1(2), car.lc2_top(2)],...
                'Color', mainLineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);

            line([car.ssc2_bottom(1), car.sc2_bottom_2(1)], [car.ssc2_bottom(2), car.sc2_bottom_2(2)],...
                'Color', mainLineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);
            line([car.ssc2_top(1), car.sc2_top_2(1)], [car.ssc2_top(2), car.sc2_top_2(2)],...
                'Color', mainLineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);

            line([car.ssc1_bottom(1), car.sc1_bottom_2(1)], [car.ssc1_bottom(2), car.sc1_bottom_2(2)],...
                'Color', mainLineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);
            line([car.ssc1_top(1), car.sc1_top_2(1)], [car.ssc1_top(2), car.sc1_top_2(2)],...
                'Color', mainLineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);
        end

        function drawland(this)
            % 创建x坐标从0到3000
            x = -100:3000;
            % 对应的y坐标为0
            y = zeros(size(x));

            % 绘制图形
            plot(x, y, 'LineWidth', 2);  % 可以设置线条宽度
        end

        function drawobstacle(this)
            % Ensure the coordinates are calculated
            if isempty(this.target_obstacle.left_top_coordinate) || isempty(this.target_obstacle.left_bottom_coordinate) ...
                    || isempty(this.target_obstacle.right_top_coordinate) || isempty(this.target_obstacle.right_bottom_coordinate)
                this.target_obstacle.calculate_top_bottom();
            end

            % Extract the coordinates
            x_coords = [this.target_obstacle.left_top_coordinate(1), ...
                this.target_obstacle.right_top_coordinate(1), ...
                this.target_obstacle.right_bottom_coordinate(1), ...
                this.target_obstacle.left_bottom_coordinate(1)];
            y_coords = [this.target_obstacle.left_top_coordinate(2), ...
                this.target_obstacle.right_top_coordinate(2), ...
                this.target_obstacle.right_bottom_coordinate(2), ...
                this.target_obstacle.left_bottom_coordinate(2)];

            % Draw the this.target_obstacle
            fill(x_coords, y_coords, 'r', 'FaceAlpha', 0.5); % Red color with transparency
            hold on; % Retain the plot if drawing multiple this.target_obstacles

            % Add labels or additional visuals if necessary
            plot(this.target_obstacle.coordinate(1), this.target_obstacle.coordinate(2), 'bo', 'MarkerSize', 8); % Mark the center
            hold off;
        end

        function distance = euclidean_distance(this,point1, point2)
            % point1 = [x1, y1], point2 = [x2, y2]
            % 计算两个点之间的欧式距离

            % 提取坐标
            x1 = point1(1);
            y1 = point1(2);
            x2 = point2(1);
            y2 = point2(2);

            % 计算欧式距离
            distance = sqrt((x2 - x1)^2 + (y2 - y1)^2);
        end
    end
end
