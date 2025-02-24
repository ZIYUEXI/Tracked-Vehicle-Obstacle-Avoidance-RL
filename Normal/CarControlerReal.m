classdef CarControlerReal < handle
    properties
        target_car  % 目标车辆对象
        target_mirror_car % 镜像对象
        target_obstacle  % 目标障碍物对象
        target_grand % 地面坐标

        target_angle_1

        signal_step
    end
    methods
        % 构造函数，初始化成员变量
        function obj = CarControlerReal(target_car, target_obstacle, target_grand)
            obj.signal_step = 0;

            obj.target_car = target_car;
            obj.target_obstacle = target_obstacle;
            obj.target_grand = target_grand;
            disp(obj.target_obstacle)

            obj.target_angle_1 = asind(obj.target_obstacle.height / (obj.target_car.small_arm_length + obj.target_car.arm_length+ obj.target_car.large_radius));


        end

        function step_1(obj)
            obj.target_mirror_car = obj.target_car.deepCopy();
            obj.move(obj.target_mirror_car,1)
            result = obj.compareData(obj.target_angle_1,obj.target_mirror_car.theta);
            if strcmp(result, 'greater')
                % 如果目标角度大，则增加车的角度
                obj.target_mirror_car.change_deg(1);
                obj.target_mirror_car.change_deg(3);
            else
                % 如果目标角度小，则减少车的角度
                obj.target_mirror_car.change_deg(-1);
                obj.target_mirror_car.change_deg(-3);
            end

            obj.target_mirror_car.update();

            %实际操作做合法判断
            if obj.laws_physics_grand() == true && obj.laws_physics_obstacle() == true
                obj.move(obj.target_car,1);
                result = obj.compareData(obj.target_angle_1,obj.target_car.theta);
                if strcmp(result, 'greater')
                    % 如果目标角度大，则增加车的角度
                    obj.target_car.change_deg(1);
                    obj.target_car.change_deg(3);
                else
                    % 如果目标角度小，则减少车的角度
                    obj.target_car.change_deg(-1);
                    obj.target_car.change_deg(-3);
                end

                if obj.check_touch(obj.target_car.lc1_bottom,obj.target_car.ssc1_bottom,obj.target_obstacle.left_top_coordinate,2) == true
                    obj.signal_step = 1;
                    disp("step1结束")
                end
            end
        end

        function step_2(obj)
            obj.target_mirror_car = obj.target_car.deepCopy();
            if obj.check_touch(obj.target_car.lc1_bottom,obj.target_car.ssc1_bottom,obj.target_obstacle.left_top_coordinate,2) == true
                obj.target_mirror_car.change_deg(5);
                if obj.laws_physics_grand() == true && obj.laws_physics_obstacle() == true
                    obj.target_car.change_deg(5);
                end
            end
            obj.move(obj.target_mirror_car,1)
            obj.move(obj.target_car,1)

            obj.target_mirror_car.change_deg(2);
            obj.target_mirror_car.change_deg(4);
            if obj.laws_physics_grand() == true && obj.laws_physics_obstacle() == true
                obj.target_car.change_deg(2);
                obj.target_car.change_deg(4);
            end

            result = compareData(obj, obj.target_car.lc1_center(1), obj.target_obstacle.left_top_coordinate(1));
            if strcmp(result, 'greater')
                obj.signal_step = 2;
                disp("step2结束")
            end
        end

        function step_3(obj)
            obj.target_mirror_car = obj.target_car.deepCopy();
            obj.target_mirror_car.change_deg(-5);

            if obj.laws_physics_grand() == true && obj.laws_physics_obstacle() == true && (obj.target_car.alpha > 1 || obj.target_car.alpha < -1)
                obj.target_car.change_deg(-5);
            end
            
            obj.target_mirror_car = obj.target_car.deepCopy();
            obj.move(obj.target_mirror_car,1)
            obj.move(obj.target_car,1)

            obj.target_mirror_car.change_deg(2);
            obj.target_mirror_car.change_deg(4);
            if obj.laws_physics_grand() == true && obj.laws_physics_obstacle() == true
                obj.target_car.change_deg(2);
                obj.target_car.change_deg(4);
            end

            if obj.target_car.com_coordinates(1) >= obj.target_obstacle.left_top_coordinate(1)
                obj.signal_step = 3;
                disp("step3结束")
            end
        end

        function step_4(obj)
            obj.target_mirror_car = obj.target_car.deepCopy();
            obj.target_mirror_car.change_deg(-5);

            if obj.laws_physics_grand() == true && obj.laws_physics_obstacle() == true && (obj.target_car.alpha > 1 || obj.target_car.alpha < -1)
                obj.target_car.change_deg(-5);
            end

            obj.target_mirror_car = obj.target_car.deepCopy();
            obj.move(obj.target_mirror_car,1)
            obj.move(obj.target_car,1)

            obj.target_mirror_car.change_deg(-2);
            obj.target_mirror_car.change_deg(-4);
            if obj.laws_physics_grand() == true && obj.laws_physics_obstacle() == true
                obj.target_car.change_deg(-2);
                obj.target_car.change_deg(-4);
            end

            if obj.target_car.com_coordinates(1) >= obj.target_obstacle.coordinate(1)
                obj.signal_step = 4;
                disp("step4结束")
            end
        end

        % 更新函数
        function done = update(obj)
            done = false;
            if obj.signal_step == 0
                obj.step_1()
            end
            if obj.signal_step == 1
                obj.step_2()
            end

            if obj.signal_step == 2
                obj.step_3()
            end
            if obj.signal_step == 3
                obj.step_4()
            end
            if obj.signal_step == 4
                done = true;
            end
        end



        function move(obj,car,signal)
            if obj.check_touch(obj.target_mirror_car.lc1_bottom,obj.target_mirror_car.ssc1_bottom,obj.target_obstacle.left_top_coordinate,2) == true
                car.calculate_move(signal,0,1);
            else
                car.calculate_move(signal,1,1);
            end
        end

        % 物理定律相关的函数
        function is_valid = laws_physics_grand(obj)
            % 检查所有 lc、sc、ssc 坐标的第二个值是否大于等于-2
            lc_fields = {'lc1_center', 'lc2_center', 'lc1_top', 'lc1_bottom', 'lc2_top', 'lc2_bottom'};
            sc_fields = {'sc1_center', 'sc2_center', 'sc1_top_1', 'sc1_bottom_1', 'sc2_top_1', 'sc2_bottom_1', ...
                'sc1_top_2', 'sc1_bottom_2', 'sc2_top_2', 'sc2_bottom_2'};
            ssc_fields = {'ssc1_center', 'ssc2_center', 'ssc1_top', 'ssc1_bottom', 'ssc2_top', 'ssc2_bottom'};

            % 合并所有需要检查的字段
            all_fields = [lc_fields, sc_fields, ssc_fields];

            is_valid = true;  % 默认是有效的

            % 遍历每个字段并检查第二个值是否大于等于-2
            for i = 1:length(all_fields)
                field = all_fields{i};
                value = obj.target_mirror_car.(field);  % 获取字段值
                if length(value) >= 2 && value(2) < -2  % 检查第二个值是否小于0
                    is_valid = false;  % 如果发现无效值，设置为false
                    return;  % 直接返回，节省运算
                end
            end
        end

        function is_valid = laws_physics_obstacle(obj)
            % 定义障碍物的坐标范围
            left = obj.target_obstacle.left_top_coordinate(1);  % 左侧 x 坐标
            right = obj.target_obstacle.right_top_coordinate(1);  % 右侧 x 坐标
            top = obj.target_obstacle.left_top_coordinate(2);  % 顶部 y 坐标
            bottom = obj.target_obstacle.left_bottom_coordinate(2);  % 底部 y 坐标

            % 获取目标车辆相关的坐标字段（如lc1_center, sc1_center等）
            lc_fields = {'lc1_center', 'lc2_center'};
            sc_fields = {'sc1_center', 'sc2_center'};
            ssc_fields = {'ssc1_center', 'ssc2_center'};

            % 合并所有需要检查的字段
            all_fields = [lc_fields, sc_fields, ssc_fields];

            % 默认有效
            is_valid = true;

            % 遍历每个坐标字段，检查是否越界
            for i = 1:length(all_fields)
                field = all_fields{i};
                value = obj.target_mirror_car.(field);  % 获取坐标值

                if length(value) >= 2
                    x = value(1);
                    y = value(2);

                    % 检查坐标是否在障碍物的范围内
                    if x >= left && x <= right && y >= bottom && y <= top
                        is_valid = false;  % 如果在范围内，表示非法
                        return;  % 提前返回，节省计算
                    end
                end
            end
        end

        function touched = check_touch(obj,A, B, P, tol)
            % A, B: [x, y] 坐标，定义线段端点
            % P: [x, y] 坐标，固定点
            % tol: 距离容差，小于此距离则认为接触

            % 线段向量和投影参数计算
            AB = B - A;
            AP = P - A;
            t = dot(AP, AB) / dot(AB, AB);

            if t < 0
                % 投影在线段外 A 侧
                closest = A;
            elseif t > 1
                % 投影在线段外 B 侧
                closest = B;
            else
                % 投影在线段内部
                closest = A + t * AB;
            end

            % 点到线段距离
            dist = norm(P - closest);

            % 判断是否接触
            touched = dist <= tol;
        end

        function result = compareData(obj, main_data, target_data)
            % 比较两个数据的大小关系
            % 输入:
            %   main_data - 主数据
            %   target_data - 目标数据
            % 输出:
            %   result - 比较结果，'greater'表示main_data大于target_data，
            %            'less'表示main_data小于target_data，
            %            'equal'表示两者相等

            if main_data > target_data
                result = 'greater';
            elseif main_data < target_data
                result = 'less';
            else
                result = 'equal';
            end
        end

    end
end
