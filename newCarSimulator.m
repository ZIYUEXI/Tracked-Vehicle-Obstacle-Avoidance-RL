classdef newCarSimulator < handle
    % 定义一个类 newCarSimulator，继承自 handle 类，表示一个新的小车模拟器

    properties
        % 定义类的属性，分组说明属性的用途

        % 基本参数属性
        large_radius       % 大圆半径
        small_radius       % 小圆半径
        small_small_radius % 二段小圆半径
        arm_length         % 摆臂长度 l2
        small_arm_length   % 小摆臂长度 l3
        length_car         % 小车长度 l1

        body_mass %小车体质量
        arm_mass %机械臂质量
        small_arm_mass %小机械臂质量

        % 位置属性
        com_coordinates %绝对位置
        XY_O               % 质心位置
        lc1_center         % 左侧大圆中心 A 坐标
        lc2_center         % 右侧大圆中心 B 坐标
        sc1_center         % C 坐标
        sc2_center         % D 坐标
        ssc1_center        % E 坐标
        ssc2_center        % F 坐标
        center_line

        % 绘制属性
        plotHandles = []   % 图形对象句柄
        lc1_top
        lc1_bottom
        lc2_top
        lc2_bottom
        sc1_top_1
        sc1_bottom_1
        sc2_top_1
        sc2_bottom_1
        sc1_top_2
        sc1_bottom_2
        sc2_top_2
        sc2_bottom_2
        ssc1_top
        ssc1_bottom
        ssc2_top
        ssc2_bottom
        lc1_top_abs
        lc1_bottom_abs
        lc2_top_abs
        lc2_bottom_abs

        % 目标对象属性
        target_signal
        target_height
        target_x

        %角度属性
        theta
        phi
        eta
        zeta
        alpha
        beta

        %完美角度
        per_theta
        per_phi
        per_eta
        per_zeta
        per_alpha
        per_beta


    end

    methods
        % 定义类的方法
        function obj = newCarSimulator(large_r, small_r, small_small_r, arm_len, small_arm_len, ...
                length_car, com_coordinates, theta, phi, eta, zeta, alpha, beta)
            % 构造函数，初始化对象的属性
            % 参数说明：
            %   large_r           - 大圆半径
            %   small_r           - 小圆半径
            %   small_small_r     - 二段小圆半径
            %   arm_len           - 摆臂长度
            %   small_arm_len     - 小摆臂长度
            %   length_car        - 小车长度
            %   com_coordinates   - 质心初始坐标
            obj.theta = theta;
            obj.phi = phi;
            obj.eta = eta;
            obj.zeta = zeta;
            obj.alpha = alpha;
            obj.beta = beta;
            obj.per_theta = theta;
            obj.per_phi = phi;
            obj.per_eta = eta;
            obj.per_zeta = zeta;
            obj.per_alpha = alpha;
            obj.per_beta = beta;
            obj.com_coordinates = com_coordinates;
            obj.length_car = length_car;
            obj.large_radius = large_r; % 初始化大圆半径
            obj.small_radius = small_r; % 初始化小圆半径
            obj.small_small_radius = small_small_r; % 初始化二段小圆半径
            obj.arm_length = arm_len; % 初始化摆臂长度
            obj.small_arm_length = small_arm_len; % 初始化小摆臂长度

            half_length = obj.length_car / 2; % 计算小车长度的一半
            % 初始左侧和右侧大圆中心位置
            obj.lc1_center = [obj.com_coordinates(1) + half_length, com_coordinates(2)];
            obj.lc2_center = [obj.com_coordinates(1) - half_length, com_coordinates(2)];

            % 在这里对左侧大圆进行旋转并保持y轴不变
            % 计算alpha对应的角度
            rotation_matrix = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];

            % 对lc1_center进行旋转，只改变x轴坐标
            displacement = obj.lc1_center - com_coordinates; % 计算相对于质心的偏移
            rotated_displacement = rotation_matrix * displacement'; % 旋转偏移量
            obj.lc1_center = com_coordinates + rotated_displacement'; % 更新lc1_center位置

            obj.target_signal = false;
            obj.calculate_particle();

            obj.sc1_center = obj.calculate_sc(obj.lc1_center, obj.arm_length, obj.theta);
            obj.sc2_center = obj.calculate_sc(obj.lc2_center, obj.arm_length, obj.phi);
            obj.ssc1_center = obj.calculate_sc(obj.sc1_center, obj.small_arm_length, obj.eta);
            obj.ssc2_center = obj.calculate_sc(obj.sc2_center, obj.small_arm_length, obj.zeta);

            obj.calculate_top_bottom();
        end


        function show_parameter(obj)

            fprintf('large_radius = %.2f\n', obj.large_radius);
            fprintf('small_radius = %.2f\n', obj.small_radius);
            fprintf('small_small_radius = %.2f\n', obj.small_small_radius);
            fprintf('arm_length = %.2f\n', obj.arm_length);
            fprintf('small_arm_length = %.2f\n', obj.small_arm_length);
            fprintf('length_car = %.2f\n', obj.length_car);

            fprintf('body_mass = %.2f\n', obj.body_mass);
            fprintf('arm_mass = %.2f\n', obj.arm_mass);
            fprintf('small_arm_mass = %.2f\n', obj.small_arm_mass);

            fprintf('XY_O = [%s]\n', num2str(obj.XY_O));
            fprintf('lc1_center = [%s]\n', num2str(obj.lc1_center));
            fprintf('lc2_center = [%s]\n', num2str(obj.lc2_center));
            fprintf('sc1_center = [%s]\n', num2str(obj.sc1_center));
            fprintf('sc2_center = [%s]\n', num2str(obj.sc2_center));
            fprintf('ssc1_center = [%s]\n', num2str(obj.ssc1_center));
            fprintf('ssc2_center = [%s]\n', num2str(obj.ssc2_center));
            fprintf('center_line = [%s]\n', num2str(obj.center_line));

            fprintf('lc1_top = [%s]\n', num2str(obj.lc1_top));
            fprintf('lc1_bottom = [%s]\n', num2str(obj.lc1_bottom));
            fprintf('lc2_top = [%s]\n', num2str(obj.lc2_top));
            fprintf('lc2_bottom = [%s]\n', num2str(obj.lc2_bottom));

            fprintf('sc1_top_1 = [%s]\n', num2str(obj.sc1_top_1));
            fprintf('sc1_bottom_1 = [%s]\n', num2str(obj.sc1_bottom_1));
            fprintf('sc2_top_1 = [%s]\n', num2str(obj.sc2_top_1));
            fprintf('sc2_bottom_1 = [%s]\n', num2str(obj.sc2_bottom_1));

            fprintf('sc1_top_2 = [%s]\n', num2str(obj.sc1_top_2));
            fprintf('sc1_bottom_2 = [%s]\n', num2str(obj.sc1_bottom_2));
            fprintf('sc2_top_2 = [%s]\n', num2str(obj.sc2_top_2));
            fprintf('sc2_bottom_2 = [%s]\n', num2str(obj.sc2_bottom_2));

            fprintf('ssc1_top = [%s]\n', num2str(obj.ssc1_top));
            fprintf('ssc1_bottom = [%s]\n', num2str(obj.ssc1_bottom));
            fprintf('ssc2_top = [%s]\n', num2str(obj.ssc2_top));
            fprintf('ssc2_bottom = [%s]\n', num2str(obj.ssc2_bottom));

            fprintf('lc1_top_abs = [%s]\n', num2str(obj.lc1_top_abs));
            fprintf('lc1_bottom_abs = [%s]\n', num2str(obj.lc1_bottom_abs));
            fprintf('lc2_top_abs = [%s]\n', num2str(obj.lc2_top_abs));
            fprintf('lc2_bottom_abs = [%s]\n', num2str(obj.lc2_bottom_abs));

            fprintf('target_signal = %.2f\n', obj.target_signal);
            fprintf('target_height = %.2f\n', obj.target_height);
            fprintf('target_x = %.2f\n', obj.target_x);

            fprintf('theta = %.2f\n', obj.theta);
            fprintf('phi = %.2f\n', obj.phi);
            fprintf('eta = %.2f\n', obj.eta);
            fprintf('zeta = %.2f\n', obj.zeta);
            fprintf('alpha = %.2f\n', obj.alpha);
            fprintf('beta = %.2f\n', obj.beta);
        end

        function calculate_particle(obj)
            obj.XY_O = [obj.lc2_center(1) + obj.length_car / 2,obj.lc2_center(2)];
        end

        function targetsc = calculate_sc(obj, targetupper, targetarm, targetdeg)
            targetsc = targetupper + targetarm * [cosd(targetdeg), sind(targetdeg)];
        end

        function calculate_top_bottom(obj)

            lc1_ctr  = obj.lc1_center;
            lc2_ctr  = obj.lc2_center;

            lr   = obj.large_radius;
            sr   = obj.small_radius;
            ssr  = obj.small_small_radius;


            obj.lc1_top    = lc1_ctr + lr * [cosd(obj.theta + 90), sind(obj.theta + 90)];
            obj.lc1_bottom = lc1_ctr + lr * [cosd(obj.theta - 90), sind(obj.theta - 90)];
            obj.lc2_top    = lc2_ctr + lr * [cosd(obj.phi + 90), sind(obj.phi + 90)];
            obj.lc2_bottom = lc2_ctr + lr * [cosd(obj.phi - 90), sind(obj.phi - 90)];

            obj.sc1_top_1    = obj.sc1_center + sr * [cosd(obj.theta + 90), sind(obj.theta + 90)];
            obj.sc1_bottom_1 = obj.sc1_center + sr * [cosd(obj.theta - 90), sind(obj.theta - 90)];
            obj.sc2_top_1    = obj.sc2_center + sr * [cosd(obj.phi + 90), sind(obj.phi + 90)];
            obj.sc2_bottom_1 = obj.sc2_center + sr * [cosd(obj.phi - 90), sind(obj.phi - 90)];

            obj.sc1_top_2    = obj.sc1_center + sr * [cosd(obj.eta + 90), sind(obj.eta + 90)];
            obj.sc1_bottom_2 = obj.sc1_center + sr * [cosd(obj.eta - 90), sind(obj.eta - 90)];
            obj.sc2_top_2    = obj.sc2_center + sr * [cosd(obj.zeta + 90), sind(obj.zeta + 90)];
            obj.sc2_bottom_2 = obj.sc2_center + sr * [cosd(obj.zeta - 90), sind(obj.zeta - 90)];

            obj.ssc1_top    = obj.ssc1_center + ssr * [cosd(obj.eta + 90), sind(obj.eta + 90)];
            obj.ssc1_bottom = obj.ssc1_center + ssr * [cosd(obj.eta - 90), sind(obj.eta - 90)];
            obj.ssc2_top    = obj.ssc2_center + ssr * [cosd(obj.zeta + 90), sind(obj.zeta + 90)];
            obj.ssc2_bottom = obj.ssc2_center + ssr * [cosd(obj.zeta - 90), sind(obj.zeta - 90)];

            obj.lc1_top_abs    = lc1_ctr + [0, lr];
            obj.lc1_bottom_abs = lc1_ctr + [0, -lr];
            obj.lc2_top_abs    = lc2_ctr + [0, lr];
            obj.lc2_bottom_abs = lc2_ctr + [0, -lr];
        end

        function update(obj)
            % 更新所有坐标点的位置，根据最新的 com_coordinates
            half_length = obj.length_car / 2; % 小车长度的一半

            % 初始设定lc1_center和lc2_center的位置
            obj.lc1_center = [obj.com_coordinates(1) + half_length, obj.com_coordinates(2)];
            obj.lc2_center = [obj.com_coordinates(1) - half_length, obj.com_coordinates(2)];

            % 以lc2_center为旋转中心，旋转lc1_center，但保持lc2_center不动
            rotation_matrix_alpha = [cosd(obj.alpha), -sind(obj.alpha);
                sind(obj.alpha),  cosd(obj.alpha)];
            displacement_alpha = obj.lc1_center - obj.lc2_center;
            rotated_displacement_alpha = rotation_matrix_alpha * displacement_alpha';
            obj.lc1_center = obj.lc2_center + rotated_displacement_alpha';

            % 接下来，以lc1_center为旋转中心，以角度obj.beta旋转lc2_center
            rotation_matrix_beta = [cosd(obj.beta), -sind(obj.beta);
                sind(obj.beta),  cosd(obj.beta)];

            % 计算lc2_center相对于旋转中心lc1_center的偏移
            displacement_beta = obj.lc2_center - obj.lc1_center;

            % 对偏移进行旋转
            rotated_displacement_beta = rotation_matrix_beta * displacement_beta';

            % 更新lc2_center的位置，以lc1_center为基准，加上旋转后的偏移
            obj.lc2_center = obj.lc1_center + rotated_displacement_beta';



            % 更新目标点的位置
            obj.sc1_center = obj.calculate_sc(obj.lc1_center, obj.arm_length, obj.theta);
            obj.sc2_center = obj.calculate_sc(obj.lc2_center, obj.arm_length, obj.phi);
            obj.ssc1_center = obj.calculate_sc(obj.sc1_center, obj.small_arm_length, obj.eta);
            obj.ssc2_center = obj.calculate_sc(obj.sc2_center, obj.small_arm_length, obj.zeta);

            % 更新各个顶点位置
            obj.calculate_top_bottom();
        end

        function calculate_move(obj, move_signal,tuning_x_coefficient,tuning_y_coefficient)
            distance = 4;  % 运动的步长，可以根据需要调整

            % 根据 alpha 计算 x 和 y 的增量
            delta_x = distance * cosd(obj.alpha) * tuning_x_coefficient;
            delta_y = distance * sind(obj.alpha) * tuning_y_coefficient;

            switch move_signal
                case 1
                    % 沿着 alpha 方向移动
                    obj.com_coordinates(1) = obj.com_coordinates(1) + delta_x;
                    obj.com_coordinates(2) = obj.com_coordinates(2) + delta_y;
                case 2
                    % 反方向运动：移动量取反
                    obj.com_coordinates(1) = obj.com_coordinates(1) - delta_x;
                    obj.com_coordinates(2) = obj.com_coordinates(2) - delta_y;
                otherwise
                    % 对于未定义的 move_signal 值，可以选择添加处理逻辑
                    % disp('Invalid move_signal value');
            end
        end


        function change_deg(obj, change_signal, perturbation_factor)
            % change_signal determines which property to change and the operation
            % positive values increase the property, negative values decrease it
            % perturbation_factor is an optional scalar between 0 and 1
            % 当 perturbation_factor 较大时，扰动值更接近2，较小时更接近0
            perturbed_change_amount = perturbation_factor;

            switch abs(change_signal)
                case 1
                    % Change theta
                    if change_signal > 0
                        obj.theta = obj.theta + perturbed_change_amount;
                        obj.per_theta = obj.per_theta + 1;
                    else
                        obj.theta = obj.theta - perturbed_change_amount;
                        obj.per_theta = obj.per_theta - 1;
                    end
                case 2
                    % Change phi
                    if change_signal > 0
                        obj.phi = obj.phi + perturbed_change_amount;
                        obj.per_phi = obj.per_phi + 1;
                    else
                        obj.phi = obj.phi - perturbed_change_amount;
                        obj.per_phi = obj.per_phi - 1;
                    end
                case 3
                    % Change eta
                    if change_signal > 0
                        obj.eta = obj.eta + perturbed_change_amount;
                        obj.per_eta = obj.per_eta + 1;
                    else
                        obj.eta = obj.eta - perturbed_change_amount;
                        obj.per_eta = obj.per_eta - 1;
                    end
                case 4
                    % Change zeta
                    if change_signal > 0
                        obj.zeta = obj.zeta + perturbed_change_amount;
                        obj.per_zeta = obj.per_zeta + 1;
                    else
                        obj.zeta = obj.zeta - perturbed_change_amount;
                        obj.per_zeta = obj.per_zeta - 1;
                    end
                case 5
                    % Change alpha
                    if change_signal > 0
                        obj.alpha = obj.alpha + perturbed_change_amount;
                        obj.per_alpha = obj.per_alpha + 1;
                    else
                        obj.alpha = obj.alpha - perturbed_change_amount;
                        obj.per_alpha = obj.per_alpha - 1;
                    end
                case 6
                    % Change beta
                    if change_signal > 0
                        obj.beta = obj.beta + perturbed_change_amount;
                        obj.per_beta = obj.per_beta + 1;
                    else
                        obj.beta = obj.beta - perturbed_change_amount;
                        obj.per_beta = obj.per_beta - 1;
                    end
                otherwise
                    warning('Invalid change_signal. No property was modified.');
            end
        end

        function copyObj = deepCopy(obj)
            % 创建新对象作为副本
            copyObj = newCarSimulator(...
                obj.large_radius, ...
                obj.small_radius, ...
                obj.small_small_radius, ...
                obj.arm_length, ...
                obj.small_arm_length, ...
                obj.length_car, ...
                obj.com_coordinates, ...
                obj.theta, ...
                obj.phi, ...
                obj.eta, ...
                obj.zeta, ...
                obj.alpha, ...
                obj.beta);

            % 复制其他属性
            copyObj.body_mass = obj.body_mass;
            copyObj.arm_mass = obj.arm_mass;
            copyObj.small_arm_mass = obj.small_arm_mass;
            copyObj.XY_O = obj.XY_O;
            copyObj.lc1_center = obj.lc1_center;
            copyObj.lc2_center = obj.lc2_center;
            copyObj.sc1_center = obj.sc1_center;
            copyObj.sc2_center = obj.sc2_center;
            copyObj.ssc1_center = obj.ssc1_center;
            copyObj.ssc2_center = obj.ssc2_center;
            copyObj.center_line = obj.center_line;
            copyObj.plotHandles = obj.plotHandles; % 这里假设 plotHandles 是独立的，若涉及到句柄对象需要特别处理
            copyObj.lc1_top = obj.lc1_top;
            copyObj.lc1_bottom = obj.lc1_bottom;
            copyObj.lc2_top = obj.lc2_top;
            copyObj.lc2_bottom = obj.lc2_bottom;
            copyObj.sc1_top_1 = obj.sc1_top_1;
            copyObj.sc1_bottom_1 = obj.sc1_bottom_1;
            copyObj.sc2_top_1 = obj.sc2_top_1;
            copyObj.sc2_bottom_1 = obj.sc2_bottom_1;
            copyObj.sc1_top_2 = obj.sc1_top_2;
            copyObj.sc1_bottom_2 = obj.sc1_bottom_2;
            copyObj.sc2_top_2 = obj.sc2_top_2;
            copyObj.sc2_bottom_2 = obj.sc2_bottom_2;
            copyObj.ssc1_top = obj.ssc1_top;
            copyObj.ssc1_bottom = obj.ssc1_bottom;
            copyObj.ssc2_top = obj.ssc2_top;
            copyObj.ssc2_bottom = obj.ssc2_bottom;
            copyObj.lc1_top_abs = obj.lc1_top_abs;
            copyObj.lc1_bottom_abs = obj.lc1_bottom_abs;
            copyObj.lc2_top_abs = obj.lc2_top_abs;
            copyObj.lc2_bottom_abs = obj.lc2_bottom_abs;
            copyObj.target_signal = obj.target_signal;
            copyObj.target_height = obj.target_height;
            copyObj.target_x = obj.target_x;
        end

    end
end
