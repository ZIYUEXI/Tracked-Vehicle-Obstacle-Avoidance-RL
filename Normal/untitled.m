function drawCar(car)
    % 设置统一的颜色和线条类型
    lineColor = 'r';  % 统一的颜色为红色
    lineWidth = 2;    % 统一的线宽
    lineStyle = '--'; % 统一的线条类型为虚线

    % 绘制各个圆圈
    viscircles(car.lc1_center, car.large_radius, 'EdgeColor', 'k');
    viscircles(car.lc2_center, car.large_radius, 'EdgeColor', 'k');
    viscircles(car.sc1_center, car.small_radius, 'EdgeColor', 'k');
    viscircles(car.sc2_center, car.small_radius, 'EdgeColor', 'k');
    viscircles(car.ssc1_center, car.small_small_radius, 'EdgeColor', 'k');
    viscircles(car.ssc2_center, car.small_small_radius, 'EdgeColor', 'k');

    % 绘制红色连线（使用统一的线条样式）
    line([car.sc1_bottom_1(1), car.lc1_bottom(1)], [car.sc1_bottom_1(2), car.lc1_bottom(2)], 'Color', lineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);
    line([car.sc1_top_1(1), car.lc1_top(1)], [car.sc1_top_1(2), car.lc1_top(2)], 'Color', lineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);

    line([car.lc1_bottom_abs(1), car.lc2_bottom_abs(1)], [car.lc1_bottom_abs(2), car.lc2_bottom_abs(2)], 'Color', 'black', 'LineWidth', lineWidth, 'LineStyle', lineStyle);
    line([car.lc1_top_abs(1), car.lc2_top_abs(1)], [car.lc1_top_abs(2), car.lc2_top_abs(2)], 'Color', 'black', 'LineWidth', lineWidth, 'LineStyle', lineStyle);

    line([car.sc2_bottom_1(1), car.lc2_bottom(1)], [car.sc2_bottom_1(2), car.lc2_bottom(2)], 'Color', lineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);
    line([car.sc2_top_1(1), car.lc2_top(1)], [car.sc2_top_1(2), car.lc2_top(2)], 'Color', lineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);

    line([car.ssc2_bottom(1), car.sc2_bottom_2(1)], [car.ssc2_bottom(2), car.sc2_bottom_2(2)], 'Color', lineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);
    line([car.ssc2_top(1), car.sc2_top_2(1)], [car.ssc2_top(2), car.sc2_top_2(2)], 'Color', lineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);

    line([car.ssc1_bottom(1), car.sc1_bottom_2(1)], [car.ssc1_bottom(2), car.sc1_bottom_2(2)], 'Color', lineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);
    line([car.ssc1_top(1), car.sc1_top_2(1)], [car.ssc1_top(2), car.sc1_top_2(2)], 'Color', lineColor, 'LineWidth', lineWidth, 'LineStyle', lineStyle);
end

function drawland()
    % 创建x坐标从0到3000
    x = -100:3000;  
    % 对应的y坐标为0
    y = zeros(size(x));
    
    % 绘制图形
    plot(x, y, 'LineWidth', 2);  % 可以设置线条宽度
end

function drawobstacle(obstacle)
    % Ensure the coordinates are calculated
    if isempty(obstacle.left_top_coordinate) || isempty(obstacle.left_bottom_coordinate) ...
            || isempty(obstacle.right_top_coordinate) || isempty(obstacle.right_bottom_coordinate)
        obstacle.calculate_top_bottom();
    end
    
    % Extract the coordinates
    x_coords = [obstacle.left_top_coordinate(1), ...
                obstacle.right_top_coordinate(1), ...
                obstacle.right_bottom_coordinate(1), ...
                obstacle.left_bottom_coordinate(1)];
    y_coords = [obstacle.left_top_coordinate(2), ...
                obstacle.right_top_coordinate(2), ...
                obstacle.right_bottom_coordinate(2), ...
                obstacle.left_bottom_coordinate(2)];
    
    % Draw the obstacle
    fill(x_coords, y_coords, 'r', 'FaceAlpha', 0.5); % Red color with transparency
    hold on; % Retain the plot if drawing multiple obstacles
    
    % Add labels or additional visuals if necessary
    plot(obstacle.coordinate(1), obstacle.coordinate(2), 'bo', 'MarkerSize', 8); % Mark the center
    hold off;
end


obstacle = Obstacle([2000,0],120,500);
% 创建 newCarSimulator 类的实例
large_radius = 50;
small_radius = 30;
small_small_radius = 15;
arm_length = 120;
small_arm_length = 180;
length_car = 200;
com_coordinates = [0,50];

theta = -10;
phi = 190;
eta = -5;
zeta = 185;
alpha = 0;
beta = 0;

% 实例化对象
carSimulator = newCarSimulator(large_radius, small_radius, small_small_radius, ...
    arm_length, small_arm_length, length_car, com_coordinates,theta,phi,eta,zeta,alpha,beta);
carSimulator.update();


% carcontroler = CarControler(carSimulator,obstacle);
carcontroler_real = CarControlerReal(carSimulator,obstacle,[-100,3000]);

data_log = struct('com_coordinates', [], 'lc1_center', [], 'lc2_center', [], ...
    'sc1_center', [], 'sc2_center', [], 'ssc1_center', [], 'ssc2_center', [], ...
    'theta', [], 'phi', [], 'eta', [], 'zeta', [], 'alpha', [], 'step', []);


figure;
hold on;
axis equal;
grid on;
nFrames = 3000;  % 动画帧数
for i = 1:nFrames
    % 清除旧的图形
    clf;
    hold on;
    axis equal;
    grid on;

    % 计算小车的移动
    % carcontroler.update(i,nFrames - i)
    % carSimulator.calculate_move(1);
    done = carcontroler_real.update();
    carSimulator.update();


    data_log.com_coordinates = [data_log.com_coordinates; carSimulator.com_coordinates];
    data_log.lc1_center = [data_log.lc1_center; carSimulator.lc1_center];
    data_log.lc2_center = [data_log.lc2_center; carSimulator.lc2_center];
    data_log.sc1_center = [data_log.sc1_center; carSimulator.sc1_center];
    data_log.sc2_center = [data_log.sc2_center; carSimulator.sc2_center];
    data_log.ssc1_center = [data_log.ssc1_center; carSimulator.ssc1_center];
    data_log.ssc2_center = [data_log.ssc2_center; carSimulator.ssc2_center];
    data_log.theta = [data_log.theta; carSimulator.theta];
    data_log.phi = [data_log.phi; carSimulator.phi];
    data_log.eta = [data_log.eta; carSimulator.eta];
    data_log.zeta = [data_log.zeta; carSimulator.zeta];
    data_log.alpha = [data_log.alpha; carSimulator.alpha];
    data_log.step = [data_log.step; i];
    
    % 绘制小车
    drawland();
    drawCar(carSimulator);
    drawobstacle(obstacle);
    % 刷新图形并暂停，控制动画的更新频率
    drawnow;
    pause(0.05);  % 每帧暂停 0.05 秒，调整此值以控制速度
    if done == true
        break;
    end
end

% 绘制 com_coordinates(1) 与 theta 和 phi 的变化图
figure;
plot3(data_log.com_coordinates(:, 1),data_log.phi, data_log.theta, '-r', 'LineWidth', 2);
ylabel('phi');
zlabel('Theta');
xlabel('X');
title('X vs. Theta and Phi');


% 绘制 com_coordinates(2) 与 theta 和 phi 的变化图
figure;
plot3(data_log.com_coordinates(:, 2),data_log.phi, data_log.theta, '-r', 'LineWidth', 2);
ylabel('phi');
zlabel('Theta');
xlabel('Y');
title('Y vs. Theta and Phi');