classdef Obstacle < handle % 定义障碍物类，继承自handle类以实现句柄语义
    properties
        height        % 障碍物的垂直高度（单位与坐标系一致）
        length        % 障碍物的水平长度（单位与坐标系一致）
        coordinate    % 障碍物底边中心点的坐标[x,y]（基准定位点）

        left_top_coordinate     % 障碍物左上角顶点坐标[x,y]
        left_bottom_coordinate  % 障碍物左下角顶点坐标[x,y]
        right_top_coordinate    % 障碍物右上角顶点坐标[x,y]
        right_bottom_coordinate % 障碍物右下角顶点坐标[x,y]
    end
    methods
        % 构造函数：通过中心坐标、高度和长度初始化障碍物对象
        function obj = Obstacle(coordinate, height, length)
            obj.coordinate = coordinate; % 设置基准坐标（底边中心点）
            obj.length = length;         % 设置障碍物水平长度
            obj.height = height;         % 设置障碍物垂直高度
            obj.calculate_top_bottom();  % 计算四个顶点的坐标
        end

        % 计算障碍物四个顶点坐标的辅助方法
        function calculate_top_bottom(obj)
            % 计算左上角坐标：基准点左移半长，上移高度
            obj.left_top_coordinate(1) = obj.coordinate(1) - (obj.length / 2); % X坐标
            obj.left_top_coordinate(2) = obj.coordinate(2) + obj.height;       % Y坐标

            % 计算左下角坐标：基准点左移半长，保持基准高度
            obj.left_bottom_coordinate(1) = obj.coordinate(1) - (obj.length / 2); % X坐标
            obj.left_bottom_coordinate(2) = obj.coordinate(2);                    % Y坐标

            % 计算右上角坐标：基准点右移半长，上移高度
            obj.right_top_coordinate(1) = obj.coordinate(1) + (obj.length / 2); % X坐标
            obj.right_top_coordinate(2) = obj.coordinate(2) + obj.height;       % Y坐标

            % 计算右下角坐标：基准点右移半长，保持基准高度
            obj.right_bottom_coordinate(1) = obj.coordinate(1) + (obj.length / 2); % X坐标
            obj.right_bottom_coordinate(2) = obj.coordinate(2);                    % Y坐标
        end
    end
end