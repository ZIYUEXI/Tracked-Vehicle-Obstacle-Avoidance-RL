

% 初始化
obstacle_height = 200;% 障碍物高度
arm_length = 200;% 臂长
small_arm_length = 130;% 小臂长
myEnv = MyEnvironment();
load('Agent85.mat', 'agent');
InitialObs = reset(myEnv, obstacle_height,arm_length,small_arm_length);

maxSteps = 8000;

for step = 1:maxSteps
    % 使用训练好的策略选择动作
    Action = getAction(agent, InitialObs);  % 根据观察选择动作

    % 执行动作并获取新状态
    obs = myEnv.step(Action{1});

    % 更新观察状态
    InitialObs = obs;

    if myEnv.Mydone == true
        break;
    end
end
    
