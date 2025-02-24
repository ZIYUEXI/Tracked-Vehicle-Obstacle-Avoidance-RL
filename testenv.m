% 创建环境
env = MyEnvironment;

% 获取环境信息
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

% 初始化选项（保持原参数）
initOpts = rlAgentInitializationOptions("NumHiddenUnit", 256, "UseRNN", false);

% 配置PPO代理选项
agentOptions = rlPPOAgentOptions(...
    'SampleTime', -1, ...         
    'DiscountFactor', 0.895, ...   % 提高长期奖励的权重
    'ExperienceHorizon', 6000, ...  % 缩短经验收集窗口
    'ClipFactor', 0.2, ...         % 降低剪切范围增加策略更新幅度
    'EntropyLossWeight', 0.01, ... % 降低熵正则化权重
    'MiniBatchSize', 4096, ...       % 减小批处理尺寸
    'NumEpoch', 40, ...           % 增加策略迭代次数
    'AdvantageEstimateMethod', "gae", ... % 使用GAE
    'GAEFactor', 0.95);           % GAE系数

% 删除或注释掉以下无效的Epsilon设置
% agentOptions.EpsilonGreedyExploration.EpsilonDecay = 0.699; 

% 设置优化器学习率
agentOptions.ActorOptimizerOptions.LearnRate = 0.001;
agentOptions.CriticOptimizerOptions.LearnRate = 0.001;

% 创建PPO代理
agent = rlPPOAgent(obsInfo, actInfo, initOpts, agentOptions);

% 配置训练参数（需补充）
trainingOpts = rlTrainingOptions(...
    'MaxEpisodes', 10000, ...          % 最大训练回合数
    'MaxStepsPerEpisode', 2000, ...   % 每回合最大步数
    'ScoreAveragingWindowLength', 20, ... % 平均奖励窗口
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 20000000, ...     % 停止训练阈值
    'SaveAgentCriteria', 'EpisodeSteps', ...  % 新增：按步数保存
    'SaveAgentValue', 5, ...       % 新增：每6000步保存一次
    'SaveAgentDirectory', 'savedAgents'); % 新增：保存路径

% 创建保存目录（确保目录存在）
if ~exist('savedAgents', 'dir')
    mkdir('savedAgents');
end

% 训练代理（需执行训练）
trainingStats = train(agent, env, trainingOpts);
