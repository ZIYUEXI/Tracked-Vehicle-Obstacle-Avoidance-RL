% 创建环境
env = MyEnvironment;

% 获取环境信息
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

% 设置优化器学习率% 初始化选项（保持原参数）
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

agentOptions.ActorOptimizerOptions.LearnRate = 0.001;
agentOptions.CriticOptimizerOptions.LearnRate = 0.001;

% 创建PPO代理
agent = rlPPOAgent(obsInfo, actInfo, initOpts, agentOptions);

actorNet =getModel(getActor(agent));
criticNet=getModel(getCritic(agent));
plot(layerGraph(criticNet))
%plot(layerGraph(criticNet))