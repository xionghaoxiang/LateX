function smoke_strategy_optimization()
    clear; clc;
    % 1. 环境与物理参数定义
    params.Vm = 300;                    % 导弹速度
    params.Pm0 = [20000, 0, 2000];      % 导弹初位置
    params.Target_missile = [0, 0, 0];  % 导弹目标点(假目标)
    params.P_true = [0, 200, 5];        % 真目标遮蔽判定点
    params.P_uav0 = [17800, 0, 1800];   % 无人机初位置
    params.g = 9.8;                     % 重力加速度
    params.Vsink = 3;                   % 烟幕下沉速度
    params.R = 10;                      % 有效半径
    params.Duration = 20;               % 烟幕持续时间
    
    % 计算导弹运动单位向量
    params.dir_m = (params.Target_missile - params.Pm0) / norm(params.Target_missile - params.Pm0);

    % 2. PSO参数设置
    nVar = 8;                           % 变量: [v, theta, t1, dt12, dt23, tau1, tau2, tau3]
    lb = [70,  0,    0,  1.0, 1.0, 0, 0, 0];   % 下界
    ub = [140, 2*pi, 60, 10,  10,  15, 15, 15]; % 上界 (t1和dt上限根据战场时空估计)
    
    nPop = 50;                          % 种群规模
    maxIter = 300;                      % 最大迭代次数
    w = 0.8;                            % 惯性权重
    c1 = 1.5;                           % 个体学习因子
    c2 = 1.5;                           % 社会学习因子
    stallLimit = 15;                    % 触发灾变的停滞步数

    % 初始化种群
    particles = repmat(struct('pos',[],'vel',[],'cost',0,'bestPos',[],'bestCost',-inf), nPop, 1);
    globalBest.cost = -inf;
    stallCounter = 0;

    for i = 1:nPop
        particles(i).pos = lb + (ub - lb) .* rand(1, nVar);
        particles(i).vel = zeros(1, nVar);
        particles(i).cost = fitness_func(particles(i).pos, params);
        particles(i).bestPos = particles(i).pos;
        particles(i).bestCost = particles(i).cost;
        if particles(i).cost > globalBest.cost
            globalBest = particles(i);
        end
    end

    % 3. 迭代优化
    for it = 1:maxIter
        prevBestCost = globalBest.cost;
        
        for i = 1:nPop
            % 更新速度与位置
            particles(i).vel = w*particles(i).vel + c1*rand(1,nVar).*(particles(i).bestPos - particles(i).pos) ...
                             + c2*rand(1,nVar).*(globalBest.pos - particles(i).pos);
            particles(i).pos = particles(i).pos + particles(i).vel;
            % 边界检查
            particles(i).pos = max(min(particles(i).pos, ub), lb);
            
            % 计算适应度
            particles(i).cost = fitness_func(particles(i).pos, params);
            
            if particles(i).cost > particles(i).bestCost
                particles(i).bestCost = particles(i).cost;
                particles(i).bestPos = particles(i).pos;
            end
            if particles(i).cost > globalBest.cost
                globalBest = particles(i);
            end
        end
        
        % --- 灾变机制 ---
        if abs(globalBest.cost - prevBestCost) < 1e-4
            stallCounter = stallCounter + 1;
        else
            stallCounter = 0;
        end
        
        if stallCounter >= stallLimit
            % 对除了全局最优外的50%粒子进行变异/重置
            for j = 1:floor(nPop/2)
                idx = randi(nPop);
                if idx ~= 1 % 假设1号不一定是最好，这里简单处理
                    particles(idx).pos = lb + (ub - lb) .* rand(1, nVar);
                    particles(idx).vel = (rand(1,nVar)-0.5).*(ub-lb)*0.2;
                end
            end
            stallCounter = 0;
            fprintf('Iter %d: Catastrophe triggered!\n', it);
        end
        
        fprintf('Iter %d: Max Duration = %.4f s\n', it, globalBest.cost);
    end

    % 4. 输出结果
    display_results(globalBest.pos, params);
end

%% 适应度函数：计算三枚弹的总有效遮蔽时长（并集）
function totalTime = fitness_func(x, params)
    v = x(1); theta = x(2);
    t_drops = [x(3), x(3)+x(4), x(3)+x(4)+x(5)]; % 投放时间
    taus = [x(6), x(7), x(8)];                   % 延时
    
    dt = 0.05; % 时间步长
    t_sim = 0:dt:100;
    is_shielded = false(size(t_sim));
    
    % 无人机速度向量
    Vu = [v*cos(theta), v*sin(theta), 0];
    
    % 计算每枚弹的爆炸点和状态
    for i = 1:3
        P_drop = params.P_uav0 + Vu * t_drops(i);
        % 起爆点坐标 (平抛运动)
        P_exp = P_drop + [Vu(1)*taus(i), Vu(2)*taus(i), -0.5*params.g*taus(i)^2];
        t_exp = t_drops(i) + taus(i);
        
        % 检查每一秒是否遮蔽
        for k = 1:length(t_sim)
            tk = t_sim(k);
            if tk >= t_exp && tk <= t_exp + params.Duration
                % 烟幕中心在 tk 时刻的位置
                P_cloud = P_exp - [0, 0, params.Vsink * (tk - t_exp)];
                % 导弹在 tk 时刻的位置
                Pm = params.Pm0 + params.dir_m * params.Vm * tk;
                % 计算点 P_cloud 到线段 (Pm -- P_true) 的距离
                dist = point_to_line_dist(P_cloud, Pm, params.P_true);
                if dist <= params.R
                    is_shielded(k) = true;
                end
            end
        end
    end
    totalTime = sum(is_shielded) * dt;
end

%% 工具函数：点到线段的距离
function d = point_to_line_dist(P, A, B)
    v = B - A;
    w = P - A;
    c1 = dot(w, v);
    if c1 <= 0, d = norm(P - A); return; end
    c2 = dot(v, v);
    if c2 <= c1, d = norm(P - B); return; end
    b = c1 / c2;
    Pb = A + b * v;
    d = norm(P - Pb);
end

function display_results(x, params)
    v = x(1); theta = x(2);
    t_drops = [x(3), x(3)+x(4), x(3)+x(4)+x(5)];
    taus = [x(6), x(7), x(8)];
    Vu = [v*cos(theta), v*sin(theta), 0];

    fprintf('\n--- 优化结果 ---\n');
    fprintf('无人机航向 (rad): %.4f (deg: %.2f)\n', theta, rad2deg(theta));
    fprintf('无人机速度 (m/s): %.2f\n', v);

    for i = 1:3
    P_drop = params.P_uav0 + Vu * t_drops(i);
    P_exp = P_drop + [Vu(1)*taus(i), Vu(2)*taus(i), -0.5*params.g*taus(i)^2];
    fprintf('第%d枚烟幕弹:\n', i);
    fprintf(' 投放点: (%.2f, %.2f, %.2f)\n', P_drop(1), P_drop(2), P_drop(3))
    fprintf(' 爆炸点: (%.2f, %.2f, %.2f)\n', P_exp(1), P_exp(2), P_exp(3));
    end
    fprintf('最终有效干扰总时长: %.4f s\n', fitness_func(x, params));
end