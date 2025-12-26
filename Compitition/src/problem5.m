%% 2025 CUMCM Problem A - Question 5 Solver
% 5 Drones vs 3 Missiles, Max 3 Bombs each, Constant Altitude Flight
% Strategy: Double-Layer Optimization (GA + Heuristic Scan)
clear; clc; format shortG;

%% 1. 全局参数定义
global Mis Drones RealTarget g
% 真实目标 (保护对象)
RealTarget = [0, 200, 0];
% 重力加速度
g = 9.8;

% --- 导弹参数 M1, M2, M3 ---
% 初始位置
M_pos = [20000, 0, 2000; 
         19000, 600, 2100; 
         18000, -600, 1900];
% 假目标位置 (导弹瞄准点)
FakeTarget = [0, 0, 0];
% 速度标量
Vm = 300;

% 构建导弹结构体
Mis = struct();
for k = 1:3
    Mis(k).P0 = M_pos(k, :);
    dir_vec = FakeTarget - Mis(k).P0;
    dist = norm(dir_vec);
    Mis(k).dir = dir_vec / dist; % 单位方向向量
    Mis(k).flight_time = dist / Vm; % 飞行总时间
    Mis(k).V = Mis(k).dir * Vm;     % 速度向量
end

% --- 无人机初始参数 FY1 - FY5 ---
D_pos = [17800, 0, 1800;
         12000, 1400, 1400;
         6000, -3000, 700;
         11000, 2000, 1800;
         13000, -2000, 1300];
Drones = struct();
for i = 1:5
    Drones(i).P0 = D_pos(i, :);
    Drones(i).h = D_pos(i, 3); % 保持固定高度
end

%% 2. 遗传算法配置 (外层优化)
% 决策变量：10个 [v1, ang1, v2, ang2, v3, ang3, v4, ang4, v5, ang5]
% v范围: [70, 140], ang范围: [0, 2*pi]
nVars = 10;
lb = repmat([70, 0], 1, 5);
ub = repmat([140, 2*pi], 1, 5);

% GA 选项 (为演示速度，种群和代数设得较小，比赛时建议加大)
options = optimoptions('ga', ...
    'PopulationSize', 150, ...     % 种群大小 (建议 100+)
    'MaxGenerations', 80, ...     % 最大迭代次数 (建议 100+)
    'Display', 'iter', ...        % 显示迭代过程
    'UseParallel', false);        % 如有并行工具箱可设为 true

fprintf('开始遗传算法优化...\n');
tic;
[best_vars, best_score] = ga(@fitness_func, nVars, [], [], [], [], lb, ub, [], options);
solve_time = toc;

%% 3. 结果解析与输出
fprintf('\n================ 优化结果 ================\n');
fprintf('求解耗时: %.2f 秒\n', solve_time);
fprintf('最大总有效遮蔽时长: %.2f 秒\n', -best_score); % 负负得正

% 解析最优解的详细投放策略
[~, final_plan] = fitness_func(best_vars);

fprintf('\n>>> 无人机飞行策略 <<<\n');
for i = 1:5
    v = best_vars(2*i-1);
    ang = best_vars(2*i);
    fprintf('FY%d: 速度 %.2f m/s, 航向 %.2f 度 (弧度 %.2f)\n', ...
        i, v, rad2deg(ang), ang);
end

fprintf('\n>>> 烟幕弹投放详细方案 (Result3) <<<\n');
fprintf('%-6s | %-6s | %-8s | %-8s | %-8s | %-15s\n', ...
    'Drone', 'BombID', 'DropTime', 'ExpTime', 'Target', 'CoverDuration');
fprintf('--------------------------------------------------------------------\n');

total_M1 = 0; total_M2 = 0; total_M3 = 0;

for i = 1:5
    drone_plan = final_plan{i};
    if isempty(drone_plan)
        continue;
    end
    for j = 1:size(drone_plan, 1)
        % plan row: [t_drop, t_exp, missile_idx, cover_start, cover_end, score]
        d_time = drone_plan(j, 1);
        e_time = drone_plan(j, 2);
        m_idx = drone_plan(j, 3);
        c_dur = drone_plan(j, 5) - drone_plan(j, 4);
        
        fprintf('FY%-4d | #%-5d | %-8.2f | %-8.2f | M%-7d | %.2f s [%.1f-%.1f]\n', ...
            i, j, d_time, e_time, m_idx, c_dur, drone_plan(j,4), drone_plan(j,5));
    end
end
fprintf('==========================================\n');


%% ---------------------------------------------------------
%  Local Functions (无需单独文件，直接包含在脚本中)
%  ---------------------------------------------------------

function [score, all_plans] = fitness_func(vars)
    % 适应度函数：输入 10 个变量，输出总遮蔽时间的负值（求最小化）
    % all_plans 用于最后输出详细信息
    
    global Mis RealTarget
    
    % 存储每枚导弹被遮蔽的时间段集合
    % 结构: cell array {M1_intervals; M2_intervals; M3_intervals}
    missile_covers = cell(3, 1);
    all_plans = cell(5, 1);
    
    % 遍历 5 架无人机
    for i = 1:5
        % 提取当前无人机的决策变量
        v = vars(2*i-1);
        ang = vars(2*i);
        
        % 计算该无人机的最佳投弹策略 (内层贪心/遍历)
        [drone_intervals, drone_plan] = calculate_drone_strategy(i, v, ang);
        
        % 收集结果
        all_plans{i} = drone_plan;
        for k = 1:3
            if ~isempty(drone_intervals{k})
                missile_covers{k} = [missile_covers{k}; drone_intervals{k}];
            end
        end
    end
    
    % 计算总有效遮蔽时间 (处理并集)
    total_time = 0;
    for k = 1:3
        intervals = missile_covers{k};
        if isempty(intervals)
            continue;
        end
        % 计算区间并集总长度
        union_len = calc_union_length(intervals);
        total_time = total_time + union_len;
    end
    
    % GA 是求最小值，所以取负
    score = -total_time; 
end

function [intervals_by_missile, selected_plan] = calculate_drone_strategy(drone_idx, v, ang)
    % 内层核心函数：给定无人机轨迹，找出最优的3次投弹
    % 输入：无人机ID，速度，航向
    % 输出：针对M1-M3的遮蔽区间，以及具体的投弹计划
    
    global Drones Mis g
    
    P0 = Drones(drone_idx).P0;
    intervals_by_missile = cell(3, 1);
    
    % 1. 生成候选投弹列表
    % 离散化遍历投弹时间 t_drop
    % 无人机最长飞行时间估计：导弹最多飞 70秒，取 0-70s
    t_drops = 0 : 1.0 : 70; 
    candidates = []; % 格式: [t_drop, t_exp, missile_idx, start_t, end_t, duration]
    
    % 无人机速度向量
    V_drone = [v * cos(ang), v * sin(ang), 0];
    
    for t_d = t_drops
        % 此刻无人机位置
        P_drop = P0 + V_drone * t_d;
        
        % 遍历每枚导弹，看能否拦截
        for m_id = 1:3
            % 几何逆推：为了拦截导弹，烟幕弹需要在空中爆炸并形成云团
            % 烟幕弹水平位置随时间变化: P(t)xy = P_drop_xy + V_drone_xy * (t - t_d)
            % 烟幕弹垂直位置: Z(t) = P_drop_z - 0.5*g*(t - t_d)^2
            
            % 简化逻辑：扫描可能的起爆延迟 delta_t
            % 延迟范围：1s 到 15s (根据高度差估算，1800m自由落体约19s)
            for delta_t = 1 : 2 : 15
                t_e = t_d + delta_t;
                
                % 爆炸点位置
                P_boom = P_drop + V_drone * delta_t; 
                P_boom(3) = P_drop(3) - 0.5 * g * delta_t^2;
                
                % 如果爆炸点已经在地下，跳过
                if P_boom(3) < 0
                    continue;
                end
                
                % 检查该爆炸点产生的云团能否遮蔽导弹 m_id
                [is_block, t_start, t_end] = check_interception(P_boom, t_e, m_id);
                
                if is_block
                    dur = t_end - t_start;
                    if dur > 0.1
                        % 记录候选策略
                        candidates = [candidates; t_d, t_e, m_id, t_start, t_end, dur];
                    end
                end
            end
        end
    end
    
    % 2. 贪心选择最优的 3 个不冲突的投弹时机
    selected_plan = []; % [t_drop, t_exp, m_idx, t_start, t_end, dur]
    
    if isempty(candidates)
        return;
    end
    
    % 按遮蔽时长降序排列
    [~, sort_idx] = sort(candidates(:, 6), 'descend');
    sorted_cands = candidates(sort_idx, :);
    
    count = 0;
    for i = 1:size(sorted_cands, 1)
        if count >= 3
            break;
        end
        
        cand = sorted_cands(i, :);
        t_d_curr = cand(1);
        
        % 检查与已选方案的时间间隔约束 (|t_d1 - t_d2| >= 1)
        conflict = false;
        for j = 1:size(selected_plan, 1)
            if abs(t_d_curr - selected_plan(j, 1)) < 1.0 - 1e-5
                conflict = true;
                break;
            end
        end
        
        if ~conflict
            selected_plan = [selected_plan; cand];
            % 将结果加入输出格式
            m_idx = cand(3);
            intervals_by_missile{m_idx} = [intervals_by_missile{m_idx}; cand(4), cand(5)];
            count = count + 1;
        end
    end
end

function [blocked, t_start, t_end] = check_interception(P_boom, t_exp, m_idx)
    % 检查单个云团对特定导弹的遮蔽情况
    global Mis RealTarget
    
    blocked = false; t_start = 0; t_end = 0;
    
    M_struct = Mis(m_idx);
    
    % 烟幕有效时间窗口
    cloud_life_start = t_exp;
    cloud_life_end = t_exp + 20;
    
    % 导弹飞行结束时间
    mis_end = M_struct.flight_time;
    
    % 实际有效检测时间段 (取交集)
    check_start = max(0, cloud_life_start);
    check_end = min(mis_end, cloud_life_end);
    
    if check_start >= check_end
        return;
    end
    
    % 离散化检测遮蔽 (步长 0.2s)
    ts = check_start : 0.2 : check_end;
    is_cov = false(size(ts));
    
    % 云团下沉速度
    V_sink = [0, 0, -3];
    
    for k = 1:length(ts)
        t = ts(k);
        % 此刻云团中心
        P_c = P_boom + V_sink * (t - t_exp);
        % 此刻导弹位置
        P_m = M_struct.P0 + M_struct.V * t;
        
        % 遮蔽判断核心：
        % 1. 计算 P_c 到线段 P_m -> RealTarget 的距离
        dist = point_to_segment_dist(P_c, P_m, RealTarget);
        
        % 2. 判断是否在半径 10m 内
        if dist <= 10
            % 3. 补充判断：云团是否在导弹前方 (简单判断距离关系)
            d_m_target = norm(P_m - RealTarget);
            d_c_target = norm(P_c - RealTarget);
            if d_c_target < d_m_target % 云团在导弹和目标之间
                is_cov(k) = true;
            end
        end
    end
    
    if any(is_cov)
        blocked = true;
        valid_indices = find(is_cov);
        t_start = ts(valid_indices(1));
        t_end = ts(valid_indices(end));
    end
end

function d = point_to_segment_dist(P, A, B)
    % 计算点 P 到线段 AB 的最短距离
    AB = B - A;
    AP = P - A;
    
    % 投影系数
    t = dot(AP, AB) / dot(AB, AB);
    
    if t < 0
        closest = A; % 投影在 A 外侧
    elseif t > 1
        closest = B; % 投影在 B 外侧
    else
        closest = A + t * AB; % 投影在线段上
    end
    
    d = norm(P - closest);
end

function len = calc_union_length(intervals)
    % 计算多个时间区间的并集总长度
    % intervals: Nx2 matrix [start, end]
    if isempty(intervals)
        len = 0; return;
    end
    
    % 按开始时间排序
    intervals = sortrows(intervals, 1);
    
    merged = intervals(1, :);
    idx = 1;
    
    for i = 2:size(intervals, 1)
        curr = intervals(i, :);
        if curr(1) < merged(idx, 2) % 有重叠
            merged(idx, 2) = max(merged(idx, 2), curr(2));
        else
            idx = idx + 1;
            merged(idx, :) = curr;
        end
    end
    
    len = sum(merged(:, 2) - merged(:, 1));
end