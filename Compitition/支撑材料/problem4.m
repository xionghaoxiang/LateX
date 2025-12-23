function Problem4_MultiUAV_Final_Strategy()
    clc; close all;
    %% 1. 全局环境参数
    Env.g = 9.8; 
    Env.Pos_True = [0, 200, 0]; 
    Env.Pos_M1   = [20000, 0, 2000];       
    Env.V_M1     = 300;                           
    Env.Vec_M    = [0, 0, 0] - Env.Pos_M1; 
    Env.Dist_M1  = norm(Env.Vec_M);
    Env.Dir_M1   = Env.Vec_M / Env.Dist_M1;         
    Env.V_sink   = 3;      
    Env.R_smk    = 10;           
    Env.T_last   = 20; 
    
    Env.Pos_UAVs = [
        17800, 0,    1800;  % FY1
        12000, 1400, 1400;  % FY2
        6000, -3000, 700    % FY3
    ];
    
    %% 2. 优化参数设置
    % 估算基准航向
    Base_Ang1 = rad2deg(atan2(200 - 0,    0 - 17800)); 
    Base_Ang2 = rad2deg(atan2(200 - 1400, 0 - 12000)); 
    Base_Ang3 = rad2deg(atan2(200 + 3000, 0 - 6000));  
    
    % 变量顺序: [V, Ang, T_drop, T_delay] * 3架
    % 速度约束 70-140
    LB = [70, Base_Ang1-45, 0, 1,   70, Base_Ang2-45, 0, 1,   70, Base_Ang3-45, 0, 1];
    UB = [140, Base_Ang1+45, 15, 8, 140, Base_Ang2+45, 35, 8, 140, Base_Ang3+45, 55, 8];
    
    de_opts.NP = 200;       
    de_opts.MaxIter = 800;  
    de_opts.F = 0.5;        
    de_opts.CR = 0.9;       
    
    fprintf('======================================================\n');
    fprintf('      问题4：三机协同立体封锁\n');
    fprintf('======================================================\n');
    
    try, if isempty(gcp('nocreate')), parpool; end; end
    CostFunc = @(x) CostFunc_3UAV(x, Env);
    
    tic;
    % 运行优化 (无初解)
    [best_x, best_val] = Run_DE_3UAV(CostFunc, LB, UB, de_opts);
    total_time = toc;
    
    %% 3. 结果解析与策略输出
    [~, real_shield_time] = CostFunc_3UAV(best_x, Env);
    Res = Parse_Params(best_x);
    
    fprintf('\n>>> 优化完成！耗时: %.2f 秒\n', total_time);
    fprintf('>>> 🏆 最终最大遮蔽时长: %.4f 秒\n', real_shield_time);
    
    fprintf('\n================== [最终投弹策略表] ==================\n');
    fprintf('| 机号 |  飞行速度 |  飞行航向 | 投放时刻(s) | 延时时长(s) | 起爆时刻(s) |\n');
    fprintf('|------|-----------|-----------|-------------|-------------|-------------|\n');
    fprintf('| FY1  | %9.2f | %9.2f | %11.4f | %11.4f | %11.4f |\n', ...
        Res(1).v, Res(1).a, Res(1).td, Res(1).ty, Res(1).tp);
    fprintf('| FY2  | %9.2f | %9.2f | %11.4f | %11.4f | %11.4f |\n', ...
        Res(2).v, Res(2).a, Res(2).td, Res(2).ty, Res(2).tp);
    fprintf('| FY3  | %9.2f | %9.2f | %11.4f | %11.4f | %11.4f |\n', ...
        Res(3).v, Res(3).a, Res(3).td, Res(3).ty, Res(3).tp);
    fprintf('======================================================\n');
    
    %% 4. 绘图
    Plot_Effective_Gantt(Res, real_shield_time, Env);
    Save_To_Excel(Res, 'result2.xlsx');
end


%  辅助函数：精确计算有效时间区间
function Intervals = Calculate_Exact_Intervals(Res, Env)
    % 初始化
    Intervals = repmat(struct('is_effective', false, 'start_time', NaN, 'end_time', NaN), 3, 1);
    
    dt = 0.01; 
    T_max = Env.Dist_M1 / Env.V_M1 + 5; 
    t_vec = 0 : dt : T_max;
    
    % 预计算每架飞机的烟雾位置参数
    SmokeData = [];
    for i=1:3
        ang = deg2rad(Res(i).a);
        V = [cos(ang), sin(ang), 0] * Res(i).v;
        P_Drop = Env.Pos_UAVs(i,:) + V * Res(i).td;
        P_Pop_Init = P_Drop + V * Res(i).ty;
        P_Pop_Init(3) = P_Pop_Init(3) - 0.5 * 9.8 * Res(i).ty^2;
        SmokeData(i).P_Pop_Init = P_Pop_Init;
        SmokeData(i).tp = Res(i).tp;
    end
    
    % 逐帧检测遮挡情况
    for i = 1:3
        mask = false(size(t_vec));
        for k = 1:length(t_vec)
            t = t_vec(k);
            % 1. 导弹当前位置
            if t * Env.V_M1 > Env.Dist_M1
                P_M = Env.Pos_True; % 已到达
            else
                P_M = Env.Pos_M1 + Env.Dir_M1 * (Env.V_M1 * t);
            end
            
            % 2. 判断该飞机的烟雾是否存在
            if t >= SmokeData(i).tp && t <= SmokeData(i).tp + Env.T_last
                t_rel = t - SmokeData(i).tp;
                P_Smk = SmokeData(i).P_Pop_Init - [0, 0, Env.V_sink * t_rel];
                
                % 3. 判断是否遮挡
                v_los = Env.Pos_True - P_M;
                w_vec = P_Smk - P_M;
                c1 = dot(w_vec, v_los);
                c2 = dot(v_los, v_los);
                if c1 > 0
                    b = c1 / c2;
                    Pb = P_M + v_los * b;
                    dist_sq = sum((P_Smk - Pb).^2);
                    if dist_sq <= Env.R_smk^2
                        mask(k) = true;
                    end
                end
            end
        end
        
        % 提取起止时间
        if any(mask)
            idx = find(mask);
            Intervals(i).is_effective = true;
            Intervals(i).start_time = t_vec(idx(1));
            Intervals(i).end_time   = t_vec(idx(end));
        end
    end
end


function Res = Parse_Params(x)
    for i = 1:3
        idx = (i-1)*4;
        Res(i).v  = x(idx+1); Res(i).a  = x(idx+2);
        Res(i).td = x(idx+3); Res(i).ty = x(idx+4);
        Res(i).tp = Res(i).td + Res(i).ty;
    end
end

function [score, pure_time] = CostFunc_3UAV(x, Env)
    Res = Parse_Params(x);
    for i=1:3
        if Res(i).v < 70 || Res(i).v > 140, score = 1e6; pure_time = 0; return; end
    end
    P_Pop = zeros(3, 3); Times_Pop = zeros(3, 1);
    for i = 1:3
        ang_rad = deg2rad(Res(i).a);
        Vel_Vec = [cos(ang_rad), sin(ang_rad), 0] * Res(i).v;
        P_Drop = Env.Pos_UAVs(i,:) + Vel_Vec * Res(i).td;
        Disp = Vel_Vec * Res(i).ty; Disp(3) = Disp(3) - 0.5 * 9.8 * Res(i).ty^2;
        P_Pop(i,:) = P_Drop + Disp; Times_Pop(i) = Res(i).tp;
        if P_Pop(i,3) < 0, score = 1e5; pure_time = 0; return; end
    end
    t_start = min(Times_Pop); t_end = min(Env.Dist_M1/Env.V_M1, max(Times_Pop) + Env.T_last);
    if t_start >= t_end, score = 1e5; pure_time = 0; return; end
    Target_Points = [0, 200, 5; 0, 200, 10; 0, 200, 0; -7, 200, 5; 7, 200, 5];
    num_pts = 5; dt = 0.05; t_vec = t_start : dt : t_end;
    total_coverage = 0; penalty_dist = 0; min_dists = [1e9, 1e9, 1e9]; 
    for t = t_vec
        P_M = Env.Pos_M1 + Env.Dir_M1 * (Env.V_M1 * t); blocked_pts_count = 0;
        for p = 1:num_pts
            TP = Target_Points(p,:); v_los = TP - P_M; len_los_sq = sum(v_los.^2); is_pt_blocked = false;
            for k = 1:3
                if t >= Times_Pop(k) && t <= Times_Pop(k) + Env.T_last
                    P_Smk = P_Pop(k,:) - [0, 0, Env.V_sink * (t - Times_Pop(k))];
                    w_vec = P_Smk - P_M; c1 = dot(w_vec, v_los);
                    if c1 > 0
                        b = max(0, min(1, c1 / len_los_sq)); Pb = P_M + v_los * b;
                        d_sq = sum((P_Smk - Pb).^2); dist = sqrt(d_sq);
                        if p == 1 && dist < min_dists(k), min_dists(k) = dist; end
                        if d_sq <= Env.R_smk^2, is_pt_blocked = true; end
                    end
                end
            end
            if is_pt_blocked, blocked_pts_count = blocked_pts_count + 1; end
        end
        total_coverage = total_coverage + (blocked_pts_count / num_pts) * dt;
    end
    pure_time = total_coverage;
    for k=1:3, penalty_dist = penalty_dist + max(0, min_dists(k) - Env.R_smk); end
    score = -total_coverage + 0.1 * penalty_dist;
end

function [best_mem, best_val] = Run_DE_3UAV(cost_func, lb, ub, opts)
    NP = opts.NP; D = length(lb);
    pop = repmat(lb, NP, 1) + rand(NP, D) .* repmat(ub-lb, NP, 1);
    
    val = zeros(NP, 1);
    parfor i=1:NP, val(i) = cost_func(pop(i,:)); end
    [best_val, idx] = min(val);
    best_mem = pop(idx, :);
    
    h = waitbar(0, '正在寻找最优策略...');
    for gen = 1 : opts.MaxIter
        F = opts.F * (1 - 0.2 * gen/opts.MaxIter); 
        pop_new = pop; val_new = val;
        parfor i = 1 : NP
            r = randperm(NP, 3);
            mutant = best_mem + F * (pop(r(1),:) - pop(r(2),:));
            trial = pop(i, :);
            j_rand = randi(D);
            for j = 1 : D
                if rand < opts.CR || j == j_rand, trial(j) = mutant(j); end
            end
            trial = max(trial, lb); trial = min(trial, ub);
            t_v = feval(cost_func, trial);
            if t_v < val(i), pop_new(i,:) = trial; val_new(i) = t_v; end
        end
        pop = pop_new; val = val_new;
        [c_best, idx] = min(val);
        if c_best < best_val, best_val = c_best; best_mem = pop(idx, :); end
        
        if mod(gen, 50) == 0
            [~, t_real] = feval(cost_func, best_mem);
            waitbar(gen/opts.MaxIter, h, sprintf('Iter %d: %.2fs', gen, t_real));
        end
    end
    close(h);
end

%  绘图函数
function Plot_Effective_Gantt(Res, score, Env)
    figure('Color', 'w', 'Position', [100, 100, 700, 500], 'Name', 'Final Strategy & Schedule');
    hold on;
    
    colors = {
        [100, 100, 255]/255,   % FY1 蓝
        [238, 130, 238]/255,   % FY2 紫
        [100, 200, 100]/255    % FY3 绿
    };
    
    % 计算精确区间
    Intervals = Calculate_Exact_Intervals(Res, Env);
    bar_height = 0.5;
    
    for i = 1:3
        % 只画有效遮蔽区间
        if Intervals(i).is_effective
            t_s = Intervals(i).start_time;
            t_e = Intervals(i).end_time;
            
            x_patch = [t_s, t_e, t_e, t_s];
            y_patch = [i - bar_height/2, i - bar_height/2, i + bar_height/2, i + bar_height/2];
            patch(x_patch, y_patch, colors{i}, 'EdgeColor', 'k', 'LineWidth', 1.2, 'FaceAlpha', 0.9);
           
            text(t_s, i - bar_height/2 - 0.2, sprintf('Start: %.1fs', t_s), ...
                'FontSize', 10, 'Color', 'k', 'HorizontalAlignment', 'left');
                
            text((t_s+t_e)/2, i, sprintf('Dur: %.2fs', t_e - t_s), ...
                 'FontSize', 9, 'Color', 'w', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
        else
            t_tp = Res(i).tp; 
            rectangle('Position', [t_tp, i-0.1, 1, 0.2], 'EdgeColor', [0.8 0.8 0.8], 'LineStyle', '--');
            text(t_tp, i-0.3, 'Ineffective', 'FontSize', 8, 'Color', 'r');
        end
    end
    
    ylim([0, 4]);
    yticks([1, 2, 3]);
    yticklabels({'FY1', 'FY2', 'FY3'});
    
    valid_times = [Intervals.start_time, Intervals.end_time];
    valid_times = valid_times(~isnan(valid_times));
    if isempty(valid_times), xlim([0, 30]); else, xlim([min(valid_times)-2, max(valid_times)+5]); end
    
    xlabel('时间 (s)', 'FontSize', 12);
    title(sprintf('三机协同有效遮蔽时序 (Total: %.4fs)', score), 'FontSize', 14);
    grid on; ax=gca; ax.GridAlpha=0.3; ax.XMinorGrid='on'; box on;
    hold off;
end

function Save_To_Excel(Res, filename)
    data = zeros(3, 5);
    for i=1:3
        data(i,1) = Res(i).v;
        data(i,2) = Res(i).a;
        data(i,3) = Res(i).td;
        data(i,4) = Res(i).ty;
        data(i,5) = Res(i).tp;
    end
    try, writematrix(data, filename); catch, end
end