function Full_Range_PSO_Sweep_GanttOnly()
    clc; close all;
    %% 1. 场景基础参数
    Env.g = 9.8; 
    Env.Pos_FakeTarget = [0, 0, 0];           
    Env.Pos_TrueTarget = [0, 200, 5];         
    Env.Pos_M1_Init = [20000, 0, 2000];       
    Env.V_M1 = 300;                           
    Env.Vec_M = Env.Pos_FakeTarget - Env.Pos_M1_Init;
    Env.Dir_M1 = Env.Vec_M / norm(Env.Vec_M);         
    Env.Dist_Total_M1 = norm(Env.Vec_M);
    Env.Pos_FY1_Init = [17800, 0, 1800];      
    Env.V_smoke_sink = 3;      
    Env.R_smoke = 10;           
    Env.Time_smoke_last = 25;   
    
    % === 仿真精度设置 ===
    Env.dt = 0.001; 
    
    %% 2. 扫描设置
    v_scan_list = 70 : 0.5 : 140; 
    num_scan = length(v_scan_list);
    
    % 结果存储
    results_score = zeros(num_scan, 1);
    results_params = zeros(num_scan, 3); % [航向, 投放, 延时]
    
    fprintf('======================================================\n');
    fprintf('      正在计算最优解...\n');
    fprintf('======================================================\n');
    
    try
        if isempty(gcp('nocreate')), parpool; end
    catch
    end
    
    %% 3. 并行扫描循环
    parfor i = 1 : num_scan
        v_curr = v_scan_list(i);
        
        % --- PSO 配置 ---
        Vec_Base = Env.Pos_TrueTarget - Env.Pos_FY1_Init;
        Base_Angle = rad2deg(atan2(Vec_Base(2), Vec_Base(1)));
        
        LB = [Base_Angle-30, 0, 1.0];
        UB = [Base_Angle+30, 12, 8.0];
        
        [best_x, best_val] = Run_Micro_PSO(v_curr, LB, UB, Env);
        
        results_score(i) = best_val;
        results_params(i, :) = best_x;
    end
    
    %% 4. 提取最优结果
    [max_score, idx_best] = max(results_score);
    best_v = v_scan_list(idx_best);
    best_p = results_params(idx_best, :); 
    
    fprintf('>>> 计算完成\n');
    fprintf('>>> 冠军速度: %.1f m/s\n', best_v);
    fprintf('>>> 极限遮蔽: %.5f 秒\n', max_score);
    
    %% 5. 绘图 
    fprintf('\n>>> 正在生成时序甘特图...\n');
    
    [t_start_opt, t_end_opt, duration_opt] = Recompute_Time_Series(best_v, best_p, Env);
    
    if duration_opt > 0
        figure('Color', 'w', 'Position', [100, 100, 600, 600], 'Name', 'Simulated Gantt Chart');
        hold on;
        
        bar_height = 0.6;
        fy_index = 1; 
        color_fill = [100, 100, 255]/255;
        
        x_patch = [t_start_opt, t_end_opt, t_end_opt, t_start_opt];
        y_patch = [fy_index - bar_height/2, fy_index - bar_height/2, fy_index + bar_height/2, fy_index + bar_height/2];
        patch(x_patch, y_patch, color_fill, 'EdgeColor', 'k', 'LineWidth', 1.5);
        
        text_str = sprintf('Start: %.1fs', t_start_opt);
        text(t_start_opt, fy_index - bar_height/2 - 0.15, text_str, ...
            'FontSize', 11, 'Color', 'k', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');

        ylim([0, 4]);
        yticks([1, 2, 3]);
        yticklabels({'FY1', 'FY2', 'FY3'});
        xlim([-10, 50]); 
        xlabel('时间 (s)', 'FontSize', 12);
 
        title(sprintf('协同遮蔽时序 (总长: %.2fs)', duration_opt), 'FontSize', 14);
        
        grid on;
        ax = gca;
        ax.GridAlpha = 0.3;
        ax.XMinorGrid = 'on';
        box on;
        hold off;
    else
        fprintf('警告: 最优解未能形成有效遮蔽，无法绘制甘特图。\n');
    end
end

%% --- 辅助函数：重新计算时序以用于绘图 ---
function [t_start, t_end, valid_dur] = Recompute_Time_Series(v, params, Env)
    % 解包参数
    a = params(1); td = params(2); ty = params(3);
    dt = 0.001; % 高精度
    
    % 物理运动计算
    ang_rad = deg2rad(a);
    Vel_Vec = [cos(ang_rad), sin(ang_rad), 0] * v;
    P_Drop = Env.Pos_FY1_Init + Vel_Vec * td;
    P_Pop = P_Drop + Vel_Vec * ty;
    P_Pop(3) = P_Pop(3) - 0.5 * Env.g * ty^2;
    
    t_pop = td + ty;
    t_max_sim = Env.Dist_Total_M1 / Env.V_M1;
    t_vec = 0 : dt : t_max_sim;
    
    valid_mask = false(size(t_vec));
    
    % 向量化检查每一时刻
    for k = 1:length(t_vec)
        t_curr = t_vec(k);
        
        % 1. 导弹位置
        if t_curr * Env.V_M1 >= Env.Dist_Total_M1
            P_M = Env.Pos_FakeTarget;
        else
            P_M = Env.Pos_M1_Init + Env.Dir_M1 * (Env.V_M1 * t_curr);
        end
        
        % 2. 烟雾位置
        if t_curr >= t_pop && (t_curr - t_pop) <= Env.Time_smoke_last
            t_rel = t_curr - t_pop;
            P_Smk = P_Pop - [0, 0, Env.V_smoke_sink] * t_rel;
            
            % 3. 遮挡判定
            P1 = P_M;
            P2 = Env.Pos_TrueTarget;
            Q = P_Smk;
            
            v_vec = P2 - P1;
            w_vec = Q - P1;
            
            c1 = dot(w_vec, v_vec);
            c2 = dot(v_vec, v_vec);
            
            if c1 <= 0
                dist = norm(Q - P1);
            elseif c2 <= c1
                dist = norm(Q - P2);
            else
                b = c1 / c2;
                Pb = P1 + b * v_vec;
                dist = norm(Q - Pb);
            end
            
            if dist <= Env.R_smoke
                valid_mask(k) = true;
            end
        end
    end
    
    % 提取结果
    if any(valid_mask)
        idx = find(valid_mask);
        t_start = t_vec(idx(1));
        t_end = t_vec(idx(end));
        valid_dur = sum(valid_mask) * dt;
    else
        t_start = NaN; t_end = NaN; valid_dur = 0;
    end
end

%% --- 内部微型 PSO 求解器 ---
function [best_pos, best_val] = Run_Micro_PSO(v, lb, ub, Env)
    % 粒子群参数
    n_part = 30;
    n_iter = 50;
    w = 0.6; c1 = 1.5; c2 = 1.5;
    n_vars = 3;
    
    % 初始化
    pos = repmat(lb, n_part, 1) + rand(n_part, n_vars) .* repmat(ub-lb, n_part, 1);
    
    % [种子注入] 
    if v < 90
        pos(1,:) = [176.88, 0.01, 2.5]; 
    else
        pos(1,:) = [178.46, 0.01, 3.3]; 
    end
    
    vel = zeros(n_part, n_vars);
    pbest_pos = pos;
    pbest_val = zeros(n_part, 1);
    gbest_pos = zeros(1, n_vars);
    gbest_val = -1e9; 
    
    % 评估初始种群
    for i = 1:n_part
        val = -Tactical_Sim_Engine(v, pos(i,1), pos(i,2), pos(i,3), Env.dt, Env);
        pbest_val(i) = val;
        if val > gbest_val
            gbest_val = val;
            gbest_pos = pos(i,:);
        end
    end
    
    % 迭代
    for t = 1:n_iter
        for i = 1:n_part
            r1 = rand(1, n_vars); r2 = rand(1, n_vars);
            vel(i,:) = w*vel(i,:) + c1*r1.*(pbest_pos(i,:)-pos(i,:)) + c2*r2.*(gbest_pos-pos(i,:));
            pos(i,:) = pos(i,:) + vel(i,:);
            pos(i,:) = max(pos(i,:), lb);
            pos(i,:) = min(pos(i,:), ub);
            
            val = -Tactical_Sim_Engine(v, pos(i,1), pos(i,2), pos(i,3), Env.dt, Env);
            
            if val > pbest_val(i)
                pbest_val(i) = val;
                pbest_pos(i,:) = pos(i,:);
            end
            if val > gbest_val
                gbest_val = val;
                gbest_pos = pos(i,:);
            end
        end
    end
    best_pos = gbest_pos;
    best_val = gbest_val;
end

%% --- 仿真核函数  ---
function score = Tactical_Sim_Engine(v, a, td, ty, dt, Env)
    ang_rad = deg2rad(a);
    Dir_Vec = [cos(ang_rad), sin(ang_rad), 0];
    Vel_Vec = Dir_Vec * v;
    P_Drop = Env.Pos_FY1_Init + Vel_Vec * td;
    P_Pop = P_Drop + Vel_Vec * ty;
    P_Pop(3) = P_Pop(3) - 0.5 * Env.g * ty^2;
    if P_Pop(3) < 0, score = 0; return; end
    
    t_pop = td + ty;
    t_start = max(0, t_pop);
    t_end = min(Env.Dist_Total_M1/Env.V_M1, t_pop + Env.Time_smoke_last);
    if t_start >= t_end, score = 0; return; end
    
    t_vec = t_start : dt : t_end;
    if isempty(t_vec), score=0; return; end
    
    d_m_vec = Env.V_M1 * t_vec;
    P_M_mat = Env.Pos_M1_Init' + Env.Dir_M1' * d_m_vec; 
    P_Smk_mat = P_Pop' - [0;0;Env.V_smoke_sink] * (t_vec - t_pop);
    
    V_LOS_mat = Env.Pos_TrueTarget' - P_M_mat;
    W_mat = P_Smk_mat - P_M_mat;
    c1 = sum(W_mat .* V_LOS_mat, 1);
    c2 = sum(V_LOS_mat .* V_LOS_mat, 1);
    b = c1 ./ c2;
    
    Pb = P_M_mat + V_LOS_mat .* b;
    idx_less = b < 0; if any(idx_less), Pb(:, idx_less) = P_M_mat(:, idx_less); end
    idx_more = b > 1; if any(idx_more), Pb(:, idx_more) = repmat(Env.Pos_TrueTarget', 1, sum(idx_more)); end
    
    dists_sq = sum((P_Smk_mat - Pb).^2, 1);
    count = sum(dists_sq <= Env.R_smoke^2);
    score = -(count * dt);
end