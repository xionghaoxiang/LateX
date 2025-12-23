clc; clear; close all;

%% 1. 参数设置 
% 烟幕弹参数
R_smoke = 10;           % 有效遮蔽半径 (阈值)
Time_smoke_last = 25;   % 延长一点仿真时间以便看全曲线
V_smoke_sink = 3;       % 烟雾团下沉速度 

g = 9.8; 

% 目标位置
Pos_FakeTarget = [0, 0, 0];          % 假目标 
Pos_TrueTarget_Center = [0, 200, 5]; % 真目标 

%% 2. 计算烟幕弹起爆初始位置 
% FY1 初始状态
Pos_FY1_0 = [17800, 0, 1800];
V_FY1 = 120; 

% 时间节点
t_drop = 1.5;          % 投放时刻
t_delay = 3.6;         % 延时时长
t_pop = t_drop + t_delay; % 起爆时刻 (5.1s)

% 1. 投放点位置 
Pos_Drop = Pos_FY1_0 + [-1, 0, 0] * V_FY1 * t_drop;
% 2. 起爆点初始位置 
Delta_X = -1 * V_FY1 * t_delay; 
Delta_Z = -0.5 * g * t_delay^2;
Pos_Smoke_Init = Pos_Drop + [Delta_X, 0, Delta_Z];

fprintf('Smoke grenade detonation time: t = %.2f s\n', t_pop);

%% 3. 计算有效遮蔽时长 (并记录数据)
% M1 初始状态
Pos_M1_0 = [20000, 0, 2000];
V_M1 = 300;
Vec_M1 = Pos_FakeTarget - Pos_M1_0; 
Dist_M1_Total = norm(Vec_M1);
Dir_M1 = Vec_M1 / Dist_M1_Total;

% 时间积分设置
dt = 0.05; 
valid_time = 0; 
t_start_effective = NaN; % 记录开始的时刻
t_end_effective = NaN;   % 记录结束的时刻

fprintf('Simulating the dynamic occlusion process...\n');

Total_Sim_Time = 30; 

for t_current = 0 : dt : Total_Sim_Time
    
    % A. 更新导弹位置
    dist_flown = V_M1 * t_current;
    if dist_flown >= Dist_M1_Total
        Current_M1_Pos = Pos_FakeTarget; 
    else
        Current_M1_Pos = Pos_M1_0 + Dir_M1 * dist_flown;
    end
    
    % B. 更新烟雾位置 & 计算距离
    if t_current >= t_pop
        t_relative = t_current - t_pop;
        
        if t_relative <= Time_smoke_last
            Sink_Dist = V_smoke_sink * t_relative;
            Current_Smoke_Pos = Pos_Smoke_Init - [0, 0, Sink_Dist];
            
            % 计算遮挡距离
            P1 = Current_M1_Pos;
            P2 = Pos_TrueTarget_Center; 
            Q = Current_Smoke_Pos; 
            
            v = P2 - P1;
            w = Q - P1;
            c1 = dot(w, v);
            c2 = dot(v, v);
            
            if c1 <= 0
                dist = norm(Q - P1);
            elseif c2 <= c1
                dist = norm(Q - P2);
            else
                b = c1 / c2;
                Pb = P1 + b * v;
                dist = norm(Q - Pb);
            end
            
            % --- 捕捉开始和结束时间 ---
            if dist <= R_smoke
                if isnan(t_start_effective)
                    t_start_effective = t_current; 
                end
                t_end_effective = t_current;      
                valid_time = valid_time + dt;
            end
            
        else
            dist = NaN; 
        end
    else
        dist = norm(Pos_Smoke_Init - Current_M1_Pos); 
    end
end
fprintf('>>> Effective shielding duration: %.4f seconds <<<\n', valid_time);

%% 4. 绘图 
figure('Color', 'w', 'Position', [100, 100, 600, 600]); 
hold on; 

bar_height = 0.6;  
fy_index = 1;       
color_fy1 = [100, 100, 255]/255;

if valid_time > 0 && ~isnan(t_start_effective)
    x_patch = [t_start_effective, t_end_effective, t_end_effective, t_start_effective];
    y_patch = [fy_index - bar_height/2, fy_index - bar_height/2, fy_index + bar_height/2, fy_index + bar_height/2];
    
    patch(x_patch, y_patch, color_fy1, 'EdgeColor', 'k', 'LineWidth', 1, 'FaceAlpha', 0.8);
    
    text_str = sprintf('Start: %.1fs', t_start_effective);
    text_y_pos = fy_index - bar_height/2 - 0.2; 
    text(t_start_effective, text_y_pos, text_str, ...
        'FontSize', 10, 'Color', 'k', 'FontWeight', 'normal', 'HorizontalAlignment', 'left');
    % ----------------
else
    fprintf('Warning: No effective shielding generated with current parameters.\n');
end

ax = gca;
ax.YDir = 'normal'; 
ylim([0, 4]); 
yticks([1, 2, 3]);
yticklabels({'FY1', 'FY2', 'FY3'});

xlim([-10, 50]); 
xlabel('时间 (s)', 'FontSize', 11);

grid on;
ax.GridAlpha = 0.3; 
ax.MinorGridAlpha = 0.1;
ax.XMinorGrid = 'on'; 

title_str = sprintf('协同遮蔽时序 (总长: %.2fs)', valid_time);
title(title_str, 'FontSize', 12, 'FontWeight', 'normal');

box on;

hold off;