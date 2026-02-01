clear; clc; close all;
%% =================================================================
%  第一阶段：拟合 OCV 曲线
%  =================================================================
disp('【阶段 1】正在读取恒流数据 data1.xlsx 以拟合 OCV...');
file_ocv = 'data1.xlsx'; 
% --- 1.1 读取 OCV 数据 ---
[~, sheets] = xlsfinfo(file_ocv);
data_ocv = [];
for i=1:length(sheets)
    try
        tmp = readtable(file_ocv,'Sheet',sheets{i}); 
        if height(tmp)>1000, data_ocv=tmp; break; end
    catch
    end
end

% 确保数据转换为数组
raw_t = table2array(data_ocv(:, 2)); 
raw_i = table2array(data_ocv(:, 7)); 
raw_v = table2array(data_ocv(:, 8));

idx = ~isnan(raw_t) & ~isnan(raw_v);
raw_t=raw_t(idx); raw_i=raw_i(idx); raw_v=raw_v(idx);
if mean(raw_i) < 0, raw_i = -raw_i; end 

% --- 1.2 提取放电段 ---
seg_idx = find(raw_i > 0.08 & raw_i < 0.12 & raw_v > 2.5);
diff_idx = diff(seg_idx); breaks = find(diff_idx > 10);
if isempty(breaks)
    fin_idx = seg_idx; 
else
    [~,p] = max(diff([0;breaks;length(seg_idx)])); 
    if p==1, s=1; e=breaks(1); else, s=breaks(p-1)+1; e=breaks(p); end
    fin_idx = seg_idx(s:e); 
end
v_ocv_data = raw_v(fin_idx);
i_ocv_data = raw_i(fin_idx);
t_ocv_data = raw_t(fin_idx); t_ocv_data = t_ocv_data - t_ocv_data(1);

% --- 1.3 计算 SOC 并拟合 ---
dt_ocv = [0; diff(t_ocv_data)];
ah = cumsum(i_ocv_data .* dt_ocv) / 3600;
Q_total = ah(end); 
soc_vec = 1 - ah/Q_total;
p_ocv = polyfit(soc_vec, v_ocv_data, 5);

fprintf('OCV 拟合完成！电池实测容量 Q = %.4f Ah\n', Q_total);
figure(1); plot(soc_vec, v_ocv_data, 'b.', soc_vec, polyval(p_ocv, soc_vec), 'r');
title('阶段1结果: OCV 曲线拟合'); legend('实测', '拟合'); grid on; drawnow;

%% =================================================================
%  第二阶段：DST 参数辨识
%  =================================================================
disp(' ');
disp('【阶段 2】正在读取 DST 数据 data3.xlsx 进行参数辨识...');
file_dst = 'data3.xlsx'; 
[~, sheets_dst] = xlsfinfo(file_dst);
data_dst = [];
for i=1:length(sheets_dst)
    try
        tmp = readtable(file_dst,'Sheet',sheets_dst{i}); 
        if height(tmp)>1000, data_dst=tmp; break; end
    catch
    end
end

% 核心修正：使用 table2array 并确保列数据正确
dst_t_raw = table2array(data_dst(:, 2)); 
dst_i_raw = table2array(data_dst(:, 7)); 
dst_v_raw = table2array(data_dst(:, 8));

idx_dst = ~isnan(dst_t_raw) & ~isnan(dst_v_raw);
dst_t = dst_t_raw(idx_dst); dst_i = dst_i_raw(idx_dst); dst_v = dst_v_raw(idx_dst);

% 符号修正
if min(dst_i) < -0.5 && max(dst_i) < 5, dst_i = -dst_i; end

% --- 2.2 截取数据 (修正索引从 1 开始) ---
start_idx = 102; % MATLAB 索引必须从 1 开始
end_idx = 11600; 
if end_idx > length(dst_t), end_idx = length(dst_t); end

t_train = dst_t(start_idx:end_idx); t_train = t_train - t_train(1);
i_train = dst_i(start_idx:end_idx);
v_train = dst_v(start_idx:end_idx);

% --- 2.3 粗估初始 SOC ---
soc_axis = 0:0.001:1;
ocv_axis = polyval(p_ocv, soc_axis);
[~, min_k] = min(abs(ocv_axis - v_train(1)));
SOC0_rough = soc_axis(min_k);
fprintf('DST 片段起始电压: %.3f V -> 粗估 SOC: %.2f%%\n', v_train(1), SOC0_rough*100);

% --- 2.4 遗传算法辨识 ---
cost_func = @(theta) objective_function(theta, i_train, v_train, t_train, p_ocv, Q_total);
lb_soc = max(0, SOC0_rough - 0.1); 
ub_soc = min(1, SOC0_rough + 0.1);
%      R0    R1     C1    R2     C2     SOC0
lb = [0.01, 0.001, 100,  0.001, 1000,  lb_soc]; 
ub = [0.30, 0.30, 5000, 0.30, 15000, ub_soc];

options = optimoptions('ga', 'Display', 'iter', 'PopulationSize', 40, 'MaxGenerations', 60);
disp('正在运行 GA 优化...');
theta_opt = ga(cost_func, 6, [], [], [], [], lb, ub, [], options);

%% =================================================================
%  第三阶段：验证与结果输出
%  =================================================================
disp('--------------------------------------------------');
fprintf('辨识结果：R0=%.5f, R1=%.5f, C1=%.2f, R2=%.5f, C2=%.2f, SOC0=%.2f%%\n', ...
    theta_opt(1), theta_opt(2), theta_opt(3), theta_opt(4), theta_opt(5), theta_opt(6)*100);

v_sim = run_model(theta_opt, i_train, t_train, p_ocv, Q_total);
figure(2);
subplot(2,1,1);
plot(t_train, v_train, 'b', 'LineWidth', 1.5); hold on;
plot(t_train, v_sim, 'r--', 'LineWidth', 1.5);
ylabel('电压 (V)'); legend('实测', '模型'); title('DST 验证'); grid on;
subplot(2,1,2);
plot(t_train, (v_train - v_sim)*1000, 'k');
ylabel('误差 (mV)'); xlabel('时间 (s)'); grid on;

%% 核心子函数
function J = objective_function(theta, I, V_exp, t, p_ocv, Qn)
    V_sim = run_model(theta, I, t, p_ocv, Qn);
    J = sqrt(mean((V_exp - V_sim).^2)); 
end

function V = run_model(theta, I, t, p_ocv, Qn)
    R0=theta(1); R1=theta(2); C1=theta(3); R2=theta(4); C2=theta(5); SOC_init=theta(6);
    N = length(t); dt_vec = [0; diff(t)];
    U1 = zeros(N,1); U2 = zeros(N,1); V = zeros(N,1); SOC = zeros(N,1);
    SOC(1) = SOC_init; Q_as = Qn * 3600;
    tau1 = R1 * C1; tau2 = R2 * C2;
    V(1) = polyval(p_ocv, SOC(1)) - I(1) * R0; % 初始时刻电压
    for k = 1:N-1
        dt = dt_vec(k+1); Ik = I(k);
        SOC(k+1) = SOC(k) - (Ik * dt) / Q_as;
        if SOC(k+1)>1, SOC(k+1)=1; elseif SOC(k+1)<0, SOC(k+1)=0; end
        U1(k+1) = U1(k)*exp(-dt/tau1) + Ik*R1*(1-exp(-dt/tau1));
        U2(k+1) = U2(k)*exp(-dt/tau2) + Ik*R2*(1-exp(-dt/tau2));
        V(k+1) = polyval(p_ocv, SOC(k+1)) - I(k+1)*R0 - U1(k+1) - U2(k+1);
    end
end