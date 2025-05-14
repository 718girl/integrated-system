clear;clc;
% 连续时间动力学模型
syms rx ry rz vx vy vz m lambda mu1 mu2 phi psi real
syms dlambda dmu1 dmu2 dphi dpsi real
x = [rx; ry; rz; vx; vy; vz; m; lambda; mu1; mu2; phi; psi];
u = [dlambda; dmu1; dmu2; dphi; dpsi]; 

% 动力学方程
f = sym('f', [12,1]);    % 状态导数

Tmax = 912000 - 67000 * exp(-ry/7110);
T = lambda.*Tmax;

% 箭体坐标系推力分量
Tx_body = T .* cosd(mu1) .* cosd(mu2);
Ty_body = T .* cosd(mu1) .* sind(mu2);
Tz_body = -T .* sind(mu1);

% 转换为着陆坐标系
Fx = Tx_body .* cosd(phi) .* cosd(psi) - Ty_body .* sind(phi) + Tz_body .*cosd(phi).* sind(psi);
Fy = Tx_body .* sind(phi) .* cosd(psi) + Ty_body .* cosd(phi) + Tz_body .*sind(phi).* sind(psi);
Fz = -Tx_body .* sind(psi) + Tz_body .* cosd(psi);

% 气动阻力(着陆坐标系)
rho = 1.225 * exp(-ry/7110);
V = sqrt(vx.^2 + vy.^2 + vz.^2);
Dx = -0.5 .* rho .* 10.52 * 1.5 .* V .* vx;
Dy = -0.5 .* rho .* 10.52 * 1.5 .* V .* vy;
Dz = -0.5 .* rho .* 10.52 * 1.5 .* V .* vz;

isp = 312 - 29 * exp(-ry/7110);

% 动力学方程（着陆坐标系）
f(1) = vx;    
f(2) = vy;    
f(3) = vz;     
f(4) = (Fx + Dx) ./ m;  
f(5) = (Fy + Dy) ./ m - 9.807;   
f(6) = (Fz + Dz) ./ m;                 
f(7) = -T ./ (isp * 9.807);                 
f(8) = dlambda;               
f(9) = dmu1;                
f(10) = dmu2;                  
f(11) = dphi;                 
f(12) = dpsi;                 

A = jacobian(f, x);  % 系统矩阵 (12×12)
B = jacobian(f, u);  % 控制矩阵 (12×5)

% 初始化参数
% 权重矩阵
Q = diag([10 10 10 10 10 10 0.01 1 1 1 100 100]); % 状态权重
R = diag([1 1 1 1 1]);       % 控制增量权重

% 约束条件
u_min = [-0.3; -5; -5; -5; -5];   % 控制量下限
u_max = [ 0.3;  5;  5;  5;  5];   % 控制量上限
x_min = [-2000;  18.6; -1000; -800; -1000; -500;  28000;  0.3; -5; -5;  60; -30]; % 状态下限
x_max = [ 2000;  8000;  1000;  800;  1000;  500;  49000;  1.2;  5;  5; 120;  30]; % 状态上限

tic;
for i=1:10
    % 加载参考轨迹
    ref_file_name = fullfile('F:\matlab\bin\rocket\reference_trajectory\', ['reference', num2str(i), '.mat']);
    ref_data = load(ref_file_name);
    t_ref = ref_data.phase1.time;
    x_ref = ref_data.phase1.state;  
    u_ref = ref_data.phase1.control;
    x_ref_interp = @(t) interp1(t_ref, x_ref, t, 'linear', 'extrap'); % 默认外推可能返回 NaN
    x_ref_interp = @(t) (t <= t_ref(end)) .* x_ref_interp(t) + ...    % 时间未超限时正常插值
                        (t > t_ref(end))  .* x_ref(end,:);           % 超限时返回最后一个值
    u_ref_interp = @(t) interp1(t_ref, u_ref, t, 'linear', 'extrap');
    u_ref_interp = @(t) (t <= t_ref(end)) .* u_ref_interp(t) + ...    % 时间未超限时正常插值
                        (t > t_ref(end))  .* u_ref(end,:);           % 超限时返回最后一个值
    % 初始状态、控制获取
    ref_total_num = length(t_ref);      % 参考轨迹中状态点数量
    index = randperm(ref_total_num-1, 10); % 每个轨迹中取10个不同的初始点

    % 状态偏差
    diff = [0.5, 0.4, 0.3, 0.2, 0.1, 0.05];
    
    for j=1:10
        % 仿真参数
        T_sim = t_ref(end) - t_ref(index(j)); % 总仿真时间
        Ts = 0.2;                      % 采样时间
        num = ceil(T_sim/Ts);     % 总采样数
        N = 20;             % 预测时域
        Nc = 15;            % 控制时域
        nx = 12;            % 状态维度
        nu = 5;             % 控制维度
        time = zeros(num+1,1);
        x_actual = zeros(nx, num+1);
        u_actual = zeros(nu, num);  
        x_0 = x_ref_interp(t_ref(index(j)))';        
        u_prev = u_ref_interp(t_ref(index(j)))'; % 初始控制量
        
        % 不同状态变量
        for p = 1:6
            % 不同偏差大小
            for q = 1:length(diff)
                % 每次不同的初始状态点都要初始化
                MPC_History = struct(...
                    'Time', cell(num, 1), ...
                    'PredictedStates', cell(num, 1), ...
                    'PredictedControls', cell(num, 1) ...
                );
                x_mid = x_0;
                x_mid(p) = (1 + diff(q)) * x_mid(p);
                % 边界约束
                if x_mid(p) > 0
                    x_mid(p) = min(x_mid(p), x_max(p));
                else
                    x_mid(p) = max(x_mid(p), x_min(p));
                end
                x_actual(:,1) = x_mid;    % 初始实际状态
                
                % 主循环
                for k = 1:num
                    t_current = t_ref(index(j)) + (k-1)*Ts;
                    time(k) = t_current;

                    % 获取当前参考轨迹
                    x_ref_k = x_ref_interp(t_current)';
                    u_ref_k = u_ref_interp(t_current)';

                    % --- 核心修改1：基于当前实际状态线性化 ---
                    [A_d, B_d] = discrete(x_actual(:,k), u_prev, A, B, x, u, nx, nu, Ts);

                    % --- 核心修改2：构建预测模型 ---
                    [Phi, Gamma] = build_prediction_matrices(A_d, B_d, N, Nc);

                    % --- 核心修改3：QP问题定义（直接跟踪参考轨迹）---
                    H = Gamma' * kron(eye(N), Q) * Gamma + kron(eye(Nc), R);
                    f = 2 * Gamma' * kron(eye(N), Q) * (Phi * (x_actual(:,k) - x_ref_k));
                    H = (H + H')/2; % 确保对称

                    % 控制量约束（绝对量）
                    u_lb = repmat(u_min, Nc, 1);
                    u_ub = repmat(u_max, Nc, 1);

                    % 状态约束（绝对量）
                    x_lb = repmat(x_min - x_ref_k, N, 1); % 参考轨迹相关约束
                    x_ub = repmat(x_max - x_ref_k, N, 1);

                    % 构建约束矩阵
                    A_con = [Gamma; -Gamma]; % 状态约束
                    b_con = [x_ub - Phi*(x_actual(:,k)-x_ref_k); 
                            -x_lb + Phi*(x_actual(:,k)-x_ref_k)];

                    % CVX求解
                    cvx_begin quiet
                        variable U(Nc*nu, 1)
                        minimize(0.5*quad_form(U, H) + f'*U)
                        subject to
                            u_lb <= U <= u_ub
                            A_con * U <= b_con
                    cvx_end

                    % ===== 记录预测结果 =====
                    % 提取预测控制序列 [Nc×nu]
                    predicted_controls = reshape(U, nu, Nc)';

                    % 提取预测状态序列 [N×nx]
                    predicted_states = (Phi * x_actual(:,k) + Gamma * U)';

                    % 保存到结构体
                    MPC_History(k).Time = t_current;
                    MPC_History(k).PredictedStates = predicted_states;
                    MPC_History(k).PredictedControls = predicted_controls;

                    % 提取控制量
                    slove_status = cvx_status;
                    if strcmp(cvx_status, 'Solved')
                        u_opt = U(1:nu);
                    else
                        warning('求解失败: %s', cvx_status);
                        break;
                    end

                    % 更新控制量
                    u_actual(:,k) = u_opt; % 注意这里用绝对量
                    u_prev = u_actual(:,k);

                    % 仿真下一步状态（使用非线性模型）
                    x_next = simulate_nonlinear(x_actual(:,k), u_actual(:,k), Ts);
                    x_actual(:,k+1) = x_next;
                    disp(x_next');
                end

                save_file_dir = 'F:\matlab\bin\rocket\track_yitihua\';
                save(fullfile(save_file_dir, [num2str(i), '_', ...
                    'ceshi', '_MPC_History.mat']), 'MPC_History');
                save(fullfile(save_file_dir, [num2str(i), '_', ...
                    'ceshi', '_x_actual.mat']), 'x_actual');
                save(fullfile(save_file_dir, [num2str(i), '_', ...
                    'ceshi', '_u_actual.mat']), 'u_actual');
                save(fullfile(save_file_dir, [num2str(i), '_', ...
                    'ceshi', '_slove_status.mat']), 'slove_status');
                disp(i);
                disp(j);
                disp(p);
                disp(q);
                if strcmp(slove_status, 'Solved')
                    break;
                end
            end                 
         end
    end    
end
total_time = toc;
disp(total_time);

%% 辅助函数
% 离散化模型矩阵
function [A_d, B_d] = discrete(x_k, u_k, A, B, x_sym, u_sym, nx, nu, Ts)
    % 在当前实际状态线性化
    A_linear = subs(A, [x_sym; u_sym], [x_k; u_k]);
    B_linear = subs(B, [x_sym; u_sym], [x_k; u_k]);
    A_num = double(A_linear);
    B_num = double(B_linear);
    
    % 离散化
    M = expm([A_num, B_num; zeros(nu, nx+nu)] * Ts);
    A_d = M(1:nx,1:nx);
    B_d = M(1:nx,nx+1:end);
end

% MPC预测矩阵
function [Phi, Gamma] = build_prediction_matrices(A, B, N, Nc)
    nx = size(A,1);
    nu = size(B,2);
    Phi = zeros(nx*N, nx);
    Gamma = zeros(nx*N, nu*Nc);
    
    % 计算A的幂次
    A_pows = cell(N,1);
    A_pows{1} = A;
    for i = 2:N
        A_pows{i} = A_pows{i-1} * A;
    end
    
    % 填充Phi矩阵
    for i = 1:N
        Phi((i-1)*nx+1:i*nx, :) = A_pows{i};
    end
    
    % 填充Gamma矩阵
    for i = 1:N
        for j = 1:min(i,Nc)
            if i == j
                Gamma_block = B;
            else
                Gamma_block = A_pows{i-j} * B;
            end
            Gamma((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = Gamma_block;
        end
    end
end

% 龙格-库塔法数值积分
function x_next = simulate_nonlinear(x, u, Ts)
    k1 = f_nonlinear(x, u)';
    k2 = f_nonlinear(x + Ts/2*k1, u)';
    k3 = f_nonlinear(x + Ts/2*k2, u)';
    k4 = f_nonlinear(x + Ts*k3, u)';
    x_next = x + Ts/6*(k1 + 2*k2 + 2*k3 + k4);
end

function f = f_nonlinear(x,u)
    % 状态变量分解
    rx = x(1); ry = x(2); rz = x(3);
    vx = x(4); vy = x(5); vz = x(6);
    m = x(7); lambda = x(8); mu1 = x(9); mu2 = x(10); phi = x(11); psi = x(12);
    
    % 控制量分解
    dlambda = u(1); dmu1 = u(2); dmu2 = u(3); dphi = u(4); dpsi = u(5);
    Tmax = 912000 - 67000 * exp(-ry/7110);
    T = lambda.*Tmax;

    % 箭体坐标系推力分量
    Tx_body = T .* cosd(mu1) .* cosd(mu2);
    Ty_body = T .* cosd(mu1) .* sind(mu2);
    Tz_body = -T .* sind(mu1);

    % 转换为着陆坐标系
    Fx = Tx_body .* cosd(phi) .* cosd(psi) - Ty_body .* sind(phi) + Tz_body .*cosd(phi).* sind(psi);
    Fy = Tx_body .* sind(phi) .* cosd(psi) + Ty_body .* cosd(phi) + Tz_body .*sind(phi).* sind(psi);
    Fz = -Tx_body .* sind(psi) + Tz_body .* cosd(psi);

    % 气动阻力(着陆坐标系)
    rho = 1.225 * exp(-ry/7110);
    V = sqrt(vx.^2 + vy.^2 + vz.^2);
    Dx = -0.5 .* rho .* 10.52 * 1.5 .* V .* vx;
    Dy = -0.5 .* rho .* 10.52 * 1.5 .* V .* vy;
    Dz = -0.5 .* rho .* 10.52 * 1.5 .* V .* vz;

    isp = 312 - 29 * exp(-ry/7110);

    % 动力学方程（着陆坐标系）
    f(1) = vx;    
    f(2) = vy;    
    f(3) = vz;     
    f(4) = (Fx + Dx) ./ m;  
    f(5) = (Fy + Dy) ./ m - 9.807;   
    f(6) = (Fz + Dz) ./ m;                 
    f(7) = -T ./ (isp * 9.807);                 
    f(8) = dlambda;               
    f(9) = dmu1;                
    f(10) = dmu2;                  
    f(11) = dphi;                 
    f(12) = dpsi;   
end