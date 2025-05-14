% 箭体初始状态
initinal_state = {[-677, 5137, -117, 29, -280, 25, 48573, 0.8, 0, 0, 80, 4],...
    [2000, 6000, 500, -300, -400, 20, 45000, 1.0, 0, 0, 80, 0],...
    [1000, 6000, 300, -200, -300, 20, 49000, 0.8, 0, 0, 90, 0],...
    [500, 5000, -250, 10, -280, -10, 42000, 0.9, 0, 0, 80, 0],...
    [-300, 5000, -180, -50, -300, 30, 42000, 0.8, 0, 0, 90, -1],...
    [-500, 5000, 300, 60, -320, -40, 42000, 0.95, 0, 0, 90, 1],...
    [400, 4000, -200, -100, -300, 15, 38000, 0.8, 0, 0, 85, -1],...
    [-500, 4500, -280, 180, -400, 30, 38000, 0.85, 0, 0, 95, 10],...
    [-200, 3800, 200, 20, -200, -35, 38000, 0.8, 0, 0, 85, 15],...
    [300, 3500, 100, -80, -200, -30, 35000, 0.85, 0, 0, 110, -3]
%     [-300, 300, -80, 50, -80, -5, 29000, 0.38, -4, 2, 70, 20],...
%     [-950, 5500, -320, 200, -380, -60, 47000, 0.98, -4, -4, 105, 29],...
%     [-550, 4800, -210, -70, -290, 28, 43000, 0.82, 3, -2, 88, -8],...
%     [-650, 800, -170, -100, -220, 18, 38000, 0.65, -1, 4, 78, 12],...
%     [-850, 6000, -270, 130, -330, -45, 49000, 0.88, 5, -5, 118, -30],...
%     [-150, 2500, -30, -180, -150, 10, 35000, 0.55, 0, 0, 62, 0],...
%     [-720, 4200, -240, 90, -270, -35, 41000, 0.72, -2, 3, 92, -18],...
    };
gpops_rocket_reentry_full(initinal_state);

function gpops_rocket_reentry_full(initinal_state)
    % 全局参数
    m_stru = 28000;                     % 结构质量(kg)
    lambda_min2 = 0.35;                 % 发动机开度下限
    lambda_max = 1.0;                   % 发动机开度上限
    lambda_rmax = 0.3;                  % 发动机开度变化率最大限度
    mu_max = 5;                         % 发动机摆角最大角度(°)
    mu_rmax = 5;                        % 发动机摆角的最大变化率(°/s)
    w_max3 = 5;                         % 箭体最大旋转角速率3(°/s)
    end_state = [0, 18.6, 0, 0, 0, 0, 28000, 0, 0, 0, 90, 0];
    
    for i=1:1     %length(initinal_state)  
        state = initinal_state{i};
        % 边界、过程状态约束
        bounds.phase.initialtime.lower = 0;
        bounds.phase.initialtime.upper = 0;
        bounds.phase.initialstate.lower = state;
        bounds.phase.initialstate.upper = state;

        bounds.phase.state.lower =  [-2000,      18.6, -1000, -500, -500, -300,  m_stru, lambda_min2,     0,      0,   60, -30];
        bounds.phase.state.upper =  [2000,   state(2),  1000,  500,  500,  300, state(7), lambda_max, mu_max, mu_max,   120, 30];
        bounds.phase.control.lower =  [-lambda_rmax, -mu_rmax, -mu_rmax, -w_max3, -w_max3];
        bounds.phase.control.upper =  [ lambda_rmax,  mu_rmax,  mu_rmax,  w_max3,  w_max3];

        % 论文式(3.25)终端约束
        bounds.phase.finaltime.lower = 0;      
        bounds.phase.finaltime.upper = 50;
        bounds.phase.finalstate.lower = [-25, 18.6, -25, -3, -1, -3, m_stru, lambda_min2,      0,      0,  89.9, -0.1]; 
        bounds.phase.finalstate.upper = [ 25, 18.6,  25,  3,  1,  3, state(7),  lambda_max, mu_max, mu_max,  90.1,  0.1]; % 垂直姿态90±0.1°

        % 初始猜测值
        phase1.time = linspace(0, 50, 50)';
        phase1.states = interp1([0;1], [state; end_state], linspace(0,1,50));
        n = length(phase1.time);
        eval(sprintf('phase1.controls = [zeros(%d, 1) linspace(0,5, %d)'' linspace(0,5,%d)'' linspace(0,5,%d)'' linspace(0,5,%d)''];', n, n, n, n, n));
         
        guess.phase.time = phase1.time;
        guess.phase.state = phase1.states;
        guess.phase.control = phase1.controls;

        %  GPOPS-II主配置
        setup.name = 'RLV_Full_Trajectory';
        setup.functions.continuous = @rocket_dynamics;
        setup.functions.endpoint = @endpoint;
        setup.bounds = bounds;
        setup.guess = guess; 
        setup.nlp.solver = 'snopt';
        setup.derivatives.supplier = 'sparseCD';
        setup.mesh.method = 'hp1';    % hp-PattersonRao
        setup.mesh.tolerance = 1e-6;
        setup.mesh.maxiterations = 5;
        setup.mesh.phase.colpoints = 50;  % * ones(1, 10)
        setup.mesh.phase.fraction = 0.1;  %  * ones(1, 10)

        totalTic = tic;
        output = gpops2(setup);
        totalTime = toc(totalTic);
        disp(totalTime);

        % 数据保存
        file_dir = 'F:\matlab\bin\rocket\reference_trajectory\';
        solution = output.result.solution;
        phase1 = solution.phase;
        save( fullfile(file_dir, ['reference', num2str(i+99), '.mat']), 'phase1');
        disp('当前已完成：');disp(i);
    end
end

% 目标函数
function output = endpoint(input)
    output.objective = -input.phase.finalstate(7);
end

% 动力学方程
function phase_dynamics = rocket_dynamics(input)
    phase = input.phase; 
    t = phase.time;
    x = phase.state;
    u = phase.control;

    rx = x(:,1); ry = x(:,2); rz = x(:,3);
    vx = x(:,4); vy = x(:,5); vz = x(:,6);
    m = x(:,7); lambda = x(:,8); 
    mu1 = x(:,9); mu2 = x(:,10);
    phi = x(:,11); psi = x(:,12);

    dlambda = u(:,1); 
    dmu1 = u(:,2); dmu2 = u(:,3);
    omega_z = u(:,4); omega_y = u(:,5);
    
    % 发动机推力
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
    drx = vx;
    dry = vy;
    drz = vz;
    dvx = (Fx + Dx) ./ m; 
    dvy = (Fy + Dy) ./ m - 9.807;
    dvz = (Fz + Dz) ./ m;  
    dm = -T ./ (isp * 9.807);  
    dphi = omega_z;
    dpsi = omega_y;

    phase_dynamics.dynamics = [drx, dry, drz, dvx, dvy, dvz, dm, dlambda, dmu1, dmu2, dphi, dpsi];
end



% % 画图
% t6 = solution.phase.time;   % 不确定单位
% 
% r6 = solution.phase.state(:,1:3);
% rx = r6(:,1);
% ry = r6(:,2);
% rz = r6(:,3);
% v6 = solution.phase.state(:,4:6);
% m6 = solution.phase.state(:,7);
% lambda6 = solution.phase.state(:,8);
% mu16 = solution.phase.state(:,9);
% mu26 = solution.phase.state(:,10);
% phi6 = solution.phase.state(:,11);
% psi6 = solution.phase.state(:,12);
% 
% dlambda6 = solution.phase.control(:,1);
% dmu16 = solution.phase.control(:,2);
% dmu26 = solution.phase.control(:,3);
% dphi6 = solution.phase.control(:,4);
% dpsi6 = solution.phase.control(:,5);
% 
% % 三维轨迹曲线
% figure;
% plot3(rx, rz, ry, 'k-', 'LineWidth', 2);
% grid on;
% hold on;
% plot3(rx(1), rz(1), ry(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
% plot3(rx(end), rz(end), ry(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
% set(gca, 'FontSize', 14);
% xlabel('x');
% ylabel('z');
% zlabel('y');
% 
% end