%% ========================================================================
%  v4_observer.tex理論に厳密準拠したシミュレーション
% =========================================================================
% [理論]
%   動力学: ṗ_i = v_i, v̇_i = u_i
%   推定器: ̇p̂_i = v_i + ξ_i
%          ̇v̂_i = u_i + K_obs(v_i - v̂_i)
%   制御器: u_i = p̈_i* - K_p(p̂_i - p_i*) - K_v(v_i - ṗ_i*) + ψ_i
% =========================================================================

%% 1. 初期化
clear; clc; close all;
fprintf('=== v4_observer.tex シミュレーション開始 ===\n\n');

%% 2. 設定の読み込み
config = define_trajectory_simple();

fprintf('=== 設定情報 ===\n');
fprintf('[システム]\n');
fprintf('  リーダー数: %d\n', config.m);
fprintf('  フォロワー数: %d\n', config.num_followers);
fprintf('  シミュレーション時間: %.2f秒\n', config.t_end);
fprintf('\n[軌道設定]\n');
fprintf('  軌道タイプ: %s\n', config.trajectory.type);
if strcmp(config.trajectory.type, 'constant')
    fprintf('  一定加速度ノルム: %.3f m/s^2\n', norm(config.target_accel{1}));
elseif strcmp(config.trajectory.type, 'time_varying')
    fprintf('  加速度振幅: %.3f m/s^2\n', config.trajectory.accel_amplitude);
    fprintf('  周波数: %.3f rad/s\n', config.trajectory.frequency);
elseif strcmp(config.trajectory.type, 'circular')
    fprintf('  円軌道半径: %.3f m\n', config.trajectory.radius);
    fprintf('  角速度: %.3f rad/s\n', config.trajectory.angular_velocity);
end
fprintf('\n[ゲイン設定（v4_observer.tex Theorem 4.2）]\n');
fprintf('  K_obs (オブザーバ): %.2f\n', config.K_obs);
fprintf('  K_p (位置制御): %.2f\n', config.K_p);
fprintf('  K_v (速度制御): %.2f\n', config.K_v);
fprintf('  γ (λ調整): %.2f\n', config.gamma);
fprintf('  λ_max (λ飽和): %.2f\n', config.lambda_max);
fprintf('  w_xi (ξゲイン): %.2f\n', config.w_xi);
fprintf('  w_psi (ψゲイン): %.2f\n\n', config.w_psi);

%% 3. 初期状態の設定（v4_observer.tex 仮定1: p̂_i(0) = p_i(0), v̂_i(0) = v_i(0)）
% 状態変数: X = [p_f; v_f; p_hat_f; v_hat_f]

% 実位置の初期値
p_f_initial = config.p_f_initial(:);  % [3*num_f x 1]

% 実速度の初期値（零速度から開始）
v_f_initial = zeros(size(p_f_initial));  % [3*num_f x 1]

% 推定位置の初期値（仮定1: p̂_i(0) = p_i(0)）
p_hat_f_initial = zeros(size(p_f_initial));
for i = 1:config.num_followers
    estimated_pos = config.agent_estimated_positions{i};
    p_hat_f_initial((i-1)*3+1:i*3) = estimated_pos;
end

% 推定速度の初期値（仮定1の拡張: ||v̂_i(0) - v_i(0)|| ≤ ε_v）
if config.initial_velocity_error > 0
    % 初期速度推定誤差を追加
    v_hat_f_initial = zeros(size(v_f_initial));
    for i = 1:config.num_followers
        random_direction = randn(3, 1);
        random_direction = random_direction / norm(random_direction);
        v_hat_f_initial((i-1)*3+1:i*3) = v_f_initial((i-1)*3+1:i*3) + ...
            config.initial_velocity_error * random_direction;
    end
else
    % 初期誤差なし（仮定1の特殊ケース: ε_v = 0）
    v_hat_f_initial = v_f_initial;  % [3*num_f x 1]
end

% 初期状態ベクトルの構成
X0 = [p_f_initial; v_f_initial; p_hat_f_initial; v_hat_f_initial];

expected_dim = 4 * 3 * config.num_followers;  % 4ブロック: p, v, p_hat, v_hat
fprintf('=== 初期状態 ===\n');
fprintf('状態ベクトル次元: %d (期待値: %d)\n', length(X0), expected_dim);

if length(X0) ~= expected_dim
    error('状態ベクトルの次元が不正です！');
end

% エージェント初期設定の表示
[p_star_initial, v_star_initial, a_star_initial] = config.get_target_positions(config.t_start);

fprintf('\n--- フォロワーの初期設定 ---\n');
for i = 1:config.num_followers
    actual_pos = p_f_initial((i-1)*3+1:i*3);
    estimated_pos = p_hat_f_initial((i-1)*3+1:i*3);
    actual_vel = v_f_initial((i-1)*3+1:i*3);
    estimated_vel = v_hat_f_initial((i-1)*3+1:i*3);
    target_pos_initial = p_star_initial{config.m + i};
    target_vel_initial = v_star_initial{config.m + i};
    target_accel = a_star_initial{config.m + i};
    
    estimation_error_p = norm(estimated_pos - actual_pos);
    estimation_error_v = norm(estimated_vel - actual_vel);
    
    fprintf('\nフォロワー%d:\n', config.m+i);
    fprintf('  実位置:     [%.2f, %.2f, %.2f]\n', actual_pos);
    fprintf('  推定位置:   [%.2f, %.2f, %.2f] (誤差: %.3e m)\n', estimated_pos, estimation_error_p);
    fprintf('  実速度:     [%.2f, %.2f, %.2f]\n', actual_vel);
    fprintf('  推定速度:   [%.2f, %.2f, %.2f] (誤差: %.3e m/s)\n', estimated_vel, estimation_error_v);
    fprintf('  目標位置:   [%.2f, %.2f, %.2f]\n', target_pos_initial);
    fprintf('  目標速度:   [%.2f, %.2f, %.2f]\n', target_vel_initial);
    fprintf('  目標加速度: [%.2f, %.2f, %.2f] (ノルム: %.3f m/s^2)\n', ...
        target_accel, norm(target_accel));
end
fprintf('\n');

%% 4. シミュレーションの実行
fprintf('=== RK4シミュレーション実行 ===\n');
h = 0.01;  % タイムステップ [s]
Nt = ceil((config.t_end - config.t_start) / h);
t = (0:Nt)' * h + config.t_start;

% 履歴の初期化
X_history = zeros(Nt+1, numel(X0));
psi_history = zeros(Nt+1, config.num_followers * 3);
tau_history = zeros(Nt+1, config.num_followers);
xi_history = zeros(Nt+1, config.num_followers * 3);

X = X0;
X_history(1,:) = X0';

% 初期状態のξ、psi、tauを記録
num_f = config.num_followers;
dim = 3;
p_f_0 = X0(1:num_f*dim);
v_f_0 = X0(num_f*dim+1:2*num_f*dim);
p_hat_f_0 = X0(2*num_f*dim+1:3*num_f*dim);
v_hat_f_0 = X0(3*num_f*dim+1:4*num_f*dim);

[p_star_all_0, ~, ~] = config.get_target_positions(t(1));
p_l_0 = cell2mat(p_star_all_0(1:config.m));

[xi_0, psi_0, tau_0, debug_info_0] = calculate_control_logic(t(1), p_f_0, v_f_0, p_hat_f_0, v_hat_f_0, p_l_0(:), config);
xi_history(1,:) = xi_0(:)';
psi_history(1,:) = psi_0(:)';
tau_history(1,:) = tau_0;

fprintf('初期状態:');
fprintf(' ξノルム=%.3e, ψノルム=%.3e, τ合計=%.3e\n', norm(xi_0), norm(psi_0), sum(tau_0));

% RK4積分ループ
for k = 1:Nt
    tk = t(k);
    
    % RK4による状態更新
    k1 = system_dynamics(tk,       X,          config);
    k2 = system_dynamics(tk+0.5*h, X+0.5*h*k1, config);
    k3 = system_dynamics(tk+0.5*h, X+0.5*h*k2, config);
    k4 = system_dynamics(tk+h,     X+h*k3,     config);
    X = X + h/6*(k1 + 2*k2 + 2*k3 + k4);
    X_history(k+1,:) = X';
    
    % 更新後の状態でξ、psi、tauを記録
    p_f_new = X(1:num_f*dim);
    v_f_new = X(num_f*dim+1:2*num_f*dim);
    p_hat_f_new = X(2*num_f*dim+1:3*num_f*dim);
    v_hat_f_new = X(3*num_f*dim+1:4*num_f*dim);
    
    [p_star_all, ~, ~] = config.get_target_positions(t(k+1));
    p_l_new = cell2mat(p_star_all(1:config.m));
    
    [xi_new, psi_new, tau_new, debug_info] = calculate_control_logic(t(k+1), p_f_new, v_f_new, p_hat_f_new, v_hat_f_new, p_l_new(:), config);
    xi_history(k+1,:) = xi_new(:)';
    psi_history(k+1,:) = psi_new(:)';
    tau_history(k+1,:) = tau_new;
    
    % 重み行列デバッグ情報の記録
    if k == 1
        % 初期化
        weight_debug.case_ids = cell(Nt+1, config.num_followers);
        weight_debug.Hij_det = zeros(Nt+1, config.num_followers);
        weight_debug.Hik_det = zeros(Nt+1, config.num_followers);
        weight_debug.Hij_trace = zeros(Nt+1, config.num_followers);
        weight_debug.Hik_trace = zeros(Nt+1, config.num_followers);
        weight_debug.is_collinear = false(Nt+1, config.num_followers);
        weight_debug.occlusion_j = false(Nt+1, config.num_followers);
        weight_debug.occlusion_k = false(Nt+1, config.num_followers);
        weight_debug.lambda_norm = zeros(Nt+1, config.num_followers);  % λノルム履歴
        
        % 初期状態のデバッグ情報を記録
        weight_debug.case_ids(1,:) = debug_info_0.case_ids;
        weight_debug.Hij_det(1,:) = debug_info_0.Hij_det;
        weight_debug.Hik_det(1,:) = debug_info_0.Hik_det;
        weight_debug.Hij_trace(1,:) = debug_info_0.Hij_trace;
        weight_debug.Hik_trace(1,:) = debug_info_0.Hik_trace;
        weight_debug.is_collinear(1,:) = debug_info_0.is_collinear;
        weight_debug.occlusion_j(1,:) = debug_info_0.occlusion_j;
        weight_debug.occlusion_k(1,:) = debug_info_0.occlusion_k;
        weight_debug.lambda_norm(1,:) = debug_info_0.lambda_norm;
    end
    
    % 更新後の状態のデバッグ情報を記録
    weight_debug.case_ids(k+1,:) = debug_info.case_ids;
    weight_debug.Hij_det(k+1,:) = debug_info.Hij_det;
    weight_debug.Hik_det(k+1,:) = debug_info.Hik_det;
    weight_debug.Hij_trace(k+1,:) = debug_info.Hij_trace;
    weight_debug.Hik_trace(k+1,:) = debug_info.Hik_trace;
    weight_debug.is_collinear(k+1,:) = debug_info.is_collinear;
    weight_debug.occlusion_j(k+1,:) = debug_info.occlusion_j;
    weight_debug.occlusion_k(k+1,:) = debug_info.occlusion_k;
    weight_debug.lambda_norm(k+1,:) = debug_info.lambda_norm;
    
    % 進捗表示
    if mod(k, 500) == 0
        fprintf('  進捗: %5.1f%% (t=%6.2f秒)\n', k/Nt*100, tk);
    end
end

fprintf('シミュレーション完了\n\n');

%% 5. シミュレーション結果の解析
fprintf('=== シミュレーション結果解析 ===\n');
fprintf('総ステップ数: %d\n', Nt);
fprintf('時間範囲: %.2f - %.2f [s]\n', config.t_start, config.t_end);

% 状態の抽出
p_f_history = X_history(:, 1:num_f*dim);
v_f_history = X_history(:, num_f*dim+1:2*num_f*dim);
p_hat_f_history = X_history(:, 2*num_f*dim+1:3*num_f*dim);
v_hat_f_history = X_history(:, 3*num_f*dim+1:4*num_f*dim);

% 推定誤差の計算
estimation_error_p = p_hat_f_history - p_f_history;
estimation_error_v = v_hat_f_history - v_f_history;

% 最大推定誤差
max_error_p = max(vecnorm(reshape(estimation_error_p', dim, []), 2, 1));
max_error_v = max(vecnorm(reshape(estimation_error_v', dim, []), 2, 1));

fprintf('\n[推定誤差の最大値（理論値: 0）]\n');
fprintf('  位置推定誤差: %.6e [m]\n', max_error_p);
fprintf('  速度推定誤差: %.6e [m/s]\n', max_error_v);

if max_error_p < 1e-6 && max_error_v < 1e-6
    fprintf('  ✅ PASS: 推定誤差は理論通りほぼ零\n');
else
    fprintf('  ⚠️  WARNING: 推定誤差が大きい\n');
end

% 軌道長の計算
fprintf('\n[実際の軌道長]\n');
for i = 1:config.num_followers
    idx = (i-1)*dim+1:i*dim;
    p_traj = p_f_history(:, idx);
    traj_length = sum(vecnorm(diff(p_traj, 1, 1)', 2, 1));
    fprintf('  フォロワー%d: %.3f m\n', config.m+i, traj_length);
end

fprintf('\n');

%% 6. 結果の可視化
fprintf('結果をプロット中...\n');
plot_results(t, X_history, xi_history, psi_history, tau_history, config, weight_debug);

fprintf('プロット完了\n');
fprintf('=== 処理完了 ===\n');
