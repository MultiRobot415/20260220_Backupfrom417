function dX_dt = system_dynamics(t, X, config)
    % =========================================================================
    % [機能] v4_observer.tex理論に厳密準拠した二次系システムの状態方程式
    % [理論式]
    %   動力学（式ref{eq:dynamics}）:
    %     ṗ_i = v_i
    %     v̇_i = u_i
    %   推定器（式ref{eq:est_p}, ref{eq:est_v}）:
    %     ˙p̂_i = v_i + ξ_i
    %     ˙v̂_i = u_i + K_obs(v_i - v̂_i)
    %   制御器（式ref{eq:controller}）:
    %     u_i = p̈_i* - K_p(p̂_i - p_i*) - K_v(v_i - v̂_i) + ψ_i
    % [状態変数] X = [p_f; v_f; p_hat_f; v_hat_f]
    % =========================================================================

    num_f = config.num_followers;
    dim = 3;
    
    % ===== 状態の抽出 =====
    p_f     = X(1 : num_f*dim);                     % 実位置
    v_f     = X(num_f*dim+1 : 2*num_f*dim);        % 実速度
    p_hat_f = X(2*num_f*dim+1 : 3*num_f*dim);      % 推定位置
    v_hat_f = X(3*num_f*dim+1 : 4*num_f*dim);      % 推定速度
    
    % ===== 目標軌道の取得 =====
    [p_star_all, v_star_all, a_star_all] = config.get_target_positions(t);
    p_l = cell2mat(p_star_all(1:config.m));                    % リーダー位置
    p_f_star = cell2mat(p_star_all(config.m+1:end));          % フォロワー目標位置
    v_f_star = cell2mat(v_star_all(config.m+1:end));          % フォロワー目標速度
    a_f_star = cell2mat(a_star_all(config.m+1:end));          % フォロワー目標加速度
    
    % ベクトル化
    p_l_vec = p_l(:);              % [3*m x 1]
    p_f_star_vec = p_f_star(:);    % [3*num_f x 1]
    v_f_star_vec = v_f_star(:);    % [3*num_f x 1]
    a_f_star_vec = a_f_star(:);    % [3*num_f x 1]
    
    % ===== 制御ロジックの計算 =====
    % ξとψを取得（推定位置ベース）
    [xi, psi, ~, ~] = calculate_control_logic(t, p_f, v_f, p_hat_f, v_hat_f, p_l_vec, config);
    
    % 行列形式からベクトル形式へ変換
    xi_vec = xi(:);    % [3*num_f x 1]
    psi_vec = psi(:);  % [3*num_f x 1]
    
    % ===== ゲインの設定 =====
    % v4_observer.tex Theorem 4.2のゲイン条件: K_obs > 0, K_p > 0, K_v > 0
    if ~isfield(config, 'K_obs'), config.K_obs = 10; end
    if ~isfield(config, 'K_p'), config.K_p = 5; end
    if ~isfield(config, 'K_v'), config.K_v = 10; end
    
    % 実装上の調整ゲイン（デフォルト: 1.0）
    if ~isfield(config, 'w_xi'), config.w_xi = 1.0; end
    if ~isfield(config, 'w_psi'), config.w_psi = 1.0; end
    
    % ===== 制御入力の計算（式ref{eq:controller}）=====
    % u_i = p̈_i* - K_p(p̂_i - p_i*) - K_v(v_i - ṗ_i*) + ψ_i
    %
    % 重要: 速度項は (v_i - ṗ_i*) = (v_i - v_i*) を使用
    %       これにより目標加速度が完全にキャンセルされ、カスケード構造が成立
    u_f = a_f_star_vec ...
          - config.K_p * (p_hat_f - p_f_star_vec) ...
          - config.K_v * (v_f - v_f_star_vec) ...
          + config.w_psi * psi_vec;
    
    % ===== 状態微分の計算 =====
    
    % 1. 実位置の微分（式ref{eq:dynamics}）
    %    ṗ_i = v_i
    p_dot_f = v_f;
    
    % 2. 実速度の微分（式ref{eq:dynamics}）
    %    v̇_i = u_i
    v_dot_f = u_f;
    
    % 3. 推定位置の微分（式ref{eq:est_p}）
    %    ˙p̂_i = v_i + ξ_i
    %
    %    重要: 真の速度v_iを使用（測定可能）
    p_hat_dot_f = v_f + config.w_xi * xi_vec;
    
    % 4. 推定速度の微分（式ref{eq:est_v}）
    %    ˙v̂_i = u_i + K_obs(v_i - v̂_i)
    %
    %    重要: 観測フィードバック項 K_obs(v_i - v̂_i) が推定器の核心
    v_hat_dot_f = u_f + config.K_obs * (v_f - v_hat_f);
    
    % ===== 状態微分ベクトルの構成 =====
    dX_dt = [p_dot_f; v_dot_f; p_hat_dot_f; v_hat_dot_f];
    
end