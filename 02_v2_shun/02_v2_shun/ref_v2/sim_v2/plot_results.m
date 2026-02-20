function plot_results(t, X_history, xi_history, psi_history, tau_history, config, weight_debug)
    % -------------------------------------------------------------------------
    % [機能] シミュレーション結果を可視化する (加速度推定システム対応)
    % [引数]
    %   t: 時間ベクトル
    %   X_history: 状態履歴 [位置; 速度; 推定位置; 推定速度]
    %   xi_history: ξ項の履歴
    %   psi_history: ψ項の履歴
    %   tau_history: τ項の履歴
    %   config: システム設定
    %   weight_debug: 重み行列のデバッグ情報（オプション）
    % -------------------------------------------------------------------------
    
    % weight_debug引数のデフォルト値設定
    if nargin < 6
        weight_debug = [];
    end

    %% 1. データの準備 (加速度推定システム + PID制御対応)
    num_f = config.num_followers;
    dim = 3;
    
    % 状態ベクトルの分解: [位置; 速度; 推定位置; 推定速度; 積分誤差]
    p_f_hist = X_history(:, 1:num_f*dim);                           % 位置履歴
    v_f_hist = X_history(:, num_f*dim+1:2*num_f*dim);              % 速度履歴
    p_hat_f_hist = X_history(:, 2*num_f*dim+1:3*num_f*dim);        % 推定位置履歴
    v_hat_f_hist = X_history(:, 3*num_f*dim+1:4*num_f*dim);        % 推定速度履歴
    e_int_f_hist = X_history(:, 4*num_f*dim+1:end);                % 積分誤差履歴 (PID制御のI項)
    
    % 目標軌道の履歴を計算 (2次系対応)
    p_star_hist = zeros(length(t), config.n*dim);
    v_star_hist = zeros(length(t), config.n*dim);
    a_star_hist = zeros(length(t), config.n*dim);
    
    for i = 1:length(t)
        [p_star_t, v_star_t, a_star_t] = config.get_target_positions(t(i));
        p_star_matrix = cell2mat(p_star_t);
        v_star_matrix = cell2mat(v_star_t);
        a_star_matrix = cell2mat(a_star_t);
        
        p_star_hist(i, :) = p_star_matrix(:)';
        v_star_hist(i, :) = v_star_matrix(:)';
        a_star_hist(i, :) = a_star_matrix(:)';
    end
    
    p_l_hist = p_star_hist(:, 1:config.m*dim);
    p_f_star_hist = p_star_hist(:, config.m*dim+1:end);

    tracking_error = p_f_hist - p_f_star_hist;
    estimation_error = p_hat_f_hist - p_f_hist;

    %% 2. 3D軌跡プロット（目標軌道と実際の軌道を同時表示）
    figure('Name', 'Agent Trajectories (3D)', 'Position', [100, 100, 1400, 700]);
    hold on; grid on; axis equal; view(30, 20);
    
    % 色定義（各機体ごとに統一）
    colors_target = [0.8 0 0; 0 0.6 0; 0.8 0 0.8];  % 薄い色（目標）
    colors_actual = [1 0 0; 0 0.8 0; 1 0 1];  % 濃い色（実際）
    colors_leader = [0 0 1; 0 0.8 0.8];  % リーダー用の色
    
    % リーダーの目標軌跡をプロット（太い実線）
    plot3(p_l_hist(:,1), p_l_hist(:,2), p_l_hist(:,3), '-', 'Color', colors_leader(1,:), ...
        'LineWidth', 2.5, 'DisplayName', 'Leader 1 (Target)');
    plot3(p_l_hist(:,4), p_l_hist(:,5), p_l_hist(:,6), '-', 'Color', colors_leader(2,:), ...
        'LineWidth', 2.5, 'DisplayName', 'Leader 2 (Target)');
    
    % リーダーの始点・終点マーカー（四角）
    plot3(p_l_hist(1,1), p_l_hist(1,2), p_l_hist(1,3), 's', 'Color', colors_leader(1,:), ...
        'MarkerSize', 10, 'MarkerFaceColor', colors_leader(1,:), 'HandleVisibility', 'off');
    plot3(p_l_hist(end,1), p_l_hist(end,2), p_l_hist(end,3), 's', 'Color', colors_leader(1,:), ...
        'MarkerSize', 10, 'MarkerFaceColor', colors_leader(1,:), 'HandleVisibility', 'off');
    plot3(p_l_hist(1,4), p_l_hist(1,5), p_l_hist(1,6), 's', 'Color', colors_leader(2,:), ...
        'MarkerSize', 10, 'MarkerFaceColor', colors_leader(2,:), 'HandleVisibility', 'off');
    plot3(p_l_hist(end,4), p_l_hist(end,5), p_l_hist(end,6), 's', 'Color', colors_leader(2,:), ...
        'MarkerSize', 10, 'MarkerFaceColor', colors_leader(2,:), 'HandleVisibility', 'off');
    
    % フォロワーの目標軌跡をプロット（破線）
    for i = 1:num_f
        idx_start = config.m*dim + (i-1)*dim + 1;
        plot3(p_f_star_hist(:,idx_start-config.m*dim), ...
              p_f_star_hist(:,idx_start-config.m*dim+1), ...
              p_f_star_hist(:,idx_start-config.m*dim+2), ...
            '--', 'Color', colors_target(i,:), 'LineWidth', 2, ...
            'DisplayName', sprintf('Follower %d (Target)', config.m+i));
        
        % 目標軌道の始点・終点マーカー（四角）
        plot3(p_f_star_hist(1,idx_start-config.m*dim), ...
              p_f_star_hist(1,idx_start-config.m*dim+1), ...
              p_f_star_hist(1,idx_start-config.m*dim+2), ...
            's', 'Color', colors_target(i,:), 'MarkerSize', 10, ...
            'MarkerFaceColor', colors_target(i,:), 'HandleVisibility', 'off');
        plot3(p_f_star_hist(end,idx_start-config.m*dim), ...
              p_f_star_hist(end,idx_start-config.m*dim+1), ...
              p_f_star_hist(end,idx_start-config.m*dim+2), ...
            's', 'Color', colors_target(i,:), 'MarkerSize', 10, ...
            'MarkerFaceColor', colors_target(i,:), 'HandleVisibility', 'off');
    end
    
    % フォロワーの実際の軌跡をプロット（実線）
    for i = 1:num_f
        plot3(p_f_hist(:,(i-1)*dim+1), p_f_hist(:,(i-1)*dim+2), p_f_hist(:,(i-1)*dim+3), ...
            '-', 'Color', colors_actual(i,:), 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Follower %d (Actual)', config.m+i));
        
        % 実際の軌道の始点・終点マーカー（丸）
        plot3(p_f_hist(1,(i-1)*dim+1), p_f_hist(1,(i-1)*dim+2), p_f_hist(1,(i-1)*dim+3), ...
            'o', 'Color', colors_actual(i,:), 'MarkerSize', 10, ...
            'MarkerFaceColor', colors_actual(i,:), 'HandleVisibility', 'off');
        plot3(p_f_hist(end,(i-1)*dim+1), p_f_hist(end,(i-1)*dim+2), p_f_hist(end,(i-1)*dim+3), ...
            'o', 'Color', colors_actual(i,:), 'MarkerSize', 10, ...
            'MarkerFaceColor', colors_actual(i,:), 'HandleVisibility', 'off');
    end
    
    %% 2.5. 開始・終了時点のフォーメーション形状をプロット
    % 全エージェントの実際の位置履歴を一つの行列にまとめる
    % p_all_hist の列の並び: [p1x, p1y, p1z, p2x, ..., p5z]
    p_all_hist = zeros(size(X_history,1), config.n * dim);
    p_all_hist(:, 1:config.m*dim) = p_l_hist; % リーダーの位置
    p_all_hist(:, config.m*dim+1:end) = p_f_hist; % フォロワーの位置
    
    % 開始時点(t=0)と終了時点(t=end)の位置を取得
    pos_start = reshape(p_all_hist(1,:), dim, config.n)';   % 5x3 行列
    pos_end   = reshape(p_all_hist(end,:), dim, config.n)'; % 5x3 行列
    
    % 接続関係（エッジ）のリストを作成
    edges = [];
    for i = 1:num_f
        follower_idx = config.m + i;
        neighbors = config.neighbors{follower_idx};
        edges = [edges; follower_idx, neighbors(1)];
        edges = [edges; follower_idx, neighbors(2)];
    end
    
    % 開始・終了フォーメーションを描画する関数を定義
    draw_formation = @(positions, style, color) ...
        arrayfun(@(i) plot3(positions(edges(i,:),1), positions(edges(i,:),2), positions(edges(i,:),3), style, 'Color', color, 'HandleVisibility', 'off'), 1:size(edges,1));
    
    % 描画を実行（フォーメーション接続線）
    draw_formation(pos_start, '-', [0.5 0.5 0.5]); % 開始時の形状を灰色で描画
    draw_formation(pos_end, '-', [0.3 0.3 0.3]);   % 終了時の形状を濃い灰色で描画
    title('Agent Trajectories'); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    legend;

    %% 3. 追従誤差のプロット（x, y, z軸別）
    figure('Name', 'Tracking Error by Axis');
    axis_names = {'X-axis', 'Y-axis', 'Z-axis'};
    for ax = 1:3
        subplot(3,1,ax);
        % 各軸の誤差を抽出（フォロワー3,4,5のx,y,z成分）
        axis_data = tracking_error(:, ax:3:end);
        plot(t, axis_data, 'LineWidth', 1);
        grid on;
        title(['Tracking Error - ' axis_names{ax}]);
        xlabel('t [s]'); ylabel('Error [m]');
        legend('Follower 3', 'Follower 4', 'Follower 5');
    end

    %% 3.5. 追従誤差のプロット（縦軸固定版：±5）
    figure('Name', 'Tracking Error by Axis (Fixed Scale ±5)');
    for ax = 1:3
        subplot(3,1,ax);
        % 各軸の誤差を抽出（フォロワー3,4,5のx,y,z成分）
        axis_data = tracking_error(:, ax:3:end);
        plot(t, axis_data, 'LineWidth', 1);
        grid on;
        title(['Tracking Error - ' axis_names{ax} ' (Fixed Scale)']);
        xlabel('t [s]'); ylabel('Error [m]');
        ylim([-5, 5]); % 縦軸を±5に固定
        legend('Follower 3', 'Follower 4', 'Follower 5');
    end

    %% 4. 推定誤差のプロット（x, y, z軸別）
    figure('Name', 'Estimation Error by Axis');
    for ax = 1:3
        subplot(3,1,ax);
        % 各軸の誤差を抽出（フォロワー3,4,5のx,y,z成分）
        axis_data = estimation_error(:, ax:3:end);
        plot(t, axis_data, 'LineWidth', 1);
        grid on;
        title(['Estimation Error - ' axis_names{ax}]);
        xlabel('t [s]'); ylabel('Error [m]');
        legend('Follower 3', 'Follower 4', 'Follower 5');
    end
    
    %% 4.5. 各エージェントごとの位置推定誤差（XYZ軸）
    figure('Name', 'Position Estimation Error per Agent', 'Position', [100, 100, 1400, 900]);
    
    colors_est = [0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250];
    
    for i = 1:num_f
        % X軸推定誤差
        subplot(num_f, 3, (i-1)*3 + 1);
        plot(t, estimation_error(:, (i-1)*dim+1), 'LineWidth', 1.5, 'Color', colors_est(i,:));
        grid on;
        xlabel('時刻 [s]', 'FontSize', 10);
        ylabel('推定誤差 [m]', 'FontSize', 10);
        title(sprintf('Follower %d: X軸推定誤差 (\\hat{p}_x - p_x)', config.m+i), 'FontSize', 11, 'FontWeight', 'bold', 'Interpreter', 'tex');
        set(gca, 'FontSize', 9);
        
        % Y軸推定誤差
        subplot(num_f, 3, (i-1)*3 + 2);
        plot(t, estimation_error(:, (i-1)*dim+2), 'LineWidth', 1.5, 'Color', colors_est(i,:));
        grid on;
        xlabel('時刻 [s]', 'FontSize', 10);
        ylabel('推定誤差 [m]', 'FontSize', 10);
        title(sprintf('Follower %d: Y軸推定誤差 (\\hat{p}_y - p_y)', config.m+i), 'FontSize', 11, 'FontWeight', 'bold', 'Interpreter', 'tex');
        set(gca, 'FontSize', 9);
        
        % Z軸推定誤差
        subplot(num_f, 3, (i-1)*3 + 3);
        plot(t, estimation_error(:, (i-1)*dim+3), 'LineWidth', 1.5, 'Color', colors_est(i,:));
        grid on;
        xlabel('時刻 [s]', 'FontSize', 10);
        ylabel('推定誤差 [m]', 'FontSize', 10);
        title(sprintf('Follower %d: Z軸推定誤差 (\\hat{p}_z - p_z)', config.m+i), 'FontSize', 11, 'FontWeight', 'bold', 'Interpreter', 'tex');
        set(gca, 'FontSize', 9);
    end
    
    sgtitle('位置推定誤差（自己位置推知エラー）', 'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'tex');

    %% 5. 推定誤差のプロット（縦軸固定版：±5）
    figure('Name', 'Estimation Error by Axis (Fixed Scale ±5)');
    for ax = 1:3
        subplot(3,1,ax);
        % 各軸の誤差を抽出（フォロワー3,4,5のx,y,z成分）
        axis_data = estimation_error(:, ax:3:end);
        plot(t, axis_data, 'LineWidth', 1);
        grid on;
        title(['Estimation Error - ' axis_names{ax} ' (Fixed Scale)'], 'Interpreter', 'tex');
        xlabel('t [s]'); ylabel('Error [m]');
        ylim([-5, 5]); % 縦軸を±5に固定
        legend('Follower 3', 'Follower 4', 'Follower 5');
    end

    %% 6. デバッグ用グラフ (psi と tau)
    figure('Name', 'Internal Controller Variables (Debug)');
    subplot(2,1,1);
    plot(t, psi_history);
    grid on;
    title('Adjustment Term $\psi$', 'Interpreter', 'tex');
    xlabel('t [s]'); ylabel('$\psi$ value');
    legend('F3-x','F3-y','F3-z','F4-x','F4-y','F4-z','F5-x','F5-y','F5-z');
    
    subplot(2,1,2);
    plot(t, tau_history);
    grid on;
    title('Switching Function $\tau$', 'Interpreter', 'tex');
    xlabel('t [s]'); ylabel('$\tau$ value');
    legend('Follower 3', 'Follower 4', 'Follower 5');

    %% 6.5. λ（調整項）のノルム
    figure('Name', 'Lambda Norm (Adjustment Term)');
    plot(t, weight_debug.lambda_norm, 'LineWidth', 1.5);
    grid on;
    title('$\|\lambda_i\|$ (Velocity Error-based Adjustment Term)', 'Interpreter', 'latex');
    xlabel('t [s]'); 
    ylabel('$\|\lambda_i\|$', 'Interpreter', 'latex');
    legend('Follower 3', 'Follower 4', 'Follower 5', 'Location', 'best');
    ylim([0, config.lambda_max * 1.1]);  % λ_maxの10%上まで表示

    %% 7. フォロワー制御入力の可視化
    figure('Name', 'Follower Control Inputs');
    
    % 制御入力を計算（system_dynamicsから）
    control_inputs = zeros(length(t), config.num_followers * 3);

    for i = 1:length(t)
        % 各時刻での状態を取得（加速度推定システム + PID制御対応）
        X_i = X_history(i, :)';
        p_f_i = reshape(X_i(1:num_f*dim), dim, num_f);                    % 位置
        v_f_i = reshape(X_i(num_f*dim+1:2*num_f*dim), dim, num_f);       % 速度
        p_hat_f_i = reshape(X_i(2*num_f*dim+1:3*num_f*dim), dim, num_f); % 推定位置
        v_hat_f_i = reshape(X_i(3*num_f*dim+1:4*num_f*dim), dim, num_f); % 推定速度
        % e_int_f_i = reshape(X_i(4*num_f*dim+1:end), dim, num_f);       % 積分誤差（必要に応じて使用）
        
        % 目標位置・速度・加速度を取得
        [p_star_all, p_dot_star_all, p_ddot_star_all] = config.get_target_positions(t(i));
        p_f_star_i = cell2mat(p_star_all(config.m+1:end));
        p_dot_f_star_i = cell2mat(p_dot_star_all(config.m+1:end));
        p_ddot_f_star_i = cell2mat(p_ddot_star_all(config.m+1:end));
        
        % 制御入力を計算: 加速度推定システム
        % u_i = ddot_p_hat_i - k_p(p_hat_i - p_i^*) - k_d(v_i - v_hat_i)
        
        % デフォルト制御ゲイン
        if ~isfield(config, 'k_p'), config.k_p = 1.0; end
        if ~isfield(config, 'k_d'), config.k_d = 2.0; end
        if ~isfield(config, 'w_p'), config.w_p = 1.0; end
        if ~isfield(config, 'w_d'), config.w_d = 1.0; end
        if ~isfield(config, 'w_xi'), config.w_xi = 1.0; end
        if ~isfield(config, 'w_psi'), config.w_psi = 0.0; end
        
        % 推定加速度の計算（簡易版：ψ項は省略）
        p_ddot_hat_i = p_ddot_f_star_i - config.w_p * (p_hat_f_i - p_f_star_i) ...
                       - config.w_d * (v_hat_f_i - p_dot_f_star_i);
        
        u_feedforward = p_ddot_hat_i;                                      % 推定加速度
        u_velocity = -config.k_d * (v_f_i - v_hat_f_i);                   % 速度制御項
        
        % 総制御入力（新制御則：u = ddot_hat_p - k_d*(dot_p - dot_hat_p)）
        u_total = u_feedforward + u_velocity;
        control_inputs(i, :) = u_total(:)';
    end
    
    % 各フォロワーの制御入力をプロット
    for i = 1:config.num_followers
        subplot(config.num_followers, 1, i);
        
        % 各軸の制御入力
        u_x = control_inputs(:, (i-1)*3+1);
        u_y = control_inputs(:, (i-1)*3+2);
        u_z = control_inputs(:, (i-1)*3+3);
        
        plot(t, u_x, 'r-', 'LineWidth', 5, 'DisplayName', 'u_x');
        hold on;
        plot(t, u_y, 'g-', 'LineWidth', 5, 'DisplayName', 'u_y');
        plot(t, u_z, 'b-', 'LineWidth', 5, 'DisplayName', 'u_z');
        
        grid on;
        title(sprintf('Follower %d Control Input', config.m+i), 'Interpreter', 'tex');
        xlabel('t [s]'); ylabel('Control Input [m/s]');
        legend('u_x', 'u_y', 'u_z', 'Location', 'best');
        
        % 制御入力の大きさも表示
        u_magnitude = sqrt(u_x.^2 + u_y.^2 + u_z.^2);
        yyaxis right;
        plot(t, u_magnitude, 'k--', 'LineWidth', 1, 'DisplayName', '||u||');
        ylabel('Control Magnitude [m/s]');
    end
    
    %% 8. 制御入力成分の詳細分析
    figure('Name', 'Control Input Components Analysis');
    
    for i = 1:config.num_followers
        subplot(config.num_followers, 3, (i-1)*3+1);
        
        % 追従制御成分
        u_track_data = zeros(length(t), 3);
        u_ff_data = zeros(length(t), 3);
        u_adj_data = zeros(length(t), 3);
        
        for j = 1:length(t)
            X_j = X_history(j, :)';
            % 状態変数の分解（加速度推定システム + PID制御対応）
            p_f_j = reshape(X_j(1:num_f*dim), dim, num_f);                    % 位置
            v_f_j = reshape(X_j(num_f*dim+1:2*num_f*dim), dim, num_f);       % 速度
            p_hat_f_j = reshape(X_j(2*num_f*dim+1:3*num_f*dim), dim, num_f); % 推定位置
            v_hat_f_j = reshape(X_j(3*num_f*dim+1:4*num_f*dim), dim, num_f); % 推定速度
            % e_int_f_j = reshape(X_j(4*num_f*dim+1:end), dim, num_f);       % 積分誤差（必要に応じて使用）
            
            [p_star_all, p_dot_star_all, p_ddot_star_all] = config.get_target_positions(t(j));
            p_f_star_j = cell2mat(p_star_all(config.m+1:end));
            p_dot_f_star_j = cell2mat(p_dot_star_all(config.m+1:end));
            p_ddot_f_star_j = cell2mat(p_ddot_star_all(config.m+1:end));
            
            psi_f_j = reshape(psi_history(j, :), 3, config.num_followers);
            
            % 加速度推定システムの各成分を計算
            % デフォルト制御ゲイン
            if ~isfield(config, 'k_p'), config.k_p = 1.0; end
            if ~isfield(config, 'k_d'), config.k_d = 2.0; end
            if ~isfield(config, 'w_p'), config.w_p = 1.0; end
            if ~isfield(config, 'w_d'), config.w_d = 1.0; end
            
            % 推定加速度（簡易版）
            p_ddot_hat_j = p_ddot_f_star_j - config.w_p * (p_hat_f_j - p_f_star_j) ...
                           - config.w_d * (v_hat_f_j - p_dot_f_star_j);
            
            u_ff_data(j, :) = p_ddot_hat_j(:, i);                                     % 推定加速度
            u_track_data(j, :) = [0, 0, 0];                                           % 位置制御項（新制御則では不使用）
            u_adj_data(j, :) = -config.k_d * (v_f_j(:, i) - v_hat_f_j(:, i));        % 速度制御項
        end
        
        % 位置制御成分（新制御則では不使用）
        plot(t, vecnorm(u_track_data, 2, 2), 'r--', 'LineWidth', 1.5);
        grid on; title(sprintf('F%d: Position Control (Not Used in New Law)', config.m+i), 'Interpreter', 'tex');
        xlabel('t [s]'); ylabel('||u_{P}|| [m/s²]');
        
        subplot(config.num_followers, 3, (i-1)*3+2);
        % フィードフォワード成分（推定加速度）
        plot(t, vecnorm(u_ff_data, 2, 2), 'g-', 'LineWidth', 1.5);
        grid on; title(sprintf('F%d: Feedforward (\\ddot{\\hat{p}})', config.m+i), 'Interpreter', 'tex');
        xlabel('t [s]'); ylabel('||u_{ff}|| [m/s²]');
        
        subplot(config.num_followers, 3, (i-1)*3+3);
        % 速度制御成分 (D制御: -k_d(v - v_hat))
        plot(t, vecnorm(u_adj_data, 2, 2), 'b-', 'LineWidth', 1.5);
        grid on; title(sprintf('F%d: Velocity Control -k_d(v-\\hat{v}), k_d=%.2f', config.m+i, config.k_d), 'Interpreter', 'tex');
        xlabel('t [s]'); ylabel('||u_{D}|| [m/s²]');
    end
    
    %% 9. 制御入力統計情報
    fprintf('\n=== 制御入力統計情報 ===\n');
    fprintf('制御ゲイン: k_p = %.4f, k_d = %.4f\n', config.k_p, config.k_d);
    fprintf('制御則: u = ddot_hat_p - k_p(hat_p - p*) - k_d(v - hat_v)\n\n');
    
    for i = 1:config.num_followers
        u_x = control_inputs(:, (i-1)*3+1);
        u_y = control_inputs(:, (i-1)*3+2);
        u_z = control_inputs(:, (i-1)*3+3);
        u_mag = sqrt(u_x.^2 + u_y.^2 + u_z.^2);
        
        fprintf('フォロワー%d:\n', config.m+i);
        fprintf('  最大制御入力: %.4f [m/s²]\n', max(u_mag));
        fprintf('  平均制御入力: %.4f [m/s²]\n', mean(u_mag));
        fprintf('  制御入力RMS: %.4f [m/s²]\n', rms(u_mag));
        fprintf('  最大x成分: %.4f [m/s²]\n', max(abs(u_x)));
        fprintf('  最大y成分: %.4f [m/s²]\n', max(abs(u_y)));
        fprintf('  最大z成分: %.4f [m/s²]\n\n', max(abs(u_z)));
    end
    
    %% 10. 速度プロット（各エージェントごとのXYZ軸）
    
    figure('Name', 'Velocity Profiles', 'Position', [100, 100, 1400, 900]);
    axis_names = {'X', 'Y', 'Z'};
    v_f_star_hist = v_star_hist(:, config.m*dim+1:end);  % フォロワーの目標速度
    
    % 色定義
    colors_target = [0.8 0 0; 0 0.6 0; 0.8 0 0.8];      % 目標（破線）
    colors_actual = [1 0 0; 0 0.8 0; 1 0 1];            % 実際（実線）
    
    % 各フォロワーの速度をプロット
    for i = 1:num_f
        % X軸速度
        subplot(num_f, 3, (i-1)*3 + 1);
        hold on; grid on;
        plot(t, v_f_star_hist(:, (i-1)*dim+1), '--', 'LineWidth', 2, 'Color', colors_target(i,:), ...
            'DisplayName', 'Target');
        plot(t, v_f_hist(:, (i-1)*dim+1), '-', 'LineWidth', 1.5, 'Color', colors_actual(i,:), ...
            'DisplayName', 'Actual');
        xlabel('時刻 [s]', 'FontSize', 10);
        ylabel('速度 [m/s]', 'FontSize', 10);
        title(sprintf('Follower %d: X軸速度', config.m+i), 'FontSize', 11, 'FontWeight', 'bold');
        legend('Location', 'best');
        set(gca, 'FontSize', 9);
        
        % Y軸速度
        subplot(num_f, 3, (i-1)*3 + 2);
        hold on; grid on;
        plot(t, v_f_star_hist(:, (i-1)*dim+2), '--', 'LineWidth', 2, 'Color', colors_target(i,:), ...
            'DisplayName', 'Target');
        plot(t, v_f_hist(:, (i-1)*dim+2), '-', 'LineWidth', 1.5, 'Color', colors_actual(i,:), ...
            'DisplayName', 'Actual');
        xlabel('時刻 [s]', 'FontSize', 10);
        ylabel('速度 [m/s]', 'FontSize', 10);
        title(sprintf('Follower %d: Y軸速度', config.m+i), 'FontSize', 11, 'FontWeight', 'bold');
        legend('Location', 'best');
        set(gca, 'FontSize', 9);
        
        % Z軸速度
        subplot(num_f, 3, (i-1)*3 + 3);
        hold on; grid on;
        plot(t, v_f_star_hist(:, (i-1)*dim+3), '--', 'LineWidth', 2, 'Color', colors_target(i,:), ...
            'DisplayName', 'Target');
        plot(t, v_f_hist(:, (i-1)*dim+3), '-', 'LineWidth', 1.5, 'Color', colors_actual(i,:), ...
            'DisplayName', 'Actual');
        xlabel('時刻 [s]', 'FontSize', 10);
        ylabel('速度 [m/s]', 'FontSize', 10);
        title(sprintf('Follower %d: Z軸速度', config.m+i), 'FontSize', 11, 'FontWeight', 'bold');
        legend('Location', 'best');
        set(gca, 'FontSize', 9);
    end
    
    sgtitle('目標速度 vs 実際の速度', 'FontSize', 14, 'FontWeight', 'bold');
    
    % 加速度履歴のプロット（目標加速度のみ）
    % figure('Name', 'Target Acceleration Profiles');
    % a_f_star_hist = a_star_hist(:, config.m*dim+1:end);  % フォロワーの目標加速度
    % 
    % for ax = 1:3
    %     subplot(3,1,ax);
    %     target_acc = a_f_star_hist(:, ax:3:end);
    %     plot(t, target_acc, '-', 'LineWidth', 1.5);
    % 
    %     grid on;
    %     title(['Target Acceleration - ' axis_names{ax} ' axis']);
    %     xlabel('t [s]'); ylabel('Acceleration [m/s²]');
    %     legend('Follower 3', 'Follower 4', 'Follower 5', 'Location', 'best');
    % end
    
    % 速度誤差のプロット
    figure('Name', 'Velocity Tracking Error');
    v_error = v_f_hist - v_f_star_hist;
    
    for ax = 1:3
        subplot(3,1,ax);
        axis_data = v_error(:, ax:3:end);
        plot(t, axis_data, 'LineWidth', 1);
        grid on;
        title(['Velocity Error - ' axis_names{ax}]);
        xlabel('t [s]'); ylabel('Velocity Error [m/s]');
        legend('Follower 3', 'Follower 4', 'Follower 5');
    end

    %% 11. 重み行列の推移可視化
    if ~isempty(weight_debug)
        plot_weight_matrix_evolution(t, weight_debug, config);
    end
    
    %% 11.5. ξ項の可視化
    figure('Name', 'Xi (ξ) Term Visualization', 'Position', [100, 100, 1400, 900]);
    
    % xi_historyを[時刻 x (3*num_f)]から[3 x num_f x 時刻]に変換
    xi_reshaped = reshape(xi_history', 3, num_f, []);
    
    colors_xi = [0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250];
    
    for i = 1:num_f
        % X軸ξ成分
        subplot(num_f, 3, (i-1)*3 + 1);
        plot(t, squeeze(xi_reshaped(1, i, :)), 'LineWidth', 1.5, 'Color', colors_xi(i,:));
        grid on;
        xlabel('時刻 [s]', 'FontSize', 10);
        ylabel('\xi_x', 'FontSize', 10);
        title(sprintf('Follower %d: \\xi_{x} (位置制約勾配X成分)', config.m+i), 'FontSize', 11, 'FontWeight', 'bold');
        set(gca, 'FontSize', 9);
        
        % Y軸ξ成分
        subplot(num_f, 3, (i-1)*3 + 2);
        plot(t, squeeze(xi_reshaped(2, i, :)), 'LineWidth', 1.5, 'Color', colors_xi(i,:));
        grid on;
        xlabel('時刻 [s]', 'FontSize', 10);
        ylabel('\xi_y', 'FontSize', 10);
        title(sprintf('Follower %d: \\xi_{y} (位置制約勾配Y成分)', config.m+i), 'FontSize', 11, 'FontWeight', 'bold');
        set(gca, 'FontSize', 9);
        
        % Z軸ξ成分
        subplot(num_f, 3, (i-1)*3 + 3);
        plot(t, squeeze(xi_reshaped(3, i, :)), 'LineWidth', 1.5, 'Color', colors_xi(i,:));
        grid on;
        xlabel('時刻 [s]', 'FontSize', 10);
        ylabel('\xi_z', 'FontSize', 10);
        title(sprintf('Follower %d: \\xi_{z} (位置制約勾配Z成分)', config.m+i), 'FontSize', 11, 'FontWeight', 'bold');
        set(gca, 'FontSize', 9);
    end
    
    sgtitle('\xi項の時間変化（位置制約の勾配）', 'FontSize', 14, 'FontWeight', 'bold');
    
    %% 11.6. ξのノルムプロット
    figure('Name', 'Xi Norm Over Time', 'Position', [100, 100, 1200, 400]);
    
    hold on; grid on;
    for i = 1:num_f
        xi_norm = vecnorm(squeeze(xi_reshaped(:, i, :)), 2, 1);
        plot(t, xi_norm, 'LineWidth', 2, 'DisplayName', sprintf('Follower %d', config.m+i));
    end
    xlabel('時刻 [s]', 'FontSize', 12);
    ylabel('||\xi||', 'FontSize', 12);
    title('\xi項のノルムの時間変化', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 10);
    set(gca, 'FontSize', 11);
    
    fprintf('\n=== ξ統計情報 ===\n');
    for i = 1:num_f
        xi_norm = vecnorm(squeeze(xi_reshaped(:, i, :)), 2, 1);
        fprintf('Follower %d:\n', config.m+i);
        fprintf('  初期||ξ||: %.6f\n', xi_norm(1));
        fprintf('  最終||ξ||: %.6f\n', xi_norm(end));
        fprintf('  最大||ξ||: %.6f\n', max(xi_norm));
        fprintf('  平均||ξ||: %.6f\n\n', mean(xi_norm));
    end
    
    %% 11.7. 積分誤差の可視化（PID制御のI項）
    if isfield(config, 'w_i') && config.w_i > 0
        figure('Name', 'Integral Error (PID I-term)', 'Position', [100, 100, 1400, 900]);
        
        % e_int_historyを[時刻 x (3*num_f)]から[3 x num_f x 時刻]に変換
        e_int_reshaped = reshape(e_int_f_hist', 3, num_f, []);
        
        colors_int = [0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250];
        
        for i = 1:num_f
            % X軸積分誤差
            subplot(num_f, 3, (i-1)*3 + 1);
            plot(t, squeeze(e_int_reshaped(1, i, :)), 'LineWidth', 1.5, 'Color', colors_int(i,:));
            grid on;
            xlabel('時刻 [s]', 'FontSize', 10);
            ylabel('e_{int,x} [m·s]', 'FontSize', 10);
            title(sprintf('Follower %d: 積分誤差X成分 (PID I項)', config.m+i), 'FontSize', 11, 'FontWeight', 'bold');
            set(gca, 'FontSize', 9);
            
            % Y軸積分誤差
            subplot(num_f, 3, (i-1)*3 + 2);
            plot(t, squeeze(e_int_reshaped(2, i, :)), 'LineWidth', 1.5, 'Color', colors_int(i,:));
            grid on;
            xlabel('時刻 [s]', 'FontSize', 10);
            ylabel('e_{int,y} [m·s]', 'FontSize', 10);
            title(sprintf('Follower %d: 積分誤差Y成分 (PID I項)', config.m+i), 'FontSize', 11, 'FontWeight', 'bold');
            set(gca, 'FontSize', 9);
            
            % Z軸積分誤差
            subplot(num_f, 3, (i-1)*3 + 3);
            plot(t, squeeze(e_int_reshaped(3, i, :)), 'LineWidth', 1.5, 'Color', colors_int(i,:));
            grid on;
            xlabel('時刻 [s]', 'FontSize', 10);
            ylabel('e_{int,z} [m·s]', 'FontSize', 10);
            title(sprintf('Follower %d: 積分誤差Z成分 (PID I項)', config.m+i), 'FontSize', 11, 'FontWeight', 'bold');
            set(gca, 'FontSize', 9);
        end
        
        sgtitle(sprintf('PID制御の積分誤差 (w_i = %.2f)', config.w_i), 'FontSize', 14, 'FontWeight', 'bold');
        
        % 積分誤差のノルムプロット
        figure('Name', 'Integral Error Norm', 'Position', [100, 100, 1200, 400]);
        
        hold on; grid on;
        for i = 1:num_f
            e_int_norm = vecnorm(squeeze(e_int_reshaped(:, i, :)), 2, 1);
            plot(t, e_int_norm, 'LineWidth', 2, 'DisplayName', sprintf('Follower %d', config.m+i));
        end
        xlabel('時刻 [s]', 'FontSize', 12);
        ylabel('||e_{int}|| [m·s]', 'FontSize', 12);
        title('積分誤差のノルムの時間変化 (PID I項)', 'FontSize', 14, 'FontWeight', 'bold');
        legend('Location', 'best', 'FontSize', 10);
        set(gca, 'FontSize', 11);
        
        fprintf('\n=== 積分誤差統計情報 (PID I項) ===\n');
        fprintf('I項ゲイン: w_i = %.4f\n', config.w_i);
        for i = 1:num_f
            e_int_norm = vecnorm(squeeze(e_int_reshaped(:, i, :)), 2, 1);
            fprintf('Follower %d:\n', config.m+i);
            fprintf('  初期||e_int||: %.6f\n', e_int_norm(1));
            fprintf('  最終||e_int||: %.6f\n', e_int_norm(end));
            fprintf('  最大||e_int||: %.6f\n', max(e_int_norm));
            fprintf('  平均||e_int||: %.6f\n\n', mean(e_int_norm));
        end
    end
    
    %% 12. 加速度プロット（目標加速度、推定加速度、実際の入力加速度）
    figure('Name', 'Acceleration Comparison', 'Position', [100, 100, 1400, 900]);
    
    % 推定加速度を数値微分で計算（推定速度の微分）
    dt = t(2) - t(1);
    a_hat_f = zeros(length(t), num_f*dim);
    for i = 1:length(t)
        if i == 1
            % 前方差分
            a_hat_f(i, :) = (v_hat_f_hist(i+1, :) - v_hat_f_hist(i, :)) / dt;
        elseif i == length(t)
            % 後方差分
            a_hat_f(i, :) = (v_hat_f_hist(i, :) - v_hat_f_hist(i-1, :)) / dt;
        else
            % 中心差分
            a_hat_f(i, :) = (v_hat_f_hist(i+1, :) - v_hat_f_hist(i-1, :)) / (2*dt);
        end
    end
    
    % 実際の入力加速度を数値微分で計算（実際の速度の微分）
    a_f_actual = zeros(length(t), num_f*dim);
    for i = 1:length(t)
        if i == 1
            a_f_actual(i, :) = (v_f_hist(i+1, :) - v_f_hist(i, :)) / dt;
        elseif i == length(t)
            a_f_actual(i, :) = (v_f_hist(i, :) - v_f_hist(i-1, :)) / dt;
        else
            a_f_actual(i, :) = (v_f_hist(i+1, :) - v_f_hist(i-1, :)) / (2*dt);
        end
    end
    
    % 目標加速度（フォロワーのみ）
    a_f_star_hist = a_star_hist(:, config.m*dim+1:end);
    
    % 色定義
    colors_target = [0.8 0 0; 0 0.6 0; 0.8 0 0.8];      % 目標（破線）
    colors_estimated = [0 0.5 1; 0 0.7 0.7; 0.7 0 0.7]; % 推定（実線）
    colors_actual = [1 0 0; 0 0.8 0; 1 0 1];            % 実際（点線）
    
    % 各フォロワーの加速度をプロット
    for i = 1:num_f
        % X軸加速度
        subplot(num_f, 3, (i-1)*3 + 1);
        hold on; grid on;
        plot(t, a_f_star_hist(:, (i-1)*dim+1), '--', 'LineWidth', 2, 'Color', colors_target(i,:), ...
            'DisplayName', 'Target');
        plot(t, a_hat_f(:, (i-1)*dim+1), '-', 'LineWidth', 1.5, 'Color', colors_estimated(i,:), ...
            'DisplayName', 'Estimated');
        plot(t, a_f_actual(:, (i-1)*dim+1), ':', 'LineWidth', 1.5, 'Color', colors_actual(i,:), ...
            'DisplayName', 'Actual Input');
        xlabel('時刻 [s]', 'FontSize', 10);
        ylabel('加速度 [m/s^2]', 'FontSize', 10);
        title(sprintf('Follower %d: X軸加速度', config.m+i), 'FontSize', 11, 'FontWeight', 'bold');
        legend('Location', 'best');
        set(gca, 'FontSize', 9);
        
        % Y軸加速度
        subplot(num_f, 3, (i-1)*3 + 2);
        hold on; grid on;
        plot(t, a_f_star_hist(:, (i-1)*dim+2), '--', 'LineWidth', 2, 'Color', colors_target(i,:), ...
            'DisplayName', 'Target');
        plot(t, a_hat_f(:, (i-1)*dim+2), '-', 'LineWidth', 1.5, 'Color', colors_estimated(i,:), ...
            'DisplayName', 'Estimated');
        plot(t, a_f_actual(:, (i-1)*dim+2), ':', 'LineWidth', 1.5, 'Color', colors_actual(i,:), ...
            'DisplayName', 'Actual Input');
        xlabel('時刻 [s]', 'FontSize', 10);
        ylabel('加速度 [m/s^2]', 'FontSize', 10);
        title(sprintf('Follower %d: Y軸加速度', config.m+i), 'FontSize', 11, 'FontWeight', 'bold');
        legend('Location', 'best');
        set(gca, 'FontSize', 9);
        
        % Z軸加速度
        subplot(num_f, 3, (i-1)*3 + 3);
        hold on; grid on;
        plot(t, a_f_star_hist(:, (i-1)*dim+3), '--', 'LineWidth', 2, 'Color', colors_target(i,:), ...
            'DisplayName', 'Target');
        plot(t, a_hat_f(:, (i-1)*dim+3), '-', 'LineWidth', 1.5, 'Color', colors_estimated(i,:), ...
            'DisplayName', 'Estimated');
        plot(t, a_f_actual(:, (i-1)*dim+3), ':', 'LineWidth', 1.5, 'Color', colors_actual(i,:), ...
            'DisplayName', 'Actual Input');
        xlabel('時刻 [s]', 'FontSize', 10);
        ylabel('加速度 [m/s^2]', 'FontSize', 10);
        title(sprintf('Follower %d: Z軸加速度', config.m+i), 'FontSize', 11, 'FontWeight', 'bold');
        legend('Location', 'best');
        set(gca, 'FontSize', 9);
    end
    
    sgtitle('目標加速度 vs 推定加速度 vs 実際の入力加速度', 'FontSize', 14, 'FontWeight', 'bold');
    
%     %% 13. 加速度ノルムの比較
%     figure('Name', 'Acceleration Norm Comparison', 'Position', [150, 150, 1200, 400]);
% 
%     for i = 1:num_f
%         % 加速度ノルムを計算
%         a_star_norm = sqrt(a_f_star_hist(:, (i-1)*dim+1).^2 + ...
%                           a_f_star_hist(:, (i-1)*dim+2).^2 + ...
%                           a_f_star_hist(:, (i-1)*dim+3).^2);
%         a_actual_norm = sqrt(a_f_actual(:, (i-1)*dim+1).^2 + ...
%                             a_f_actual(:, (i-1)*dim+2).^2 + ...
%                             a_f_actual(:, (i-1)*dim+3).^2);
% 
%         subplot(1, num_f, i);
%         hold on; grid on;
%         plot(t, a_star_norm, '--', 'LineWidth', 2.5, 'Color', colors_target(i,:), ...
%             'DisplayName', 'Target');
%         plot(t, a_actual_norm, '-', 'LineWidth', 1.5, 'Color', colors_actual(i,:), ...
%             'DisplayName', 'Actual');
%         xlabel('時刻 [s]', 'FontSize', 11);
%         ylabel('加速度ノルム [m/s^2]', 'FontSize', 11);
%         title(sprintf('Follower %d', config.m+i), 'FontSize', 12, 'FontWeight', 'bold');
%         legend('Location', 'best');
%         set(gca, 'FontSize', 10);
%     end
% 
%     sgtitle('加速度ノルムの比較', 'FontSize', 14, 'FontWeight', 'bold');

    %% 13. 3D軌跡アニメーション
    fprintf('\n3D軌跡アニメーションを生成しますか？ (y/n): ');
    user_input = input('', 's');
    
    if strcmpi(user_input, 'y') || strcmpi(user_input, 'yes')
        animate_3d_trajectory(t, X_history, config);
    else
        fprintf('アニメーションをスキップしました。\n');
        fprintf('手動で実行する場合: animate_3d_trajectory(t, X_history, config)\n');
    end
end

function plot_weight_matrix_evolution(t, weight_debug, config)
    % 重み行列の推移を可視化する関数
    
    % 変数の定義
    num_f = config.num_followers;
    dim = 3;
    
    %% 重み行列の行列式の推移
    figure('Name', 'Weight Matrix Determinants Evolution');
    
    for i = 1:config.num_followers
        subplot(config.num_followers, 2, (i-1)*2+1);
        plot(t, weight_debug.Hij_det(:, i), 'r-', 'LineWidth', 5);
        grid on;
        title(sprintf('Follower %d: det(Hij)', config.m+i));
        xlabel('t [s]'); ylabel('det(Hij)');
        
        subplot(config.num_followers, 2, (i-1)*2+2);
        plot(t, weight_debug.Hik_det(:, i), 'b-', 'LineWidth', 5);
        grid on;
        title(sprintf('Follower %d: det(Hik)', config.m+i));
        xlabel('t [s]'); ylabel('det(Hik)');
    end
    
    %% 重み行列のトレースの推移
    % figure('Name', 'Weight Matrix Traces Evolution');
    
    % for i = 1:config.num_followers
    %     subplot(config.num_followers, 2, (i-1)*2+1);
    %     plot(t, weight_debug.Hij_trace(:, i), 'r-', 'LineWidth', 5);
    %     grid on;
    %     title(sprintf('Follower %d: trace(Hij)', config.m+i));
    %     xlabel('t [s]'); ylabel('trace(Hij)');
    % 
    %     subplot(config.num_followers, 2, (i-1)*2+2);
    %     plot(t, weight_debug.Hik_trace(:, i), 'b-', 'LineWidth', 5);
    %     grid on;
    %     title(sprintf('Follower %d: trace(Hik)', config.m+i));
    %     xlabel('t [s]'); ylabel('trace(Hik)');
    % end
    
    %% オクルージョン状態の可視化
    figure('Name', 'Occlusion Status');
    
    for i = 1:config.num_followers
        subplot(config.num_followers, 1, i);
        
        % オクルージョン状態をプロット
        occlusion_j_plot = double(weight_debug.occlusion_j(:, i));
        occlusion_k_plot = double(weight_debug.occlusion_k(:, i));
        collinear_plot = double(weight_debug.is_collinear(:, i));
        
        % オクルージョンjを赤で表示
        area(t, occlusion_j_plot * 0.8, 'FaceColor', 'r', 'FaceAlpha', 0.3, 'DisplayName', 'Occlusion j');
        hold on;
        % オクルージョンkを青で表示
        area(t, occlusion_k_plot * 0.6, 'FaceColor', 'b', 'FaceAlpha', 0.3, 'DisplayName', 'Occlusion k');
        % 共線状態を黄色で表示
        area(t, collinear_plot * 0.4, 'FaceColor', 'y', 'FaceAlpha', 0.3, 'DisplayName', 'Collinear');
        
        grid on;
        title(sprintf('Follower %d: Occlusion and Collinear Status', config.m+i));
        xlabel('t [s]'); ylabel('Status');
        ylim([0, 1]);
        legend('Location', 'best');
    end
    
    %% 重み行列の正規化ノルムの推移
    % figure('Name', 'Weight Matrix Frobenius Norms');
    % 
    % for i = 1:config.num_followers
    %     subplot(config.num_followers, 1, i);
    % 
    %     % フロベニウスノルムを計算（行列式とトレースから近似）
    %     Hij_norm = sqrt(abs(weight_debug.Hij_det(:, i)) + weight_debug.Hij_trace(:, i).^2);
    %     Hik_norm = sqrt(abs(weight_debug.Hik_det(:, i)) + weight_debug.Hik_trace(:, i).^2);
    % 
    %     plot(t, Hij_norm, 'r-', 'LineWidth', 5, 'DisplayName', '||Hij||_F (approx)');
    %     hold on;
    %     plot(t, Hik_norm, 'b-', 'LineWidth', 5, 'DisplayName', '||Hik||_F (approx)');
    % 
    %     % オクルージョン期間を背景色で表示
    %     occlusion_j_indices = find(weight_debug.occlusion_j(:, i));
    %     occlusion_k_indices = find(weight_debug.occlusion_k(:, i));
    % 
    %     if ~isempty(occlusion_j_indices)
    %         for idx = occlusion_j_indices'
    %             xline(t(idx), 'r--', 'Alpha', 0.3, 'HandleVisibility', 'off');
    %         end
    %     end
    % 
    %     if ~isempty(occlusion_k_indices)
    %         for idx = occlusion_k_indices'
    %             xline(t(idx), 'b--', 'Alpha', 0.3, 'HandleVisibility', 'off');
    %         end
    %     end
    % 
    %     grid on;
    %     title(sprintf('Follower %d: Weight Matrix Norms', config.m+i));
    %     xlabel('t [s]'); ylabel('Matrix Norm');
end