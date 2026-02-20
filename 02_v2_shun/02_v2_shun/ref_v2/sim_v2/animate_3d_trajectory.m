function animate_3d_trajectory(t, X_history, config)
    % -------------------------------------------------------------------------
    % [機能] 3D軌跡のアニメーション表示
    % [引数]
    %   t: 時間ベクトル
    %   X_history: 状態履歴 [位置; 速度; 推定位置; 推定速度]
    %   config: システム設定
    % -------------------------------------------------------------------------
    
    fprintf('\n=== 3D軌跡アニメーションを生成中 ===\n');
    
    num_f = config.num_followers;
    dim = 3;
    
    % 状態ベクトルの分解: [位置; 速度; 推定位置; 推定速度; 積分誤差]
    p_f_hist = X_history(:, 1:num_f*dim);
    p_hat_f_hist = X_history(:, 2*num_f*dim+1:3*num_f*dim);
    
    % リーダーの軌跡を計算
    num_steps = length(t);
    p_l_hist = zeros(num_steps, config.m * dim);
    p_f_star_hist = zeros(num_steps, num_f * dim);
    
    for i = 1:num_steps
        [p_star_all, ~, ~] = config.get_target_positions(t(i));
        p_l_hist(i, :) = reshape(cell2mat(p_star_all(1:config.m)), 1, []);
        p_f_star_hist(i, :) = reshape(cell2mat(p_star_all(config.m+1:end)), 1, []);
    end
    
    %% アニメーションフィギュアの作成
    fig = figure('Name', '3D Trajectory Animation', 'Position', [100, 100, 1200, 800]);
    
    % カラーマップ
    colors_leader = [0.5 0 0.5; 0.7 0 0.7];  % 紫系（リーダー）
    colors_follower = [0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250];
    
    % 全軌跡を薄く表示（背景）
    hold on; grid on; axis equal;
    xlabel('X [m]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontSize', 12, 'FontWeight', 'bold');
    title('3D Trajectory Animation', 'FontSize', 14, 'FontWeight', 'bold');
    view(45, 30);
    
    % リーダーの軌跡（薄く表示）
    for i = 1:config.m
        px = p_l_hist(:, (i-1)*dim+1);
        py = p_l_hist(:, (i-1)*dim+2);
        pz = p_l_hist(:, (i-1)*dim+3);
        plot3(px, py, pz, '--', 'Color', [colors_leader(i,:), 0.3], 'LineWidth', 1);
    end
    
    % フォロワーの目標軌跡（薄く表示）
    for i = 1:num_f
        px_star = p_f_star_hist(:, (i-1)*dim+1);
        py_star = p_f_star_hist(:, (i-1)*dim+2);
        pz_star = p_f_star_hist(:, (i-1)*dim+3);
        plot3(px_star, py_star, pz_star, ':', 'Color', [colors_follower(i,:), 0.3], 'LineWidth', 1);
    end
    
    % 実際の軌跡（薄く表示）
    for i = 1:num_f
        px = p_f_hist(:, (i-1)*dim+1);
        py = p_f_hist(:, (i-1)*dim+2);
        pz = p_f_hist(:, (i-1)*dim+3);
        plot3(px, py, pz, '-', 'Color', [colors_follower(i,:), 0.3], 'LineWidth', 1);
    end
    
    % 動的要素の初期化
    leader_markers = gobjects(config.m, 1);
    follower_markers = gobjects(num_f, 1);
    follower_target_markers = gobjects(num_f, 1);
    follower_estimate_markers = gobjects(num_f, 1);
    follower_trails = gobjects(num_f, 1);
    
    % リーダーのマーカー
    for i = 1:config.m
        px = p_l_hist(1, (i-1)*dim+1);
        py = p_l_hist(1, (i-1)*dim+2);
        pz = p_l_hist(1, (i-1)*dim+3);
        leader_markers(i) = plot3(px, py, pz, 'p', 'MarkerSize', 20, ...
            'MarkerFaceColor', colors_leader(i,:), 'MarkerEdgeColor', 'k', 'LineWidth', 2);
    end
    
    % フォロワーの初期マーカー
    trail_length = 50;  % 軌跡の長さ（ステップ数）
    
    for i = 1:num_f
        % 実際の位置
        px = p_f_hist(1, (i-1)*dim+1);
        py = p_f_hist(1, (i-1)*dim+2);
        pz = p_f_hist(1, (i-1)*dim+3);
        follower_markers(i) = plot3(px, py, pz, 'o', 'MarkerSize', 15, ...
            'MarkerFaceColor', colors_follower(i,:), 'MarkerEdgeColor', 'k', 'LineWidth', 2);
        
        % 目標位置
        px_star = p_f_star_hist(1, (i-1)*dim+1);
        py_star = p_f_star_hist(1, (i-1)*dim+2);
        pz_star = p_f_star_hist(1, (i-1)*dim+3);
        follower_target_markers(i) = plot3(px_star, py_star, pz_star, 's', 'MarkerSize', 12, ...
            'MarkerFaceColor', 'none', 'MarkerEdgeColor', colors_follower(i,:), 'LineWidth', 2);
        
        % 推定位置
        px_hat = p_hat_f_hist(1, (i-1)*dim+1);
        py_hat = p_hat_f_hist(1, (i-1)*dim+2);
        pz_hat = p_hat_f_hist(1, (i-1)*dim+3);
        follower_estimate_markers(i) = plot3(px_hat, py_hat, pz_hat, 'x', 'MarkerSize', 12, ...
            'Color', colors_follower(i,:), 'LineWidth', 2);
        
        % 軌跡（初期化）
        follower_trails(i) = plot3(px, py, pz, '-', 'Color', colors_follower(i,:), 'LineWidth', 2);
    end
    
    % 時刻表示
    time_text = text(0.02, 0.98, '', 'Units', 'normalized', 'FontSize', 14, ...
        'FontWeight', 'bold', 'VerticalAlignment', 'top', 'BackgroundColor', [1 1 1 0.8]);
    
    % 凡例
    legend_entries = cell(config.m + num_f, 1);
    for i = 1:config.m
        legend_entries{i} = sprintf('Leader %d', i);
    end
    for i = 1:num_f
        legend_entries{config.m + i} = sprintf('Follower %d', config.m + i);
    end
    legend([leader_markers; follower_markers], legend_entries, 'Location', 'best', 'FontSize', 10);
    
    % アニメーション速度の調整（実時間の何倍速か）
    animation_speed = 2.0;  % 2倍速
    skip_frames = 5;  % フレームをスキップして高速化
    
    fprintf('アニメーション実行中（終了するには図を閉じてください）...\n');
    fprintf('  総フレーム数: %d\n', num_steps);
    fprintf('  表示フレーム数: %d (skip=%d)\n', ceil(num_steps/skip_frames), skip_frames);
    fprintf('  再生速度: %.1f倍速\n', animation_speed);
    
    %% アニメーションループ
    for k = 1:skip_frames:num_steps
        if ~ishandle(fig)
            fprintf('アニメーション中断（図が閉じられました）\n');
            break;
        end
        
        % リーダーの位置更新
        for i = 1:config.m
            px = p_l_hist(k, (i-1)*dim+1);
            py = p_l_hist(k, (i-1)*dim+2);
            pz = p_l_hist(k, (i-1)*dim+3);
            set(leader_markers(i), 'XData', px, 'YData', py, 'ZData', pz);
        end
        
        % フォロワーの位置更新
        for i = 1:num_f
            % 実際の位置
            px = p_f_hist(k, (i-1)*dim+1);
            py = p_f_hist(k, (i-1)*dim+2);
            pz = p_f_hist(k, (i-1)*dim+3);
            set(follower_markers(i), 'XData', px, 'YData', py, 'ZData', pz);
            
            % 目標位置
            px_star = p_f_star_hist(k, (i-1)*dim+1);
            py_star = p_f_star_hist(k, (i-1)*dim+2);
            pz_star = p_f_star_hist(k, (i-1)*dim+3);
            set(follower_target_markers(i), 'XData', px_star, 'YData', py_star, 'ZData', pz_star);
            
            % 推定位置
            px_hat = p_hat_f_hist(k, (i-1)*dim+1);
            py_hat = p_hat_f_hist(k, (i-1)*dim+2);
            pz_hat = p_hat_f_hist(k, (i-1)*dim+3);
            set(follower_estimate_markers(i), 'XData', px_hat, 'YData', py_hat, 'ZData', pz_hat);
            
            % 軌跡更新（最近のtrail_lengthステップのみ表示）
            start_idx = max(1, k - trail_length);
            px_trail = p_f_hist(start_idx:k, (i-1)*dim+1);
            py_trail = p_f_hist(start_idx:k, (i-1)*dim+2);
            pz_trail = p_f_hist(start_idx:k, (i-1)*dim+3);
            set(follower_trails(i), 'XData', px_trail, 'YData', py_trail, 'ZData', pz_trail);
        end
        
        % 時刻表示更新
        set(time_text, 'String', sprintf('t = %.2f s', t(k)));
        
        % 描画更新
        drawnow;
        
        % 待機（実時間に近づける）
        if k < num_steps
            dt = (t(min(k+skip_frames, num_steps)) - t(k)) / animation_speed;
            pause(dt);
        end
    end
    
    fprintf('アニメーション完了！\n\n');
end
