function [xi_val, psi_f, tau_vals, debug_info] = calculate_control_logic(t, p_f, v_f, p_hat_f, v_hat_f, p_l, config)
    % =========================================================================
    % [機能] v4_observer.tex理論に準拠した制御ロジック計算
    % [入力]
    %   t: 現在時刻
    %   p_f: 実位置 [3*num_f x 1]
    %   v_f: 実速度 [3*num_f x 1]
    %   p_hat_f: 推定位置 [3*num_f x 1]
    %   v_hat_f: 推定速度 [3*num_f x 1]
    %   p_l: リーダー位置 [3*m x 1]
    %   config: 設定構造体
    % [出力]
    %   xi_val: ξ項 [3 x num_f]（式ref{eq:est_p}）
    %   psi_f: ψ項 [3 x num_f]（式ref{eq:controller}）
    %   tau_vals: τ値 [1 x num_f]（式ref{eq:tau}）
    % =========================================================================
    
    dim = 3;
    num_f = config.num_followers;
    
    % デバッグ情報を格納する構造体
    debug_info = struct();
    debug_info.case_ids = cell(1, num_f);
    debug_info.Hij_det = zeros(1, num_f);
    debug_info.Hik_det = zeros(1, num_f);
    debug_info.Hij_trace = zeros(1, num_f);
    debug_info.Hik_trace = zeros(1, num_f);
    debug_info.is_collinear = false(1, num_f);
    debug_info.occlusion_j = false(1, num_f);
    debug_info.occlusion_k = false(1, num_f);
    debug_info.lambda_norm = zeros(1, num_f);  % λのノルムを記録
    
    B_fl = zeros(num_f*dim, config.m*dim);
    B_ff = zeros(num_f*dim, num_f*dim);
    psi_f = zeros(dim, num_f);
    tau_vals = zeros(1, num_f);

    % 推定位置配列（制御入力の計算に使用）
    p_hat_all = [num2cell(reshape(p_l, dim, []), 1), num2cell(reshape(p_hat_f, dim, []), 1)];
    
    % 真の位置配列（センサ計測のシミュレーション用）
    % Bearing onlyでも、センサが計測する「真のbearing」からH行列を計算可能
    % シミュレーションでは真の位置からHを計算するが、これは実機で
    % 「計測されたbearingからHを計算する」ことに対応する
    p_true_all = [num2cell(reshape(p_l, dim, []), 1), num2cell(reshape(p_f, dim, []), 1)];
    
    % 実速度（λ計算用）
    v_f_mat = reshape(v_f, dim, num_f);
    v_hat_f_mat = reshape(v_hat_f, dim, num_f);
    
    % 目標軌道の取得
    [p_star_all, v_star_all, ~] = config.get_target_positions(t);

    for i = 1:num_f
        follower_idx_global = config.m + i;
        j_idx = config.neighbors{follower_idx_global}(1);
        k_idx = config.neighbors{follower_idx_global}(2);

        % 推定位置（制御ロジックで使用）
        p_hat_i = p_hat_all{follower_idx_global};
        p_hat_j = p_hat_all{j_idx};
        p_hat_k = p_hat_all{k_idx};
        
        % 真の位置（センサ情報から導かれる重み行列の計算用）
        p_true_i = p_true_all{follower_idx_global};
        p_true_j = p_true_all{j_idx};
        p_true_k = p_true_all{k_idx};
        
        % オクルージョン状態をチェック
        occlusion_j = config.check_occlusion(t, follower_idx_global, 'j');
        occlusion_k = config.check_occlusion(t, follower_idx_global, 'k');
        
        % 重み行列の計算（真の相対関係＝センサ情報を使用）
        % これは実機で「計測されたbearingからHを計算する」ことに対応
        % オクルージョン時：bearing計測不可 → H = 0
        [Hij, Hik, case_id] = calculate_weight_matrices(p_true_i, p_true_j, p_true_k, occlusion_j, occlusion_k);
        
        % デバッグ情報を記録
        debug_info.case_ids{i} = case_id;
        debug_info.Hij_det(i) = det(Hij);
        debug_info.Hik_det(i) = det(Hik);
        debug_info.Hij_trace(i) = trace(Hij);
        debug_info.Hik_trace(i) = trace(Hik);
        debug_info.is_collinear(i) = strcmp(case_id, 'collinear');
        debug_info.occlusion_j(i) = occlusion_j;
        debug_info.occlusion_k(i) = occlusion_k;
        
        Hii = Hij + Hik;

        row_idx = (i-1)*dim+1 : i*dim;
        B_ff(row_idx, row_idx) = Hii;
        
        if j_idx > config.m
            B_ff(row_idx, (j_idx-config.m-1)*dim+1:(j_idx-config.m)*dim) = -Hij;
        else
            B_fl(row_idx, (j_idx-1)*dim+1:j_idx*dim) = -Hij;
        end
        
        if k_idx > config.m
            B_ff(row_idx, (k_idx-config.m-1)*dim+1:(k_idx-config.m)*dim) = -Hik;
        else
            B_fl(row_idx, (k_idx-1)*dim+1:k_idx*dim) = -Hik;
        end

        p_i_star = p_star_all{follower_idx_global};
        
        % ベアリング計算（推定位置を使用）
        v_ij_star = (p_star_all{j_idx} - p_i_star); 
        g_ij_star = v_ij_star / (norm(v_ij_star) + 1e-9);
        
        v_ik_star = (p_star_all{k_idx} - p_i_star); 
        g_ik_star = v_ik_star / (norm(v_ik_star) + 1e-9);
        
        v_ij_hat = (p_hat_j - p_hat_i); 
        g_ij_hat = v_ij_hat / (norm(v_ij_hat) + 1e-9);
        
        v_ik_hat = (p_hat_k - p_hat_i); 
        g_ik_hat = v_ik_hat / (norm(v_ik_hat) + 1e-9);
        
        % τの計算（式ref{eq:tau}）
        % ベアリング誤差を計算（推定位置ベース）
        bearing_error = norm(g_ij_hat - g_ij_star) + norm(g_ik_hat - g_ik_star);

        % 局所化不可能（unlocalizable）状態の判定
        % 理論：相対測定が失われる状態 → τ/ψによる補償が必要
        % 
        % (1) 共線状態（collinear）: 幾何学的に位置が一意に定まらない
        % (2) オクルージョン: センサ遮蔽により相対測定が失われる
        % 
        % 両者とも「bearing情報の欠如」という点で数学的に等価
        
        % 共線状態の判定（case_idベース）
        is_collinear = strcmp(case_id, 'collinear') || ...
                       strcmp(case_id, 'collocation') || ...
                       strcmp(case_id, 'collocation_jk') || ...
                       strcmp(case_id, 'near_singular_alpha') || ...
                       strcmp(case_id, 'near_singular_beta') || ...
                       strcmp(case_id, 'near_singular_gamma') || ...
                       strcmp(case_id, 'degenerate') || ...
                       strcmp(case_id, 'degenerate_projection') || ...
                       strcmp(case_id, 'degenerate_denominator');
        
        % オクルージョン状態の判定（直接フラグをチェック）
        % 少なくとも1つの隣人がオクルージョンなら、局所化不可能
        is_occluded = occlusion_j || occlusion_k;
        
        % 統合判定：共線状態またはオクルージョン
        is_unlocalizable = is_collinear || is_occluded;
        
        if is_unlocalizable
            % 局所化不可能状態（共線またはオクルージョン）
            % bearing誤差に基づいてτを設定
            % 理論：τ>0により、ψ（共線回避項）が有効化され、
            %       目標位置p^*への誘導が行われる
            tau_i = bearing_error;
        else
            % 局所化可能状態（正常）では、τは常にゼロ
            % 理論：ξ（相対測定からの補正）のみで制御
            tau_i = 0;
        end
        
        tau_vals(i) = tau_i;
        
        % === λの計算（式ref{eq:lambda}）===
        % 実位置と実速度を使用（測定可能な情報）
        p_i = p_true_all{follower_idx_global};
        v_i = v_f_mat(:, i);
        v_hat_i = v_hat_f_mat(:, i);
        v_i_star = v_star_all{follower_idx_global};
        
        % Δv_i = bar{e}_{v,i} - hat{e}_{v,i}
        %      = (v_i - v_i*) - (v_hat_i - v_i)
        Delta_v_i = (v_i - v_i_star) - (v_hat_i - v_i);
        
        % λ_i = -γ * Δv_i （式ref{eq:lambda}）
        if isfield(config, 'gamma')
            gamma = config.gamma;
        else
            gamma = 0.5; % デフォルト値
        end
        
        if isfield(config, 'lambda_max')
            lambda_max = config.lambda_max;
        else
            lambda_max = 0.9; % デフォルト値
        end
        
        lambda_i = -gamma * Delta_v_i;
        
        % 飽和処理（式ref{eq:lambda}）
        if norm(lambda_i) > lambda_max
            lambda_i = lambda_max * lambda_i / norm(lambda_i);
        end
        
        % === ψの計算（式ref{eq:psi}）===
        % sign関数の実装（成分ごと）
        sign_term = sign(p_hat_i - p_i_star);
        
        % ψ_i = -τ_i(sign(p̂_i - p_i*) - λ_i)
        psi_f(:, i) = -tau_i * (sign_term - lambda_i);
        
        % λのノルムを記録
        debug_info.lambda_norm(i) = norm(lambda_i);
    end
    
    %% === ξの一括計算（位置制約の勾配）===
    % 推定位置ベクトル化
    p_hat_vec = p_hat_f(:);  % [3*num_f x 1]
    p_l_vec = p_l(:);        % [3*m x 1]
    
    % **重要**: Bearing onlyの制御では真の位置pは使用できない
    % 相対測定（bearing）のみから計算される補正項
    % 
    % 元論文（Fang et al., Automatica 2025, 式20）:
    %   ξ_ijk = H_ii^T H_ij (p̂_j - p̂_i) + H_ii^T H_ik (p̂_k - p̂_i)
    % 
    % ブロック行列表記:
    %   ξ = -B_ff^T * (B_ff * p̂ + B_fl * p_l)
    %     = -B_ff^T * B_ff * p̂ - B_ff^T * B_fl * p_l
    % 
    % 各フォロワーiについて展開すると:
    %   ξ_i = H_ii^T H_ij (p̂_j - p̂_i) + H_ii^T H_ik (p̂_k - p̂_i) + ...
    % 
    % 重み行列H_ij, H_ikは「真のbearing」（センサ計測）から計算されるが、
    % ξの計算自体は推定位置のみを使用
    
    % Bearing制約の残差（推定位置のみから計算）
    Error = B_ff * p_hat_vec + B_fl * p_l_vec;
    
    % 完全な勾配 ξ = -B_ff^T * Error
    xi_vec_complete = -B_ff' * Error;
    
    % 行列形式に戻す [dim x num_f]
    xi_val = reshape(xi_vec_complete, dim, num_f);
end