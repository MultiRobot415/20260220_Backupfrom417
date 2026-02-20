function [Hij, Hik, case_id] = calculate_weight_matrices(p_i, p_j, p_k, occlusion_j, occlusion_k)
    % =========================================================================
    % [機能]
    %   論文 Appendix A の21ケースに完全準拠した重み行列計算
    %   
    % [処理順序]
    %   1. 3D共線判定（Algorithm 1）
    %   2. 相似な配置 q の計算（Appendix C）
    %   3. 2D射影（α, β, γ）の計算
    %   4. 21ケースの判定（Appendix A）
    %   5. 各ケースの重み行列計算式（式49～69）
    %
    % [引数]
    %   p_i, p_j, p_k: エージェントi, j, kの3D位置ベクトル [3×1]
    %   occlusion_j, occlusion_k: オクルージョンフラグ (optional)
    %
    % [工学的閾値]
    %   EPS: 位置重複判定閾値
    %   EPS_COLLINEAR: 共線判定閾値
    %   EPS_CASE: 2D射影での重なり判定閾値
    % =========================================================================
    
    % オクルージョン引数のデフォルト値設定
    if nargin < 4
        occlusion_j = false;
    end
    if nargin < 5
        occlusion_k = false;
    end
    
    %% 工学的閾値の定義（調整可能）
    EPS = 0.1;              % 位置の一致判定閾値 [m]
    EPS_COLLINEAR = 0.999;  % 共線判定閾値（cos(θ)の絶対値）
    EPS_CASE = 0.05;         % ケース分岐判定閾値 [m]

    %% 1. 位置の重複・共線状態の判定（工学的閾値使用）
    v_ij = p_j - p_i; n_ij = norm(v_ij);
    v_ik = p_k - p_i; n_ik = norm(v_ik);
    
    % 位置の重複判定（collocation）
    if n_ij < EPS || n_ik < EPS
        Hij = zeros(3,3); Hik = zeros(3,3); 
        case_id = 'collocation'; 
        return;
    end
    
    % 共線判定（collinear）
    cos_theta = dot(v_ij, v_ik) / (n_ij * n_ik);
    if abs(cos_theta) > EPS_COLLINEAR
        Hij = zeros(3,3); Hik = zeros(3,3); 
        case_id = 'collinear'; 
        return;
    end

    %% 2. 相似な配置 q の計算（特異点回避）
    g_ij = v_ij / n_ij; 
    g_ik = v_ik / n_ik; 
    v_jk = p_k - p_j;
    n_jk = norm(v_jk);
    
    % j-k間の距離チェック
    if n_jk < EPS
        Hij = zeros(3,3); Hik = zeros(3,3); 
        case_id = 'collocation_jk'; 
        return;
    end
    
    g_jk = v_jk / n_jk;
    q_i = [0;0;0]; 
    q_j = g_ij;
    
    % 角度計算（数値安定性のためクランプ）
    theta_ijk = acos(max(-1, min(1, -dot(g_ij, g_jk))));
    theta_ikj = acos(max(-1, min(1, dot(g_ik, g_jk))));
    
    % 特異点判定（sin(theta_ikj)がゼロに近い場合）
    % sin(θ) < EPS の場合、θ ≈ 0 または π（共線に近い）
    if sin(theta_ikj) < EPS
        Hij = zeros(3,3); Hik = zeros(3,3); 
        case_id = 'degenerate'; 
        return;
    end
    
    d_ik = norm(q_j-q_i) * sin(theta_ijk) / sin(theta_ikj);
    q_k = d_ik * g_ik;
    
    %% 3. 全パラメータ s1～s12 の計算
    alpha_i=[q_i(1);q_i(2)]; alpha_j=[q_j(1);q_j(2)]; alpha_k=[q_k(1);q_k(2)];
    beta_i =[q_i(2);q_i(3)]; beta_j =[q_j(2);q_j(3)]; beta_k =[q_k(2);q_k(3)];
    gamma_i=[q_i(3);q_i(1)]; gamma_j=[q_j(3);q_j(1)]; gamma_k=[q_k(3);q_k(1)];

    % 2D射影での距離を計算（ケース判定に使用）
    n_alpha_ij = norm(alpha_i-alpha_j); n_alpha_ik = norm(alpha_i-alpha_k); n_alpha_jk = norm(alpha_j-alpha_k);
    n_beta_ij = norm(beta_i-beta_j); n_beta_ik = norm(beta_i-beta_k); n_beta_jk = norm(beta_j-beta_k);
    n_gamma_ij = norm(gamma_i-gamma_j); n_gamma_ik = norm(gamma_i-gamma_k);
    
    %% 4. 21ケースの判定（論文 Appendix A に完全準拠）
    % 工学的閾値を使用して2D射影での重なりを判定
    tol = EPS_CASE;
    
    % α平面での重なり判定
    alpha_ij_overlap = (n_alpha_ij < tol);
    alpha_ik_overlap = (n_alpha_ik < tol);
    alpha_jk_overlap = (n_alpha_jk < tol);
    
    % β平面での重なり判定
    beta_ij_overlap = (n_beta_ij < tol);
    beta_ik_overlap = (n_beta_ik < tol);
    beta_jk_overlap = (n_beta_jk < tol);
    
    % γ平面での重なり判定
    gamma_ij_overlap = (n_gamma_ij < tol);
    gamma_ik_overlap = (n_gamma_ik < tol);
    
    %% 5. パラメータ s1～s12 の計算（論文式45）
    % 各ケースで必要なパラメータのみを安全に計算
    % ゼロ除算を避けるため、分母が小さい場合は計算をスキップ
    
    s = zeros(1, 12);  % 初期化
    
    % s1～s4の計算（α平面、Case 1, 3, 4で使用）
    if ~alpha_ij_overlap && ~alpha_ik_overlap
        t1 = (alpha_j(1)-alpha_i(1))/n_alpha_ij^2;
        t2 = (alpha_i(1)-alpha_k(1))/n_alpha_ik^2;
        t3 = (alpha_j(2)-alpha_i(2))/n_alpha_ij^2;
        t4 = (alpha_i(2)-alpha_k(2))/n_alpha_ik^2;
        denom1 = (t1+t2)^2 + (t3+t4)^2;
        if denom1 > EPS^2
            s(1) = (t1*(t1+t2) + t3*(t3+t4))/denom1;
            s(2) = (-t3*(t1+t2) + t1*(t3+t4))/denom1;
            s(3) = (t2*(t1+t2) + t4*(t3+t4))/denom1;
            s(4) = (-t4*(t1+t2) + t2*(t3+t4))/denom1;
        end
    end
    
    % s5～s8の計算（β平面、Case 1, 2, 3, 4で使用）
    if ~beta_ij_overlap && ~beta_ik_overlap
        t5 = (beta_j(1)-beta_i(1))/n_beta_ij^2;
        t6 = (beta_i(1)-beta_k(1))/n_beta_ik^2;
        t7 = (beta_j(2)-beta_i(2))/n_beta_ij^2;
        t8 = (beta_i(2)-beta_k(2))/n_beta_ik^2;
        denom2 = (t5+t6)^2 + (t7+t8)^2;
        if denom2 > EPS^2
            s(5) = (t5*(t5+t6) + t7*(t7+t8))/denom2;
            s(6) = (-t7*(t5+t6) + t5*(t7+t8))/denom2;
            s(7) = (t6*(t5+t6) + t8*(t7+t8))/denom2;
            s(8) = (-t8*(t5+t6) + t6*(t7+t8))/denom2;
        end
    end
    
    % s9～s12の計算（γ平面、Case 1, 2, 4で使用）
    if ~gamma_ij_overlap && ~gamma_ik_overlap
        t9 = (gamma_j(2)-gamma_i(2))/n_gamma_ij^2;
        t10 = (gamma_i(2)-gamma_k(2))/n_gamma_ik^2;
        t11 = (gamma_j(1)-gamma_i(1))/n_gamma_ij^2;
        t12 = (gamma_i(1)-gamma_k(1))/n_gamma_ik^2;
        denom3 = (t9+t10)^2 + (t11+t12)^2;
        if denom3 > EPS^2
            s(9) = (t9*(t9+t10) + t11*(t11+t12))/denom3;
            s(10) = (-t11*(t9+t10) + t9*(t11+t12))/denom3;
            s(11) = (t10*(t9+t10) + t12*(t11+t12))/denom3;
            s(12) = (-t12*(t9+t10) + t10*(t11+t12))/denom3;
        end
    end
    
    %% 6. 21ケースの分類（論文 Appendix A に完全準拠）
    % Case(1): αi ≠ αj ≠ αk
    % Case(2): αi = αj
    % Case(3): αi = αk
    % Case(4): αj = αk
    
    if ~alpha_ij_overlap && ~alpha_jk_overlap && ~alpha_ik_overlap
        % Case (1): αi ≠ αj ≠ αk
        if beta_ij_overlap
            case_id = '1_1';  % (1.1) βi = βj
        elseif beta_ik_overlap
            case_id = '1_2';  % (1.2) βi = βk
        elseif beta_jk_overlap && gamma_ij_overlap
            case_id = '1_4';  % (1.4) βj = βk, γi = γj
        elseif beta_jk_overlap && gamma_ik_overlap
            case_id = '1_5';  % (1.5) βj = βk, γi = γk
        elseif beta_jk_overlap
            case_id = '1_6';  % (1.6) βj = βk, γi ≠ γj ≠ γk
        else
            case_id = '1_3';  % (1.3) βi ≠ βj ≠ βk
        end
        
    elseif alpha_ij_overlap
        % Case (2): αi = αj
        if beta_ik_overlap
            case_id = '2_1';  % (2.1) βi = βk
        elseif beta_jk_overlap && gamma_ik_overlap
            case_id = '2_3';  % (2.3) βj = βk, γi = γk
        elseif beta_jk_overlap
            case_id = '2_4';  % (2.4) βj = βk, γi ≠ γj ≠ γk
        else
            case_id = '2_2';  % (2.2) βi ≠ βj ≠ βk
        end
        
    elseif alpha_ik_overlap
        % Case (3): αi = αk
        if beta_ij_overlap
            case_id = '3_1';  % (3.1) βi = βj
        elseif beta_jk_overlap && gamma_ij_overlap
            case_id = '3_3';  % (3.3) βj = βk, γi = γj
        elseif beta_jk_overlap
            case_id = '3_4';  % (3.4) βj = βk, γi ≠ γj ≠ γk
        else
            case_id = '3_2';  % (3.2) βi ≠ βj ≠ βk
        end
        
    elseif alpha_jk_overlap
        % Case (4): αj = αk
        if beta_ij_overlap && gamma_ik_overlap
            case_id = '4_1';  % (4.1) βi = βj, γi = γk
        elseif beta_ij_overlap
            case_id = '4_2';  % (4.2) βi = βj, γi ≠ γk
        elseif beta_ik_overlap && gamma_ij_overlap
            case_id = '4_3';  % (4.3) βi = βk, γi = γj
        elseif beta_ik_overlap
            case_id = '4_4';  % (4.4) βi = βk, γi ≠ γj
        elseif gamma_ij_overlap
            case_id = '4_5';  % (4.5) βi ≠ βj ≠ βk, γi = γj
        elseif gamma_ik_overlap
            case_id = '4_6';  % (4.6) βi ≠ βj ≠ βk, γi = γk
        else
            case_id = '4_7';  % (4.7) βi ≠ βj ≠ βk, γi ≠ γj ≠ γk
        end
    else
        % 該当するケースなし（理論上は起こらない）
        Hij = zeros(3,3); Hik = zeros(3,3);
        case_id = 'undefined';
        return;
    end
    
    %% 7. 重み行列の計算（論文 式49～69に完全準拠）
    switch case_id
        % Case (1): αi ≠ αj ≠ αk
        case '1_1'  % 式(49): βi = βj
            Hij = [s(1), -s(2), 0; s(2), 1+s(1), 0; 0, 0, 1];
            Hik = [s(3), -s(4), 0; s(4), s(3), 0; 0, 0, 0];
            
        case '1_2'  % 式(50): βi = βk
            Hij = [s(1), -s(2), 0; s(2), s(1), 0; 0, 0, 0];
            Hik = [s(3), -s(4), 0; s(4), 1+s(3), 0; 0, 0, 1];
            
        case '1_3'  % 式(51): βi ≠ βj ≠ βk
            Hij = [s(1), -s(2), 0; s(2), s(1)+s(5), -s(6); 0, s(6), s(5)];
            Hik = [s(3), -s(4), 0; s(4), s(3)+s(7), -s(8); 0, s(8), s(7)];
            
        case '1_4'  % 式(52): βj = βk, γi = γj
            Hij = [1+s(1), -s(2), 0; s(2), s(1), 0; 0, 0, 1];
            Hik = [s(3), -s(4), 0; s(4), s(3), 0; 0, 0, 0];
            
        case '1_5'  % 式(53): βj = βk, γi = γk
            Hij = [s(1), -s(2), 0; s(2), s(1), 0; 0, 0, 0];
            Hik = [1+s(3), -s(4), 0; s(4), s(3), 0; 0, 0, 1];
            
        case '1_6'  % 式(54): βj = βk, γi ≠ γj ≠ γk
            Hij = [s(1)+s(9), -s(2), -s(10); s(2), s(1), 0; s(10), 0, s(9)];
            Hik = [s(3)+s(11), -s(4), -s(12); s(4), s(3), 0; s(12), 0, s(11)];
            
        % Case (2): αi = αj
        case '2_1'  % 式(55): βi = βk
            Hij = [1, 0, 0; 0, 1, 0; 0, 0, 0];
            Hik = [0, 0, 0; 0, 1, 0; 0, 0, 1];
            
        case '2_2'  % 式(56): βi ≠ βj ≠ βk
            Hij = [1, 0, 0; 0, 1+s(5), -s(6); 0, s(6), s(5)];
            Hik = [0, 0, 0; 0, s(7), -s(8); 0, s(8), s(7)];
            
        case '2_3'  % 式(57): βj = βk, γi = γk
            Hij = [1, 0, 0; 0, 1, 0; 0, 0, 0];
            Hik = [1, 0, 0; 0, 0, 0; 0, 0, 1];
            
        case '2_4'  % 式(58): βj = βk, γi ≠ γj ≠ γk
            Hij = [1+s(9), 0, -s(10); 0, 1, 0; s(10), 0, s(9)];
            Hik = [s(11), 0, -s(12); 0, 0, 0; s(12), 0, s(11)];
            
        % Case (3): αi = αk
        case '3_1'  % 式(59): βi = βj
            Hij = [0, 0, 0; 0, 1, 0; 0, 0, 1];
            Hik = [1, 0, 0; 0, 1, 0; 0, 0, 0];
            
        case '3_2'  % 式(60): βi ≠ βj ≠ βk
            Hij = [0, 0, 0; 0, s(5), -s(6); 0, s(6), s(5)];
            Hik = [1, 0, 0; 0, 1+s(7), -s(8); 0, s(8), s(7)];
            
        case '3_3'  % 式(61): βj = βk, γi = γj
            Hij = [1, 0, 0; 0, 0, 0; 0, 0, 1];
            Hik = [1, 0, 0; 0, 1, 0; 0, 0, 0];
            
        case '3_4'  % 式(62): βj = βk, γi ≠ γj ≠ γk
            Hij = [s(9), 0, -s(10); 0, 0, 0; s(10), 0, s(9)];
            Hik = [1+s(11), 0, -s(12); 0, 1, 0; s(12), 0, s(11)];
            
        % Case (4): αj = αk
        case '4_1'  % 式(63): βi = βj, γi = γk
            Hij = [0, 0, 0; 0, 1, 0; 0, 0, 1];
            Hik = [1, 0, 0; 0, 0, 0; 0, 0, 1];
            
        case '4_2'  % 式(64): βi = βj, γi ≠ γk
            Hij = [s(9), 0, -s(10); 0, 1, 0; s(10), 0, 1+s(9)];
            Hik = [s(11), 0, -s(12); 0, 0, 0; s(12), 0, s(11)];
            
        case '4_3'  % 式(65): βi = βk, γi = γj
            Hij = [1, 0, 0; 0, 0, 0; 0, 0, 1];
            Hik = [0, 0, 0; 0, 1, 0; 0, 0, 1];
            
        case '4_4'  % 式(66): βi = βk, γi ≠ γj
            Hij = [s(9), 0, -s(10); 0, 0, 0; s(10), 0, s(9)];
            Hik = [s(11), 0, -s(12); 0, 1, 0; s(12), 0, 1+s(11)];
            
        case '4_5'  % 式(67): βi ≠ βj ≠ βk, γi = γj
            Hij = [1, 0, 0; 0, s(5), -s(6); 0, s(6), 1+s(5)];
            Hik = [0, 0, 0; 0, s(7), -s(8); 0, s(8), s(7)];
            
        case '4_6'  % 式(68): βi ≠ βj ≠ βk, γi = γk
            Hij = [0, 0, 0; 0, s(5), -s(6); 0, s(6), s(5)];
            Hik = [1, 0, 0; 0, s(7), -s(8); 0, s(8), 1+s(7)];
            
        case '4_7'  % 式(69): βi ≠ βj ≠ βk, γi ≠ γj ≠ γk
            Hij = [s(9), 0, -s(10); 0, s(5), -s(6); s(10), s(6), s(5)+s(9)];
            Hik = [s(11), 0, -s(12); 0, s(7), -s(8); s(12), s(8), s(7)+s(11)];
            
        otherwise
            % 該当するケースなし
            Hij = zeros(3,3); Hik = zeros(3,3);
            warning('未定義のケースID: %s', case_id);
    end
    
    %% 8. オクルージョン処理
    % オクルージョンが発生している場合、対応する重み行列を0にする
    if occlusion_j
        Hij = zeros(3,3);
        if ~occlusion_k  % kがオクルージョンでない場合のみcase_idを更新
            case_id = [case_id, '_occluded_j'];
        end
    end
    
    if occlusion_k
        Hik = zeros(3,3);
        if ~occlusion_j  % jがオクルージョンでない場合のみcase_idを更新
            case_id = [case_id, '_occluded_k'];
        end
    end
    
    % 両方がオクルージョンの場合
    if occlusion_j && occlusion_k
        case_id = [case_id, '_occluded_both'];
    end
end