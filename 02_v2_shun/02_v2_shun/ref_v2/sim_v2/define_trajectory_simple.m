function cfg = define_trajectory_simple()
    % =========================================================================
    % [機能] v4_observer.tex理論に厳密準拠した軌道設定
    % [特徴]
    %   ① 一定加速度ベクトルによる軌道生成（ゴール地点なし）
    %   ② 加速度 → 速度 → 位置の逐次計算
    %   ③ オブザーバ型二次系制御（理論式ref{eq:dynamics}, ref{eq:est_p}, ref{eq:est_v}, ref{eq:controller}）
    % =========================================================================

    %% シミュレーション基本設定
    cfg.t_start = 0;    % 開始時刻 [s]
    cfg.t_end   = 10;   % 終了時刻 [s]

    %% エージェント設定
    cfg.m = 2; % リーダーの数
    cfg.n = 5; % 全エージェント数
    cfg.num_followers = cfg.n - cfg.m; % フォロワーの数

    % 各フォロワーの隣接エージェント（ネイバー）を定義
    cfg.neighbors{3} = [1, 2]; % フォロワー3の隣人はリーダー1, 2
    cfg.neighbors{4} = [1, 3]; % フォロワー4の隣人はリーダー1, フォロワー3
    cfg.neighbors{5} = [3, 4]; % フォロワー5の隣人はフォロワー3, 4

    %% オブザーバ型制御パラメータ（v4_observer.tex Theorem 4.2）
    % 理論で要求される条件: K_obs > 0, K_p > 0, K_v > 0, γ ∈ (0,1], λ_max ∈ (0,1)
    cfg.K_obs = 10;       % オブザーバゲイン（式ref{eq:est_v}）
    cfg.K_p = 10;          % 位置制御ゲイン（式ref{eq:controller}）
    cfg.K_v = 10;         % 速度制御ゲイン（式ref{eq:controller}）
    cfg.gamma = 1;      % λ調整ゲイン（式ref{eq:lambda}）
    cfg.lambda_max = 0.99; % λ飽和限界（式ref{eq:lambda}）
    
    %% 実装上の調整ゲイン
    % 理論では明示的なゲインはないが、実装上の調整のため設定可能にする
    cfg.w_xi = 5.0;   % ξ項のゲイン（式ref{eq:est_p}）
    cfg.w_psi = 1.0;  % ψ項のゲイン（式ref{eq:controller}）
    
    %% 収束判定パラメータ
    cfg.convergence_threshold = 0; % 収束判定閾値 [m]

    %% 目標軌道パラメータ
    cfg.trajectory = struct();
    % type: 'constant'（一定加速度）, 'time_varying'（時変加速度）, 'circular'（円軌道）
    cfg.trajectory.type = 'constant';  % 軌道タイプ
    cfg.trajectory.accel_amplitude = 0.1;  % 加速度振幅 [m/s^2]
    cfg.trajectory.frequency = 2*pi/5;  % 時変加速度の周波数 [rad/s]
    cfg.trajectory.radius = 2.0;  % 円軌道の半径 [m]
    cfg.trajectory.angular_velocity = 0.5;  % 円軌道の角速度 [rad/s]

    %% 初期目標位置（t=0）
    cfg.p_initial_target = {
        [1; 1; 2],      % p1 (リーダー1)
        [1; -1; 2.1],   % p2 (リーダー2)
        [2; 2; 0],      % p3 (フォロワー3)
        [2; -2; 0.1],   % p4 (フォロワー4)
        [2; 0; -1]      % p5 (フォロワー5)
    };
    
    %% 初期推定誤差の設定（v4_observer.tex 仮定1の拡張）
    % 初期速度推定誤差 ||v̂_i(0) - v_i(0)|| ≤ ε_v
    cfg.initial_velocity_error = 0.2;  % 初期速度推定誤差 [m/s]
    
    %% 目標加速度ベクトルの定義（一定加速度の場合のみ使用）
    % 各エージェントの目標加速度（3次元ベクトル）
    % v4_observer.tex Remark（行124）: 加速度は任意で時変可能
    cfg.target_accel = {
        [0.1; 0; 0],    % p1 (リーダー1)
        [0.1; 0; 0],    % p2 (リーダー2)
        [0.1; 0; 0],    % p3 (フォロワー3)
        [0.1; 0; 0],    % p4 (フォロワー4)
        [0.1; 0; 0]     % p5 (フォロワー5)
    };

    %% エージェントの初期位置設定
    % v4_observer.tex 仮定1（式ref{eq:initial_condition}）: ||p̂_i(0) - p_i(0)|| ≤ ε_p
    
    % ① エージェントの実際の初期位置
    cfg.agent_actual_positions = {
        [-4; 3; -1];         % p3 (フォロワー3)
        [-1.5; -2.5; -4.4];  % p4 (フォロワー4)
        [-2; 0; -4]          % p5 (フォロワー5)
    };
    
    % ② エージェントの初期推定位置（手動指定）
    cfg.agent_estimated_positions = {
        [-1; 3; -0];         % p3 (フォロワー3) 推定位置
        [-1; -2; -4];  % p4 (フォロワー4) 推定位置
        [-2; 0; -3]          % p5 (フォロワー5) 推定位置
    };
    
    % 行列形式に変換
    p_f_initial_mat = zeros(3, cfg.num_followers);
    for i = 1:cfg.num_followers
        p_f_initial_mat(:, i) = cfg.agent_actual_positions{i};
    end
    cfg.p_f_initial = p_f_initial_mat;

    %% オクルージョン設定
    cfg.occlusion_enabled = true; % オクルージョンを有効化(false/true)
    cfg.occlusion_events = {{5, 'both', 0, 5.0}}; 
    cfg.check_occlusion = @(t, follower_id, neighbor_type) check_occlusion_func(t, follower_id, neighbor_type, cfg);

    %% 目標軌道生成用の関数ハンドル
    cfg.get_target_positions = @(t) get_simple_target_positions(t, cfg);
end

%% ========================================================================
%  内部関数：目標軌道生成（3種類の加速度タイプ対応）
% =========================================================================

function [p_star, v_star, a_star] = get_simple_target_positions(t, cfg)
    % =========================================================================
    % [機能] 目標軌道の逐次生成（v4_observer.tex Remark行124準拠）
    % [対応軌道]
    %   'constant': 一定加速度
    %   'time_varying': 時変加速度（正弦波）
    %   'circular': 円軌道
    % =========================================================================
    
    p_star = cell(1, cfg.n);
    v_star = cell(1, cfg.n);
    a_star = cell(1, cfg.n);
    
    % 時刻を[0, t_end]の範囲にクランプ
    t_clamped = max(0, min(t, cfg.t_end));
    
    switch cfg.trajectory.type
        case 'constant'
            % 一定加速度軌道
            for i = 1:cfg.n
                p_initial = reshape(cfg.p_initial_target{i}, 3, 1);
                accel = reshape(cfg.target_accel{i}, 3, 1);
                
                a_star{i} = accel;
                v_star{i} = accel * t_clamped;
                p_star{i} = p_initial + 0.5 * accel * t_clamped^2;
            end
            
        case 'time_varying'
            % 時変加速度軌道（正弦波）
            % a(t) = A·[sin(ω₁t); sin(ω₂t); sin(ω₃t)]
            A = cfg.trajectory.accel_amplitude;
            omega = cfg.trajectory.frequency;
            
            for i = 1:cfg.n
                p_initial = reshape(cfg.p_initial_target{i}, 3, 1);
                
                % 各軸で異なる周波数（非調和）
                a_x = A * sin(omega * t_clamped);
                a_y = A * sin(omega * 1.3 * t_clamped);
                a_z = A * sin(omega * 0.7 * t_clamped);
                a_star{i} = [a_x; a_y; a_z];
                
                % 速度：v(t) = ∫a(t)dt
                v_x = -(A/omega) * (cos(omega * t_clamped) - 1);
                v_y = -(A/(omega*1.3)) * (cos(omega * 1.3 * t_clamped) - 1);
                v_z = -(A/(omega*0.7)) * (cos(omega * 0.7 * t_clamped) - 1);
                v_star{i} = [v_x; v_y; v_z];
                
                % 位置：p(t) = p₀ + ∫v(t)dt
                p_x = p_initial(1) + (A/omega) * t_clamped - (A/omega^2) * sin(omega * t_clamped);
                p_y = p_initial(2) + (A/(omega*1.3)) * t_clamped - (A/(omega*1.3)^2) * sin(omega * 1.3 * t_clamped);
                p_z = p_initial(3) + (A/(omega*0.7)) * t_clamped - (A/(omega*0.7)^2) * sin(omega * 0.7 * t_clamped);
                p_star{i} = [p_x; p_y; p_z];
            end
            
        case 'circular'
            % 円軌道（xy平面）
            R = cfg.trajectory.radius;
            omega_c = cfg.trajectory.angular_velocity;
            
            for i = 1:cfg.n
                p_initial = reshape(cfg.p_initial_target{i}, 3, 1);
                center = p_initial + [R; 0; 0];  % 円の中心
                
                % 位置：p(t) = center + R[cos(ωt); sin(ωt); 0]
                p_star{i} = center + R * [cos(omega_c * t_clamped); sin(omega_c * t_clamped); 0];
                
                % 速度：v(t) = Rω[-sin(ωt); cos(ωt); 0]
                v_star{i} = R * omega_c * [-sin(omega_c * t_clamped); cos(omega_c * t_clamped); 0];
                
                % 加速度：a(t) = -Rω²[cos(ωt); sin(ωt); 0] （求心加速度）
                a_star{i} = -R * omega_c^2 * [cos(omega_c * t_clamped); sin(omega_c * t_clamped); 0];
            end
            
        otherwise
            error('未知の軌道タイプ: %s', cfg.trajectory.type);
    end
end

%% ========================================================================
%  オクルージョン判定関数
% =========================================================================

function is_occluded = check_occlusion_func(t, follower_id, neighbor_type, cfg)
    is_occluded = false;
    
    if ~cfg.occlusion_enabled
        return;
    end
    
    for i = 1:length(cfg.occlusion_events)
        event = cfg.occlusion_events{i};
        event_follower_id = event{1};
        event_neighbor_type = event{2};
        start_time = event{3};
        end_time = event{4};
        
        if event_follower_id ~= follower_id
            continue;
        end
        
        if t < start_time || t > end_time
            continue;
        end
        
        if strcmp(event_neighbor_type, 'both') || strcmp(event_neighbor_type, neighbor_type)
            is_occluded = true;
            return;
        end
    end
end
