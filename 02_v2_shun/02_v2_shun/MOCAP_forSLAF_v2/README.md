# オブザーバ型SLAF実機実装プログラム (v2)

## 概要
このプログラムは、オブザーバ型SLAF（Simultaneous Localization And Formation tracking control）制御を実機ドローンで実装したものです。

### 理論ベース
- **理論文書**: `02_v2_shun/ref_v2/v4_observer.tex`
- **シミュレーション**: `02_v2_shun/ref_v2/sim_v2/system_dynamics.m`
- **参照実装**: `01_v1_PID/MOCAP_forSLAF/` (v1, PID階層型)

### v1との主な違い
| 項目 | v1 (PID階層型) | v2 (オブザーバ型) |
|------|----------------|-------------------|
| **推定器** | PID型（5状態） | Luenbergerオブザーバ型（4状態） |
| **積分状態** | あり（I制御） | **なし** |
| **速度フィードバック** | `v̂ᵢ - vᵢ` | `vᵢ - v̂ᵢ`（オブザーバ） |
| **理論構造** | 統合型 | **完全分離（カスケード）** |
| **目標加速度** | 理論上要求 | **任意（完全キャンセル）** |
| **ゲイン** | `k_p, k_v, k_i, k_cp, k_cv` | `K_obs, K_p, K_v` |
| **補正ゲイン** | `xi_gain` | `w_xi, w_psi` |

---

## 制御則（厳密な理論実装）

### 推定器（Luenbergerオブザーバ型）
v4_observer.tex 式(143-144)に準拠：
```
˙p̂ᵢ = vᵢ + w_xi * ξᵢ                    (式143)
˙v̂ᵢ = uᵢ + K_obs(vᵢ - v̂ᵢ)               (式144)
```

**重要な特徴**:
- 実速度 `vᵢ` を直接使用（測定可能と仮定、実機では数値微分）
- オブザーバゲイン `K_obs` による速度推定誤差の収束
- **積分状態 `zᵢ` は存在しない**（I制御なし）

### 制御器（PD型 + フィードフォワード）
v4_observer.tex 式(174)に準拠：
```
uᵢ = p̈ᵢ* - K_p(p̂ᵢ - pᵢ*) - K_v(vᵢ - ṗᵢ*) + w_psi * ψᵢ  (式174)
```

**重要な特徴**:
- 目標加速度 `p̈ᵢ*` をフィードフォワード（完全キャンセル）
- 推定位置 `p̂ᵢ` を使用（位置フィードバック）
- 実速度 `vᵢ` を使用（速度フィードバック）

### 状態変数（4変数）
```python
pᵢ       # 実位置 [x, z]
vᵢ       # 実速度 [vx, vz]（数値微分）
p̂ᵢ       # 推定位置 [x_hat, z_hat]
v̂ᵢ       # 推定速度 [vx_hat, vz_hat]

# ★積分状態 zᵢ は存在しない★
```

---

## ゲイン設定

### デフォルト値（シミュレーションと同じ）
```python
# オブザーバゲイン
K_obs = 10.0   # 速度推定誤差の収束速度

# 制御器ゲイン
K_p = 10.0     # 位置制御ゲイン（固有振動数 ω_n = √K_p ≈ 3.16 rad/s）
K_v = 10.0     # 速度制御ゲイン（減衰比 ζ = K_v/(2√K_p) ≈ 1.58）

# 補正項のゲイン（実装上の調整）
w_xi = 5.0     # ξ項のゲイン（シミュレーションと同じ）
w_psi = 1.0    # ψ項のゲイン（シミュレーションと同じ）

# 共線回避パラメータ
gamma = 1.0
lambda_max = 0.99
```

### ゲイン調整指針

#### 1. オブザーバゲイン `K_obs`
- 大きい → 速い収束、振動的
- 小さい → 遅い収束、滑らか
- 推奨範囲: 5.0 ~ 20.0

#### 2. 位置制御ゲイン `K_p`
- 固有振動数: ω_n = √K_p
- 推奨 ω_n: 2 ~ 5 rad/s
- 推奨 K_p: 4 ~ 25

#### 3. 速度制御ゲイン `K_v`
- 減衰比: ζ = K_v / (2√K_p)
- 推奨 ζ: 0.7 ~ 1.5（臨界減衰付近）

#### 4. 補正ゲイン `w_xi`, `w_psi`
- `w_xi`: 推定誤差の収束速度に影響（推奨: 1.0 ~ 10.0）
- `w_psi`: オクルージョン時の共線回避の強さ（推奨: 0.5 ~ 2.0）

---

## システム構成

### 主要ファイル

#### 1. `slaf_observer_controller.py` ★新規（v2専用）★
オブザーバ型SLAF制御システムの中核

- **クラス**:
  - `SLAFObserverController`: 単一フォロワーの制御器
  - `SLAFSystemManager`: 複数フォロワーの統合管理

- **理論対応**:
  - `update_estimator()`: v4_observer.tex 式(143-144)
  - `calculate_control_input()`: v4_observer.tex 式(174)
  - `K_obs, K_p, K_v`: 定理4.2の条件を満たす

#### 2. `mocap_slaf_main.py`
メインプログラム（v1から変更：インポートとタイトルのみ）

#### 3. `virtual_leader.py`
仮想リーダーの軌道生成と管理（v1と同じ）

#### 4. `weight_matrices.py`
Bearingベースの重み行列計算（v1と同じ）

#### 5. その他のモジュール（v1と同じ）
- `mocap_stream.py`: MOCAP連携
- `custom_tello.py`: Telloドローン制御
- `keyboard_control.py`: キーボード入力
- `csv_logger.py`: CSVログ記録

---

## 設計方針

### 1. 仮想リーダーの配置
- **固定位置配置**: 仮想リーダー1, 2は固定位置に配置
- **役割**: 相対測定の基準点として機能
- **位置**:
  - リーダー1: `[0.0, 1.0, -0.5]` (x, y, z)
  - リーダー2: `[0.0, 1.0, 0.5]`

### 2. フォロワー（実機ドローン）
- **ドローン構成**:
  - フォロワー3 = 実機ドローン1（TelloID 0）
  - フォロワー4 = 実機ドローン2（TelloID 1）
- **初期位置**:
  - フォロワー3: `[0.0, 1.0, 0.0]`
  - フォロワー4: `[0.0, 1.0, 1.0]`

### 3. 初期位置設定（v1準拠、重要）

#### 3つの位置の概念

**Tキー押下時に現在のMOCAP位置を一瞬取得して設定**（固定値は使わない）

1. **初期位置** `pᵢ(0)`: MOCAPから取得した現在位置
2. **初期目標位置** `pᵢ*(0)`: MOCAPから取得した現在位置（初期位置と同じ）
3. **初期推定位置** `p̂ᵢ(0)`: 初期目標位置をコピー（推定誤差零の仮定）

#### 初期化の流れ（Tキー押下時）

```python
# 1. MOCAPから現在位置を取得（一瞬のみ）
mocap_pos = ms.get_rigid_body_position(rigid_id)
x = mocap_pos.get('x', 0.0)
z = mocap_pos.get('z', 0.0)

# 2. 目標位置を現在位置に設定
follower_target_positions[follower_id] = np.array([x, z])  # pᵢ*(0) = pᵢ(0)

# 3. 推定器を初期化（推定誤差零）
controller.initialize_state(
    follower_target_positions[follower_id].copy(),  # p̂ᵢ(0) = pᵢ*(0) = pᵢ(0)
    np.array([0.0, 0.0])                            # v̂ᵢ(0) = 0
)
```

**★重要★**: 固定値（`DEFAULT_FOLLOWER_POSITIONS`等）は**使用しない**

### 4. 目標加速度
- **v1と同様に一定値で検証**:
  ```python
  target_acceleration_2d = np.array([0.03, -0.03])  # [X, Z] m/s^2
  ```
- v2では目標加速度は完全にキャンセルされるため、任意の値でよい

### 5. 速度測定
- **実速度は数値微分で推定**（v1と同じ方法）
- オブザーバは速度推定誤差を補正

### 6. オクルージョン
- **手動設定**（自分で明示的に発生させる）
- Oキー: オクルージョンON
- Pキー: オクルージョンOFF
- オクルージョン時: ξ=0, ψが有効

---

## 必要なハードウェア
- Tello EDUドローン x2
- モーションキャプチャシステム（OptiTrack等）

## 必要なソフトウェア
- Python 3.8以上
- djitellopy
- numpy
- pygame
- NatNetClient (OptiTrack用)

---

## セットアップ

### 1. 依存パッケージのインストール
```bash
pip install djitellopy numpy pygame opencv-python
```

### 2. MOCAPシステムの設定
- OptiTrackの場合、NatNetClientを適切に設定
- Rigid Body IDを設定：
  - ドローン1（TelloID 0）: Rigid Body ID 1
  - ドローン2（TelloID 1）: Rigid Body ID 2

### 3. Telloドローンの設定
- 2機のTello EDUドローンをステーションモードで接続
- WiFi経由で制御PC接続

---

## 使用方法

### 1. プログラム実行
```bash
cd /path/to/MOCAP_forSLAF_v2
python mocap_slaf_main.py
```

### 2. キーボード操作

#### 基本操作
- **Q**: 全ドローン同時離陸
- **E**: 全ドローン同時着陸
- **ESC**: 緊急停止
- **SPACE**: 正常終了

#### モード切替
- **T**: SLAF制御モード開始
  - 現在のMOCAP位置を目標位置として設定
  - 推定器を初期化（推定誤差零）
  - Hモードをリセット
- **M**: 手動モード

#### SLAF制御モード（Tモード後）
- **H**: Hモード開始（目標軌道生成開始）
  - 目標加速度: `[0.03, -0.03]` m/s^2
  - 目標速度・位置が時変的に更新される
- **J**: Hモード停止
  - 目標速度・位置が固定される
- **O**: オクルージョンモードON（フォロワー4）
  - ξ = 0（幾何学的補正なし）
  - ψ, τが有効（共線回避動作）
- **P**: オクルージョンモードOFF（フォロワー4）

---

## 制御フロー

### 初期化フェーズ
1. ドローン接続
2. MOCAP接続
3. SLAFシステム初期化
   - 仮想リーダー管理初期化（固定位置）
   - オブザーバ型制御器初期化
4. CSVロガー初期化

### 制御ループ（Tモード時、10Hz）
1. **MOCAP測定**: 3D座標取得 → 2D変換 [x, z]
2. **速度推定**: 数値微分 `(pᵢ - pᵢ₋₁)/dt`
3. **Hモード**: 目標軌道更新（有効時）
   - `v*ₙₑw = v*ₒₗd + a* * dt`
   - `p*ₙₑw = p*ₒₗd + v* * dt`
4. **幾何学的補正項 ξ 計算**:
   - 重み行列Hを実位置（観測値）から計算
   - ξを推定位置から計算
   - オクルージョン時はξ=0
5. **推定器更新**（オブザーバ型）:
   - `˙p̂ᵢ = vᵢ + w_xi * ξᵢ`
   - `˙v̂ᵢ = uᵢ + K_obs(vᵢ - v̂ᵢ)`
6. **共線回避項 ψ 計算**（オクルージョン時のみ）
7. **制御入力計算**:
   - `uᵢ = aᵢ* - K_p(p̂ᵢ - pᵢ*) - K_v(vᵢ - vᵢ*) + w_psi * ψᵢ`
8. **RC値変換**: 加速度指令 → RC値（±20制限）
9. **コマンド送信**: `send_rc_control(lr, fb, ud, yaw)`
10. **CSVログ記録**（0.5秒ごと）

---

## データログ

### ログディレクトリ
```
slaf_results/
├── control_log_YYYYMMDD_HHMMSS.csv
└── (他のログファイル)
```

### 記録されるデータ（control_log.csv）
- **時刻**: timestamp
- **ドローンID**: drone_id, follower_id
- **モード**: mode ('slaf' or 'manual')
- **位置**: position [x, z], position_hat [x_hat, z_hat]
- **速度**: velocity [vx, vz], velocity_hat [vx_hat, vz_hat]
- **目標**: target_position, target_velocity
- **制御入力**: control_input [ax, az]
- **RC指令**: rc_command [lr, fb, ud, yaw]
- **補正項**: xi, psi, tau
- **誤差**: tracking_error, estimation_error
- **状態**: is_collinear, is_occluded
- **オブザーバ**: observer_feedback

---

## パラメータ調整

### 1. ゲイン調整（`slaf_observer_controller.py`）
```python
# オブザーバ型制御器のゲイン
K_obs = 10.0   # オブザーバゲイン
K_p = 10.0     # 位置制御ゲイン
K_v = 10.0     # 速度制御ゲイン
w_xi = 5.0     # ξゲイン
w_psi = 1.0    # ψゲイン
```

### 2. 目標加速度（`mocap_slaf_main.py`）
```python
target_acceleration_2d = np.array([0.03, -0.03])  # [X, Z] m/s^2
```

### 3. 不感帯（`slaf_observer_controller.py`）
```python
self.deadband_x = 0.0  # x方向（前後）の不感帯 (m)
self.deadband_z = 0.0  # z方向（左右）の不感帯 (m)
```

---

## トラブルシューティング

### 1. MOCAP接続失敗
- OptiTrackが起動しているか確認
- NatNetClientのネットワーク設定を確認
- ファイアウォールが通信をブロックしていないか確認

### 2. ドローン接続失敗
- TelloドローンがステーションモードでWiFi接続されているか確認
- `djitellopy`が最新版か確認

### 3. 制御が不安定
- ゲインを調整：
  - K_obsを下げる（振動を抑制）
  - K_pを下げる（応答を緩やか）
  - K_vを上げる（減衰を増加）
- 不感帯を設定（ノイズ低減）

### 4. 推定誤差が収束しない
- w_xiを上げる（幾何学的補正を強化）
- K_obsを上げる（オブザーバの収束を加速）
- MOCAP測定値のノイズを確認

### 5. 定常偏差が残る
- v2はI制御がないため、定常偏差が残る可能性あり
- 目標位置設計を見直す
- v1（PID階層型）の使用を検討

---

## 理論的背景

### 収束性証明
v4_observer.tex 定理4.2より：

**補題（推定器）**:
- 初期条件 `p̂ᵢ(0) = pᵢ(0)`, `v̂ᵢ(0) = vᵢ(0)` の下で：
  ```
  p̂ᵢ(t) ≡ pᵢ(t),  v̂ᵢ(t) ≡ vᵢ(t),  ∀t ≥ 0
  ```

**定理（制御器）**:
- 推定誤差が零のため、追跡誤差は漸近的に零に収束：
  ```
  lim(t→∞) ||pᵢ(t) - pᵢ*(t)|| = 0
  lim(t→∞) ||vᵢ(t) - ṗᵢ*(t)|| = 0
  ```

### カスケード構造
- **推定器系**: 完全独立（追跡誤差が現れない）
- **制御器系**: 一方向結合（推定誤差の影響のみ）
- **分離原理**: それぞれ独立に証明可能

---

## 参考資料

### 理論文書
- `02_v2_shun/ref_v2/v4_observer.tex`: 厳密な収束性証明
- `02_v2_shun/ref_v2/README.md`: v2の理論概要
- `02_v2_shun/Read.me`: プロジェクト全体の技術文書

### シミュレーション
- `02_v2_shun/ref_v2/sim_v2/main_simple.m`: MATLABシミュレーション
- `02_v2_shun/ref_v2/sim_v2/system_dynamics.m`: 状態方程式

### 設計書
- `V2_OBSERVER_REAL_IMPLEMENTATION_DESIGN.md`: v2実機実装設計書（本ディレクトリ内）

### v1参照実装
- `01_v1_PID/MOCAP_forSLAF/`: PID階層型実機実装（完成・検証済み）
- `01_v1_PID/MOCAP_forSLAF/slaf_pid_controller.py`: v1制御器（比較用）

---

## ライセンス
研究・教育目的での利用に限ります。

---

**作成日**: 2025年12月11日  
**理論ベース**: v4_observer.tex, sim_v2/system_dynamics.m  
**実装**: slaf_observer_controller.py (v2専用)  
**参照実装**: 01_v1_PID/MOCAP_forSLAF/ (v1, PID階層型)
