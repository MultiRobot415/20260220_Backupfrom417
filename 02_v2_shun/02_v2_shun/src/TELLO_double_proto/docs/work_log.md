# TELLO_double_proto 作業履歴

## 2025-05-01 11:14
- プロジェクトの初期設定
  - TELLO_double_protoディレクトリ内に基本構造（src, cmd, docs）を作成
  - 作業履歴ファイル（work_log.md）を作成
- 作業目標
  - 複数機（2機）のTelloドローンをキーボード操作で同時制御するプログラムの実装
  - ステーションモードでの接続を前提
  - キーボードアサインはTELLO_single内と一致させる
  - Python3環境でDJITellopyを使用

## 2025-05-01 11:20
- 既存コードの調査と分析
  - TELLO_singleのキーボード制御コード（Tello_KeyControl1.py, KeyPressModule.py）を確認
  - TELLO_devのマルチドローン制御コード（keyboard_formation_control.py, multi_tello_test_py3.py, tello_manager_py3.py）を確認

- 新規ファイルの作成
  - KeyPressModule.py - キーボード入力を検出するモジュール
  - tello_double_control.py - 2機のTelloドローンを同時制御するメインプログラム
  - drone_config.yaml - ドローン設定ファイル（IPアドレスや制御パラメータ）

- 実装機能
  - YAMLファイルからの設定読み込み
  - 複数ドローンへの同時接続
  - キーボード操作による同時制御（前後左右、上昇下降、回転）
  - 離陸・着陸・緊急停止機能

## 2025-05-01 11:29
- tello_managerの実装
  - TELLO_devのtello_manager_py3.pyを参考に、Python3環境で動作するtello_manager.pyを実装
  - ネットワーク内のTelloドローンを自動検出する機能を追加
  - ソケット通信によるコマンド送受信機能を実装

- メインプログラムの改良
  - tello_managerを使用してドローンを自動検出する機能を追加
  - 自動検出に失敗した場合は設定ファイルのIPアドレスを使用するフォールバック機能を実装
  - READMEを更新して、自動検出機能の説明を追加

- 追加依存パッケージ
  - netifaces: ネットワークインターフェース情報の取得
  - netaddr: IPアドレス操作ライブラリ

## 2025-05-01 11:32
- YAMLファイル関連の削除
  - cmdディレクトリとdrone_config.yamlファイルを削除
  - tello_double_control.pyからYAML関連のコード（設定ファイル読み込み、フォールバック機能）を削除
  - TELLO_devと同様に、完全に自動検出のみに依存するように変更
  - READMEを更新して、YAMLファイル関連の記述を削除

- 最終的なプロジェクト構成
  - src/KeyPressModule.py: キーボード入力検出モジュール
  - src/tello_double_control.py: メインプログラム
  - src/tello_manager.py: Telloドローン管理クラス（自動検出機能）
  - docs/work_log.md: 作業履歴
  - README.md: 使用方法の説明
