#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CSV Post-Processing Script for MOCAP Tello Data
CSVファイルのpost-process処理：正しいrole列の追加と不要カラムの削除

機能:
1. 正しいleader/follower role列を追加（プログラム内ロジックに依存しない判定）
2. 不要なカラムを削除（S列battery, T列process_time, U~X列quat, AF列battery以降）
3. CSVファイルの整理・最適化

使用方法:
python csv_postprocess.py <input_csv_file> [output_csv_file]

作成日: 2025-08-07
"""

import pandas as pd
import sys
import os
from pathlib import Path
import argparse

def determine_correct_role(drone_index, trust_metrics, leader_switching_threshold=0.6):
    """
    信頼度に基づいて正しいリーダー/フォロワー役割を判定
    
    Args:
        drone_index (int): ドローンインデックス（0または1）
        trust_metrics (list): 信頼度リスト [drone1_trust, drone2_trust]
        leader_switching_threshold (float): リーダー交代しきい値
    
    Returns:
        str: 'leader' または 'follower'
    """
    if len(trust_metrics) < 2:
        # デフォルト役割（1号機=leader, 2号機=follower）
        return 'leader' if drone_index == 0 else 'follower'
    
    trust_1, trust_2 = trust_metrics[0], trust_metrics[1]
    
    # 両方の信頼度が正常な場合：デフォルト役割
    if trust_1 >= leader_switching_threshold and trust_2 >= leader_switching_threshold:
        return 'leader' if drone_index == 0 else 'follower'
    
    # 1号機の信頼度が低い場合：2号機がリーダー
    if trust_1 < leader_switching_threshold and trust_2 >= leader_switching_threshold:
        return 'follower' if drone_index == 0 else 'leader'
    
    # 2号機の信頼度が低い場合：1号機がリーダー
    if trust_2 < leader_switching_threshold and trust_1 >= leader_switching_threshold:
        return 'leader' if drone_index == 0 else 'follower'
    
    # 両方の信頼度が低い場合：デフォルト役割を維持
    return 'leader' if drone_index == 0 else 'follower'

def get_columns_to_remove():
    """
    削除対象のカラム名リストを取得
    
    Returns:
        list: 削除対象カラム名のリスト
    """
    columns_to_remove = [
        # S列: battery（重複）
        'battery',
        # T列: process_time
        'process_time',
        # U~X列: quaternion関連
        'quat_w', 'quat_x', 'quat_y', 'quat_z',
        'quaternion_w', 'quaternion_x', 'quaternion_y', 'quaternion_z',
        # AF列以降: battery関連（重複）とその他不要カラム
        'battery_percent', 'battery_level', 'battery_voltage',
        'flight_time', 'height_raw', 'temperature',
        'wifi_signal', 'sdk_version', 'serial_number'
    ]
    return columns_to_remove

def process_csv_file(input_file, output_file=None):
    """
    CSVファイルのpost-process処理
    
    Args:
        input_file (str): 入力CSVファイルパス
        output_file (str): 出力CSVファイルパス（Noneの場合は自動生成）
    
    Returns:
        str: 出力ファイルパス
    """
    print(f"📊 CSV Post-Process開始: {input_file}")
    
    # CSVファイル読み込み
    try:
        df = pd.read_csv(input_file)
        print(f"✅ CSVファイル読み込み完了: {len(df)} 行, {len(df.columns)} 列")
    except Exception as e:
        print(f"❌ CSVファイル読み込みエラー: {e}")
        return None
    
    # 出力ファイル名を自動生成
    if output_file is None:
        input_path = Path(input_file)
        output_file = input_path.parent / f"{input_path.stem}_processed{input_path.suffix}"
    
    # 1. 正しいrole列を追加
    print("🔄 正しいrole列を追加中...")
    correct_roles = []
    
    for index, row in df.iterrows():
        try:
            # ドローンインデックスを取得
            drone_index = int(row.get('drone_index', 0))
            
            # 信頼度を取得（存在しない場合は1.0）
            trust_1 = float(row.get('trust_metric', 1.0)) if drone_index == 0 else 1.0
            trust_2 = float(row.get('trust_metric', 1.0)) if drone_index == 1 else 1.0
            
            # 他のドローンの信頼度を取得（同じタイムスタンプの行から）
            timestamp = row.get('timestamp', '')
            if timestamp:
                same_time_rows = df[df['timestamp'] == timestamp]
                if len(same_time_rows) >= 2:
                    trust_values = same_time_rows['trust_metric'].tolist()
                    if len(trust_values) >= 2:
                        trust_1, trust_2 = trust_values[0], trust_values[1]
            
            # 正しい役割を判定
            correct_role = determine_correct_role(drone_index, [trust_1, trust_2])
            correct_roles.append(correct_role)
            
        except Exception as e:
            # エラー時はデフォルト役割
            drone_index = int(row.get('drone_index', 0))
            correct_role = 'leader' if drone_index == 0 else 'follower'
            correct_roles.append(correct_role)
    
    # 正しいrole列を追加
    df['correct_role'] = correct_roles
    print(f"✅ 正しいrole列を追加: {len(correct_roles)} 行")
    
    # 2. 不要カラムを削除
    print("🗑️  不要カラムを削除中...")
    columns_to_remove = get_columns_to_remove()
    initial_columns = len(df.columns)
    
    # 存在するカラムのみ削除
    existing_columns_to_remove = [col for col in columns_to_remove if col in df.columns]
    if existing_columns_to_remove:
        df = df.drop(columns=existing_columns_to_remove)
        print(f"✅ 削除されたカラム: {existing_columns_to_remove}")
    
    final_columns = len(df.columns)
    print(f"📊 カラム数: {initial_columns} → {final_columns} ({initial_columns - final_columns} 削除)")
    
    # 3. CSVファイル保存
    try:
        df.to_csv(output_file, index=False)
        print(f"✅ 処理済みCSVファイル保存完了: {output_file}")
        print(f"📊 最終結果: {len(df)} 行, {len(df.columns)} 列")
        
        # カラム一覧表示
        print("📋 最終カラム一覧:")
        for i, col in enumerate(df.columns):
            print(f"  {i+1:2d}. {col}")
            
        return str(output_file)
        
    except Exception as e:
        print(f"❌ CSVファイル保存エラー: {e}")
        return None

def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='CSV Post-Processing for MOCAP Tello Data')
    parser.add_argument('input_file', help='Input CSV file path')
    parser.add_argument('output_file', nargs='?', help='Output CSV file path (optional)')
    parser.add_argument('--threshold', type=float, default=0.6, 
                       help='Leader switching threshold (default: 0.6)')
    
    args = parser.parse_args()
    
    # 入力ファイルの存在確認
    if not os.path.exists(args.input_file):
        print(f"❌ 入力ファイルが見つかりません: {args.input_file}")
        return 1
    
    # Post-process実行
    result = process_csv_file(args.input_file, args.output_file)
    
    if result:
        print(f"🎉 CSV Post-Process完了: {result}")
        return 0
    else:
        print("❌ CSV Post-Process失敗")
        return 1

if __name__ == "__main__":
    sys.exit(main())
