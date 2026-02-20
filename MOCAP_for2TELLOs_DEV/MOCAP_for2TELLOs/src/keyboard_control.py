#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
キーボード入力処理モジュール

キーボードの入力を検出し、ドローン制御に必要な機能を提供するモジュール。
ref/TELLO_double_protoとref/MOCAP_for_TELLOのKeyPressModule.pyを参考に実装しています。

作成日: 2025-06-26
更新日: 2025-07-11 (スレッド対応強化)
"""

import pygame
import os
import threading
import time

# 現在押されているキーの状態を保持するグローバル変数
_pressed_keys = []
_is_running = True
_key_thread = None

# Pygameのウィンドウインスタンスをグローバルに保持
_win = None

def init():
    """
    Pygameを初期化し、キーボード入力の検出準備をする
    小さいウィンドウを作成してキーボード入力を処理する
    """
    global _key_thread, _is_running, _win
    
    # Pygameが既に初期化されている場合は再初期化しない
    if pygame.get_init():
        print("Pygameは既に初期化されています")
    else:
        pygame.init()
        print("Pygameを初期化しました")
    
    # ウィンドウを作成
    _win = pygame.display.set_mode((400, 400))
    pygame.display.set_caption("Tello Control - キーボード入力ウィンドウ")
    _win.fill((200, 200, 200))
    
    # テキスト描画
    try:
        font = pygame.font.Font(None, 24)
        text1 = font.render("Focus this window", True, (0, 0, 0))
        text2 = font.render("Q: Takeoff | E: Land", True, (0, 0, 0))
        text3 = font.render("W/S: Up/Down | A/D: Yaw", True, (0, 0, 0))
        text4 = font.render("Arrows: Fwd/Back/Left/Right", True, (0, 0, 0))
        text5 = font.render("ESC: Emergency Stop", True, (0, 0, 0))
        text6 = font.render("T: Auto mode | M: Manual mode", True, (0, 0, 0))
        _win.blit(text1, (50, 50))
        _win.blit(text2, (50, 100))
        _win.blit(text3, (50, 150))
        _win.blit(text4, (50, 200))
        _win.blit(text5, (50, 250))
        _win.blit(text6, (50, 300))
        pygame.display.update()
    except Exception as e:
        print(f"ウィンドウ描画エラー: {e}")
    
    # キー入力監視用のスレッドを開始（既存のスレッドがあれば先に終了）
    if _key_thread and _key_thread.is_alive():
        _is_running = False
        _key_thread.join(1.0)
        print("既存のキーボード監視スレッドを停止しました")
    
    _is_running = True
    _key_thread = threading.Thread(target=_key_monitoring_thread)
    _key_thread.daemon = True
    _key_thread.start()
    print("キーボード入力監視スレッドを開始しました")

def _key_monitoring_thread():
    """
    キーボード入力を監視し続けるスレッド関数
    メインループがブロックされていても動作し続ける
    """
    global _pressed_keys, _is_running
    
    # メインスレッドと同じPygame環境にアクセスするための安全対策
    if not pygame.get_init():
        print("警告: キー監視スレッドでPygameが初期化されていません")
        return
    
    keys_to_check = [
        'a', 'd', 'w', 's',              # 上下移動、左右回転
        'UP', 'DOWN', 'LEFT', 'RIGHT',   # 前後、左右移動
        'q', 'e',                        # 離陸、着陸
        't', 'm',                        # 自動ホバリング切替, 手動モード切替
        'z',                             # 目標位置リセット
        '1', '2', '0',                   # ドローン選択（1・2・全）
        'ESCAPE', 'SPACE'                # 緊急停止、通常停止
    ]
    
    print("キー監視スレッド開始: ", keys_to_check)
    
    # メインスレッドがPygameをシャットダウンしたかどうかのフラグ
    local_running = True
    
    while _is_running and local_running:
        try:
            # Pygameが正常に初期化されているか確認
            if not pygame.get_init():
                print("Pygameが初期化されていないため、キー監視スレッドを終了します")
                local_running = False
                break
                
            # イベントを処理
            try:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        local_running = False
            except pygame.error:
                print("Pygameイベント取得エラー - 初期化されていない可能性があります")
                local_running = False
                break
            
            # 現在押されているキーを検出
            try:
                current_pressed = []
                keys = pygame.key.get_pressed()
                
                for key in keys_to_check:
                    if _is_key_pressed(key, keys):
                        current_pressed.append(key)
                
                # グローバル変数を更新
                _pressed_keys = current_pressed
                
                # デバッグ出力はパフォーマンス向上のためコメントアウト
                # if current_pressed:
                #     print(f"キー検出 (スレッド内): {current_pressed}")
                pass  # キー入力があってもログ出力しない
            except pygame.error:
                print("キー状態取得エラー - Pygame初期化エラーの可能性")
                local_running = False
                break
                
            # 短い待機で負荷を軽減
            time.sleep(0.05)
        except Exception as e:
            print(f"キー監視スレッドでエラー: {e}")
            time.sleep(0.2)  # エラー時は少し長めに待機


def _is_key_pressed(key_name, keys):
    """
    与えられたキー配列で特定のキーが押されているか判定
    
    Args:
        key_name: キー名
        keys: pygame.key.get_pressed()の戻り値
        
    Returns:
        bool: キーが押されていればTrue
    """
    if not pygame.get_init():
        return False  # Pygameが初期化されていない場合はFalseを返す
        
    try:
        if key_name == 'LEFT':
            return keys[pygame.K_LEFT]
        elif key_name == 'RIGHT':
            return keys[pygame.K_RIGHT]
        elif key_name == 'UP':
            return keys[pygame.K_UP]
        elif key_name == 'DOWN':
            return keys[pygame.K_DOWN]
        elif key_name == 'ESCAPE':
            return keys[pygame.K_ESCAPE]
        elif key_name == 'SPACE':
            return keys[pygame.K_SPACE]
        else:
            # 通常のアルファベットキーの場合
            try:
                key_code = getattr(pygame, f'K_{key_name.lower()}')
                return keys[key_code]
            except (AttributeError, KeyError):
                print(f"キー名エラー: '{key_name}'は有効なPygameキーではありません")
                return False
    except Exception as e:
        # エラーをキャッチしてFalseを返すが、詳細なエラーログは出さない
        # （頻繁なエラーログを避けるため）
        return False
        

def getKey(key_name):
    """
    指定したキーが押されているかどうかを返す
    スレッド化された新実装版 - グローバルの状態を参照
    
    Args:
        key_name: キー名 ('a', 'w', 's', 'd', 'UP', 'DOWN', 'LEFT', 'RIGHT', 'ESCAPE', 'SPACE', 'q', 'e' など)
    
    Returns:
        bool: キーが押されているかどうか
    """
    global _pressed_keys
    return key_name in _pressed_keys

def get_pressed_keys():
    """
    現在押されているキーのリストを返す
    スレッド化された新実装版 - グローバル変数から直接取得
    
    Returns:
        list: 押されているキー名のリスト
    """
    global _pressed_keys
    
    # デバッグ: 取得中に表示
    if _pressed_keys:
        print(f"キー検出 (メインスレッドから): {_pressed_keys}")
        
    return _pressed_keys.copy()  # 安全のためコピーを返す

def clear_events():
    """
    Pygameのイベントキューをクリアする
    """
    pygame.event.clear()

def quit():
    """
    Pygameを終了し、キー監視スレッドを停止する
    """
    global _is_running, _key_thread, _pressed_keys
    
    print("キーボード監視モジュールのシャットダウンを開始")
    
    # スレッドを停止する前に状態をリセット
    _pressed_keys = []
    
    # スレッドを停止
    _is_running = False
    if _key_thread and _key_thread.is_alive():
        try:
            _key_thread.join(1.0)  # 最大1秒待機
            print("キーボード監視スレッドを停止しました")
        except Exception as e:
            print(f"スレッド停止中にエラー: {e}")
    
    # Pygameを終了（既に終了している場合はスキップ）
    try:
        if pygame.get_init():
            pygame.quit()
            print("Pygameを正常に終了しました")
    except Exception as e:
        print(f"Pygame終了中にエラー: {e}")
        
    print("キーボード監視モジュールのシャットダウン完了")


# テスト用コード
if __name__ == "__main__":
    print("キーボード入力モジュールのテスト")
    print("----------------------------")
    print("何かキーを押してください。終了するにはESCキーを押してください。")
    
    # Pygameを初期化
    init()
    
    # キー入力をテスト
    running = True
    while running:
        # 押されているキーを取得して表示
        pressed_keys = get_pressed_keys()
        if pressed_keys:
            print(f"押されているキー: {', '.join(pressed_keys)}")
        
        # ESCキーで終了
        if getKey('ESCAPE'):
            running = False
        
        # 一定時間待機
        pygame.time.delay(100)
    
    # Pygameを終了
    quit()
    print("テスト完了")
