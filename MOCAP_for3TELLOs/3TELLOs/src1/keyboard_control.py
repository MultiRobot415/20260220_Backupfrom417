#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
キーボード入力処理モジュール（3機対応版）

キーボードの入力を検出し、3機のドローン制御に必要な機能を提供するモジュール。

作成日: 2025-09-30
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
    
    # デバッグ情報の出力
    print("=== キーボードコントロール初期化開始 ====")
    
    # Pygameが既に初期化されている場合は再初期化しない
    if pygame.get_init():
        print("Pygameは既に初期化されています")
    else:
        pygame.init()
        print("Pygameを初期化しました")
    
    # ウィンドウを作成
    _win = pygame.display.set_mode((800, 500))
    pygame.display.set_caption("3 Tello Control - Keyboard Input Window (Click Here)")
    _win.fill((200, 200, 200))  # グレー背景
    
    # テキスト描画
    try:
        font = pygame.font.Font(None, 32)
        text1 = font.render("CLICK THIS WINDOW FOR KEYBOARD FOCUS", True, (0, 0, 255))
        text2 = font.render("Manual Control Mode for 3 Tello Drones", True, (255, 0, 0))
        text3 = font.render("Q: Takeoff | E: Land | W/S: Up/Down | A/D: Yaw", True, (0, 0, 0))
        text4 = font.render("Arrows: Movement | SPACE: Exit | ESC: Emergency", True, (0, 0, 0))
        text5 = font.render("Drone Selection: 1:Drone1 | 2:Drone2 | 3:Drone3 | 0:All", True, (0, 128, 0))
        text6 = font.render("Keyboard Status: OK if window is active", True, (255, 0, 0))
        text7 = font.render("YOU MUST CLICK THIS WINDOW", True, (128, 0, 128))
        
        _win.blit(text1, (20, 40))
        _win.blit(text7, (20, 80))
        _win.blit(text2, (20, 150))
        _win.blit(text3, (20, 200))
        _win.blit(text4, (20, 250))
        _win.blit(text5, (20, 300))
        _win.blit(text6, (20, 400))
        pygame.display.update()
        
        # ウィンドウを前面に表示
        pygame.display.flip()
    except Exception as e:
        print(f"UIレンダリングエラー: {e}")

    print("Pygameウィンドウの状態: ", pygame.display.get_active())
    print("Pygameウィンドウサイズ: ", pygame.display.get_window_size())

    # キーモニタリングスレッドの起動
    if _key_thread and _key_thread.is_alive():
        print("キーモニタリングスレッドは既に実行中です")
    else:
        _is_running = True
        _key_thread = threading.Thread(target=_key_monitoring_thread, daemon=True)
        _key_thread.start()
        print("キーモニタリングスレッドを開始しました")
        
    # 定期的にウィンドウを更新して表示し続けるスレッドを起動
    update_thread = threading.Thread(target=_window_update_thread, daemon=True)
    update_thread.start()
    print("ウィンドウ更新スレッドを開始しました")

def _key_monitoring_thread():
    """
    キーボード入力を監視し続けるスレッド関数
    """
    global _pressed_keys, _is_running
    
    if not pygame.get_init():
        print("警告: キー監視スレッドでPygameが初期化されていません")
        return
    
    print("===== キー入力監視スレッドが開始されました =====")
    print("Pygameウィンドウの状態: ", pygame.display.get_active())
    print("Pygameウィンドウサイズ: ", pygame.display.get_window_size())
    
    keys_to_check = [
        'a', 'd', 'w', 's',              # 上下移動、左右回転
        'UP', 'DOWN', 'LEFT', 'RIGHT',   # 前後、左右移動
        'q', 'e',                        # 離陸、着陸
        '1', '2', '3', '0',              # ドローン選択（1・2・3・全）
        'ESCAPE', 'SPACE'                # 緊急停止、通常停止
    ]
    
    print("キー監視スレッド開始: ", keys_to_check)
    
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
                
                # デバッグ出力は削除（パフォーマンス向上のため）
            except pygame.error:
                print("キー状態取得エラー - Pygame初期化エラーの可能性")
                local_running = False
                break
                
            # 短い待機で負荷を軽減
            time.sleep(0.05)
        except Exception as e:
            print(f"キー監視スレッドでエラー: {e}")
            time.sleep(0.2)


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
        return False
        
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
            # 通常のアルファベット・数字キーの場合
            try:
                key_code = getattr(pygame, f'K_{key_name.lower()}')
                return keys[key_code]
            except (AttributeError, KeyError):
                print(f"キー名エラー: '{key_name}'は有効なPygameキーではありません")
                return False
    except Exception as e:
        return False
        

def getKey(key_name):
    """
    指定したキーが押されているかどうかを返す
    
    Args:
        key_name: キー名 ('a', 'w', 's', 'd', 'UP', 'DOWN', 'LEFT', 'RIGHT', 'ESCAPE', 'SPACE', 'q', 'e', '1', '2', '3', '0' など)
    
    Returns:
        bool: キーが押されているかどうか
    """
    global _pressed_keys
    return key_name in _pressed_keys

def get_pressed_keys():
    """
    現在押されているキーのリストを返す
    
    Returns:
        list: 押されているキー名のリスト
    """
    global _pressed_keys
    return _pressed_keys.copy()  # 安全のためコピーを返す

def clear_events():
    """
    Pygameのイベントキューをクリアする
    """
    pygame.event.clear()

def _window_update_thread():
    """
    ウィンドウを定期的に更新して表示し続けるスレッド
    """
    global _is_running, _win
    
    print("ウィンドウ更新スレッドが開始されました")
    
    while _is_running:
        try:
            # ウィンドウが実際に見えているか確認
            if _win and pygame.get_init():
                # ウィンドウの更新・表示を維持
                pygame.display.flip()
                
                # 背景色を一定に保つ
                _win.fill((200, 200, 200))
                
                # テキストの再描画
                try:
                    font = pygame.font.Font(None, 32)
                    text1 = font.render("CLICK THIS WINDOW FOR KEYBOARD FOCUS", True, (0, 0, 255))
                    text2 = font.render("Manual Control Mode for 3 Tello Drones", True, (255, 0, 0))
                    text3 = font.render("Q: Takeoff | E: Land | W/S: Up/Down | A/D: Yaw", True, (0, 0, 0))
                    text4 = font.render("Arrows: Movement | SPACE: Exit | ESC: Emergency", True, (0, 0, 0))
                    text5 = font.render("Drone Selection: 1:Drone1 | 2:Drone2 | 3:Drone3 | 0:All", True, (0, 128, 0))
                    text6 = font.render("Keyboard Status: " + ("OK" if pygame.display.get_active() else "WINDOW NOT ACTIVE"), True, (255, 0, 0))
                    text7 = font.render("YOU MUST CLICK THIS WINDOW", True, (128, 0, 128))
                    
                    _win.blit(text1, (20, 40))
                    _win.blit(text7, (20, 80))
                    _win.blit(text2, (20, 150))
                    _win.blit(text3, (20, 200))
                    _win.blit(text4, (20, 250))
                    _win.blit(text5, (20, 300))
                    _win.blit(text6, (20, 400))
                    pygame.display.update()
                except Exception as e:
                    print(f"テキスト描画エラー: {e}")
        except Exception as e:
            print(f"ウィンドウ更新スレッドでエラー: {e}")
            
        time.sleep(0.1)  # スレッドの負荷を軽減


def quit():
    """
    Pygameを終了し、キー監視スレッドを停止する
    """
    global _key_thread, _is_running, _key_thread, _pressed_keys
    
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
    
    # Pygameを終了
    try:
        if pygame.get_init():
            pygame.quit()
            print("Pygameを正常に終了しました")
    except Exception as e:
        print(f"Pygame終了中にエラー: {e}")
        
    print("キーボード監視モジュールのシャットダウン完了")


# テスト用コード
if __name__ == "__main__":
    print("キーボード入力モジュールのテスト（3機対応版）")
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
