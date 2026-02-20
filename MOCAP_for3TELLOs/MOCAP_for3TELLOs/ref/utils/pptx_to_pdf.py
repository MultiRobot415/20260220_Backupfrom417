#!/usr/bin/env python3
"""
PowerPointファイルをPDFに変換するスクリプト
"""

import sys
import os
from pathlib import Path
import subprocess
from pptx import Presentation
import tempfile

def convert_pptx_to_pdf(pptx_path, output_dir=None):
    """
    PowerPointファイルをPDFに変換する
    
    Args:
        pptx_path (str or Path): PowerPointファイルのパス
        output_dir (str or Path, optional): 出力ディレクトリ
    """
    pptx_path = Path(pptx_path)
    
    if not pptx_path.exists():
        print(f"エラー: ファイルが見つかりません: {pptx_path}")
        return
    
    if output_dir is None:
        output_dir = pptx_path.parent
    else:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
    
    # 出力ファイル名
    pdf_file = output_dir / f"{pptx_path.stem}.pdf"
    
    try:
        # 方法1: LibreOfficeを使用する方法（インストールされている場合）
        try:
            print(f"LibreOfficeを使用してPDFに変換しています: {pptx_path}")
            result = subprocess.run(
                ["soffice", "--headless", "--convert-to", "pdf", "--outdir", str(output_dir), str(pptx_path)],
                capture_output=True,
                text=True,
                check=True
            )
            print(f"変換完了: {pdf_file}")
            return pdf_file
        except (subprocess.SubprocessError, FileNotFoundError) as e:
            print(f"LibreOfficeでの変換に失敗しました: {e}")
            
        # 方法2: unoconvを使用する方法（インストールされている場合）
        try:
            print(f"unoconvを使用してPDFに変換しています: {pptx_path}")
            result = subprocess.run(
                ["unoconv", "-f", "pdf", "-o", str(output_dir), str(pptx_path)],
                capture_output=True,
                text=True,
                check=True
            )
            print(f"変換完了: {pdf_file}")
            return pdf_file
        except (subprocess.SubprocessError, FileNotFoundError) as e:
            print(f"unoconvでの変換に失敗しました: {e}")
        
        # 方法3: 各スライドを画像として保存し、それらをPDFに結合する
        print(f"Pythonライブラリを使用してPDFに変換しています: {pptx_path}")
        prs = Presentation(pptx_path)
        
        # 一時ディレクトリを作成
        with tempfile.TemporaryDirectory() as tmpdirname:
            temp_dir = Path(tmpdirname)
            
            # 各スライドを画像として保存
            for i, slide in enumerate(prs.slides):
                # スライドの画像を保存する処理
                # （この部分は実装が複雑なため、ここでは省略）
                pass
            
            # 画像をPDFに結合する処理
            # （この部分は実装が複雑なため、ここでは省略）
            pass
        
        print(f"変換に失敗しました。LibreOfficeまたはunoconvをインストールしてください。")
        return None
        
    except Exception as e:
        print(f"エラー: {e}")
        import traceback
        traceback.print_exc()
        return None

def main():
    if len(sys.argv) < 2:
        print("使用方法: python pptx_to_pdf.py <pptx_path> [output_dir]")
        sys.exit(1)
    
    pptx_path = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None
    
    convert_pptx_to_pdf(pptx_path, output_dir)

if __name__ == "__main__":
    main()
