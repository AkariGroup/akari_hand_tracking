# akari_hand-tracking
AKARIでハンド追跡をするアプリです

## 機能
AKARIが認識した手を追跡します  

## セットアップ
- モジュールのインストール  
`pip install -r requirements.txt`

## 実行方法
1. 仮想環境を有効にする  
`poetry shell`  
2. アプリの実行  
`python3 hand_tracking.py`  

## オプション
- コマンドライン引数に以下を与えることで追跡するジェスチャーを設定できます  
`python3 hand_tracking.py -g <任意のジェスチャー名>`  
指定できるジェスチャー名は以下  
・OK  
・PEACE  
・FIST  
・ONE  
・TWO  
・THREE  
・FOUR  
・FIVE  
