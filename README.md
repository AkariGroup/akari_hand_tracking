# akari_hand-tracking
AKARIでハンド追跡をするアプリです

## 機能
AKARIが認識した手を追跡します  

## Clone
- レポジトリのClone  
`git clone --recursive https://github.com/AkariGroup/akari_hand-tracking.git `

## セットアップ
- モジュールのインストール  
`pip install -r depthai_hand_tracker/requirements.txt`

## 実行方法
1. 仮想環境を有効にする  
`poetry shell`  
2. アプリの実行  
`python3 main.py`  

## オプション
- コマンドライン引数に以下を与えることで追跡するジェスチャーを設定できます  
`python3 main.py -g <任意のジェスチャー名>`  
指定できるジェスチャー名は以下  
・OK  
・PEACE  
・FIST  
・ONE  
・TWO  
・THREE  
・FOUR  
・FIVE  
