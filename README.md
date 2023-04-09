# akari_hand-tracking
AKARIでハンド追跡をするアプリです

## 機能
AKARIが認識した手を追跡します  

## Clone
- レポジトリのClone  
`git clone --recursive https://github.com/AkariGroup/akari_hand-tracking.git `

## submoduleのclone
`git submodule update --init --recursive`

## 仮想環境の作成
`python -m venv venv`  
`source venv/bin/activate`  
`pip install -r requirements.txt`

## アプリの実行
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
