手順書
➀8月13日の午前中に済ませてほしいこと
1.ラズパイイメイジャーを起動しSDカードに書き込みを行う
Ver2.0.10が一番最新
https://www.raspberrypi.com/software/
このURLでやればいける

PCにSDカードを入れて
ダウンロードしたイメイジャーを起動してください

ラズパイの種類を選びます
Raspberry　Pi　02Wを選択してください

OSの種類は1番上の物を選択してください

SDカードを選択してください

Hostname
Localisation（国・キーボード設定）
User
Wi-Fi
Remote access（SSHの有効化、パスワード認証で十分）
Raspberry Pi Connect（オフでよい）

次にこれらの上記の順で設定していくんだけど
ホストネームはraspberrypi
国は日本の東京
ユーザーネームはpi
Wifiは河原のスマホのやつ
次に忘れずにSSH接続の有効化を選択してください（ダブルチェック推奨）
最後にコネクトをオフに設定をして
書き込みを始めてください

書き込み終わったら
SDカードを抜いてラズパイにさして電源を入れてください
（注意：今刺さってるＳＤカードはなくさず必ず区別して持ち歩いてなくさないようにしてね）

PCのコマンドプロンプトを開き
スマホのテザリングのWIFIをつないでください

つないだらしばらく待って
コマンドに
ssh pi@raspberrypi.local
と打ち込んでください！

いろいろ出てきたらYesと打ってください

そしたら順に以下のコマンドを打ち込んで実行していってください
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y git tmux i2c-tools curl
sudo apt install -y python3-gpiozero python3-rpi-lgpio liblgpio1 python3-serial python3-smbus
sudo apt install -y python3-pip python3-setuptools
sudo apt install -y python3-picamera2 python3-libcamera libcamera-apps python3-opencv python3-numpy
sudo apt install -y libgl1
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install --upgrade pip
pip install pynmea2
deactivate

★ OpenCV (カメラでゴールの赤いコーンを見つけるのに使う) が入ったか確認してください
（上の python3-opencv で一緒に入りますが、入っていなかったことがあるので必ず確認）
python3 -c "import cv2; print(cv2.__version__)"
4.10.0 のように数字が出れば OK です。
「No module named 'cv2'」と出たら、次のコマンドで OpenCV だけ入れ直してください
（5〜10分くらいかかります。途中で止めないでください）
sudo apt install -y --no-install-recommends python3-opencv
入れ直したら、もう一度上の確認コマンドを実行して数字が出ることを確認してください。
venv の中でも使えるか確認するときは
source venv/bin/activate
python3 -c "import cv2; print(cv2.__version__)"
deactivate
（venv は --system-site-packages で作っているので、apt で入れた OpenCV がそのまま使えます）

注意
・pip install opencv-python はしないでください（ラズパイだとビルドに何時間もかかったり失敗したりします。必ず apt の python3-opencv を使う）
・OpenCV が入っていなくてもプログラムは止まらずに動きます（自動で精度の低い numpy 版に切り替わります）。
  ただしゴールの形の判定が無くなり、検知も約 3 倍遅くなるので、本番では必ず入れてください
・メモリが少ないので、apt のインストール中は他の重いプログラムを動かさないでください

次にラズパイの設定をいじります
sudo raspi-config

中に入れたら
Interface Options → I2C → Yes
と順番に選んで設定してください

次に
Interface Options → Serial Port
に入ったら
順番に
Yes　→　NO
を選んで設定してください

設定し終わったらFinishに入ってそのままrebootするか聞かれると思うのでYesを選択してください
（もしなんもなくて元の画面に戻ってきたら
sudo reboot
を実行してね）

最後に
git clone https://github.com/jumonjiakimsap2-cell/NSE2026
を実行してください
これで準備はばっちり
僕がしてたみたいに
cd NSE2026
ls
などで中のファイルを確認して下さい

プログラムを実行したいときは
source venv/bin/activate
を実効した後に
その階層のところに入り
python3 〇〇〇.pyと打って実行してね
途中で終了したいときは
コントロール＋ｃを押してください
もしファイルの内容を変えたいときは
nano 〇〇〇.pyと打って編集画面に入ってください
編集したら
コントロール＋ｓで保存，
コントロール＋ｘで編集画面から脱出してください
（編集画面の下にコマンドの説明あります）


➁僕がいないとき（鬼頭のＰＣで実行するとき）
リポですが先輩たちの方を使ってください（そっちの方が安定するため）

君たちだけで実行するときは以下の手順で実験を行ってください
まず階層toolsに入り（2026-11 の整理で main から tools に移しました）
finishv2.pyを実行してください（あえて名前はそのままにしました）
言わずもがな各種センサを動かすコードです
これで右側にある二つの値
緯度，経度を写真を撮りメモしてください

メモした緯度経度を本番用コードの設定ファイル
main/mission/config.py の TARGET_LAT と TARGET_LNG
に編集します（もうすでに先輩が図った本番ゴール周辺の座標が入れられています）
（2026-11 の整理で設定値はすべて config.py にまとめました。main.py は書き換えなくて OK）

ここで事前準備終了です

最後に
source venv/bin/activate
cd NSE2026
cd main
python3 main.py
をすればコードが実行されます
（マジで僕がいけなくてごめんなさい，もし想定外の動きをしてもごめんなさい、、、プログラマーの僕の責任や…）


➂僕のＰＣ，スマホでやる時
僕がその場にいなければ先輩のリポを使ってね（なくさないでね）

まず僕のスマホのパスワード
226515j
PCのパスワード
8128

僕のスマホのホーム画面になったら
右側の画面を上から下にスワイプし
ｗｉｆｉとかモード設定が見れる画面を出してください
歯車マークを押してください
接続を押してください
そこからテザリングをしてください

ＰＣとラズパイがつながったらあとは同じです
ですが
一回ラズパイとＰＣがつながり
プログラムを実行する画面に来たら
cd NSE2026
git stash
git pull
をしてください（プログラムの更新を適用する魔法です）

その後上記のように実験を遂行してください

もしこれを見てもわからなければもっと詳しいものが下記にあります
https://github.com/YutakaOkutani/NSE2026









━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# ファイル一覧 (どの .py が何をするか)

**本番で動かすのは `main/main.py` です**（`python3 main/main.py`）。
main.py はフェーズの切り替えだけを行い、中身は `main/mission/` に分かれています。
リポジトリ全体の地図は、いちばん上の `README.md` にもあります。
実機ができたらやることは **`main/FIELD_CHECKLIST.md`** にまとめてあります。

2026-11-11 に整理し、使わなくなった古いプログラム（main2〜6.py、test_main*.py、test_GPSrun*.py など）は削除しました。
必要になったら git の履歴から取り出せます（例: `git show 81b922f:main/main6.py > main6.py`）。

```
TRC2027/
├── main/      本番プログラム（main.py と mission/）・この README・実機チェックリスト
├── sensor/    センサのドライバと、カメラ検知・方位計算などの部品
├── tools/     実機で手で動かす道具（センサ確認・キャリブレーション・Phase5 の検知確認）
│   └── network/   SSH・Wi-Fi の試験
├── test/      自動テスト（実機不要・何も動かない）
├── docs/      作業報告と解説
└── test_images/  カメラの確認画像
```

## main/ (本番プログラム)

| ファイル | 何をするか |
|---|---|
| **`main.py`** | **本番の入口。フェーズの切り替えだけ** (Phase0 → 1 → 2 → 3 → 4 ⇄ 5) |
| **`mission/config.py`** | **設定値はすべてここ** (目標座標・会場の偏角・各フェーズの閾値・ピン番号)。実地で調整するのはこのファイルだけ |
| `mission/state.py` | フェーズ間で共有する状態 (センサ最新値 `g_*`・現在フェーズ・ログバッファ) |
| `mission/logger.py` | ログ出力・CSV 保存・LED 点滅 |
| `mission/hardware.py` | GPS 受信・超音波 (SonarSensor)・モータ (MotorController)・BNO055 の取り付け向き設定 |
| `mission/navigation.py` | 距離・方位の計算 |
| `mission/camera.py` | カメラ・色検知器の組み立て |
| `mission/phase0_init.py` | Phase0: 初期化 |
| `mission/phase1_fall.py` | Phase1: 落下検知 |
| `mission/phase2_separation.py` | Phase2: 分離 (後退→停止→前進)。**パラシュート分離に関わるので変更禁止** |
| `mission/phase3_calibration.py` | Phase3: BNO055 キャリブレーションと GPS 安定化 |
| `mission/phase4_gps.py` | Phase4: GPS 誘導走行 (方位融合・カメラでのゴール見張り) |
| `mission/stuck.py` | Phase4 のスタック検知と回避 (段差型: 後退→回転 / 轍型: ツイスト→ロッキング) |
| `mission/phase5_camera.py` | Phase5: カメラ最終接近 + 超音波で 0m 確認 |
| `README.md` | このファイル (セットアップ手順・ファイル一覧) |
| `FIELD_CHECKLIST.md` | 実機ができたらやることのチェックリスト |

## sensor/ (ドライバと部品)

| ファイル | 何をするか | 単体で動かすと |
|---|---|---|
| `BNO055.py` | 9 軸センサ (加速度・ジャイロ・地磁気) のドライバ | 値を表示 |
| `BMP180.py` | 気圧センサのドライバ | 値を表示 |
| `micropyGPS.py` | GPS の文字列 (NMEA) を読み解くライブラリ (外部のもの) | — |
| `GPS.py` | GPS の全情報を表示 | 緯度経度・衛星数などを表示 |
| `HC-SR04.py` | 超音波距離センサのドライバ | 距離を表示 |
| `goal_vision.py` | カメラでゴール (赤いコーン) を見つける処理・超音波の 0m 判定 | 画像ファイルに対して検知 |
| `heading_fusion.py` | 地磁気と GPS の進行方位を組み合わせて機体の向きを求める | — |
| `bno_calib.py` | BNO055 のキャリブレーション結果の読み書き | — |

## tools/ (実機で使う道具)

| ファイル | 何をするか | モータ |
|---|---|---|
| `finishv2.py` | 全センサの値を表示。**ゴール座標を測るとき** に使う | 動かない |
| `bno055_calibrate.py` | **本番前に** BNO055 をキャリブレーションして `config/bno055_calib.json` に保存 | 動かない |
| `phase5_live.py` | Phase5 だけを実機で試す。既定は検知だけ (方向・距離・判断を表示)。`--drive` で本物の Phase5 | 既定は動かない / `--drive` で動く |
| `motor_test.py` | モータの前進・後退の動作確認 | 動く |
| `led_test.py` | LED を 5 秒点灯 | — |
| `network/test_onoff*.py`, `rpi_blink.py`, `test_wifi.py` | SSH 接続の切り替え試験・SSH 切断中の LED 点滅・Wi-Fi 切り替えの事前試験 (PC 側で動かすものを含む。本番の main とは無関係) | — |

## test/ (自動テスト。実機不要)

全部まとめて実行: `python3 test/run_all.py` (数分。最後に一覧が出て、すべて ✅ なら OK)

| ファイル | 何をテストするか |
|---|---|
| `test_mission_flow.py` | main.py のフェーズ切り替え・偏角プリセット |
| `test_heading_fusion.py` | 方位の融合 |
| `test_goal_vision.py` | カメラの色検知・形の判定・超音波の 0m 判定 |
| `test_stuck_recovery.py` | スタック検知と回避 (段差・轍・片輪空転・空転・壁) |
| `test_phase4_sim.py` | Phase4 の走行シミュレーション・カメラでのゴール確定・キャリブレーション保存 |
| `test_phase5_sim.py` | Phase5 のシミュレーション |
| `mission_facade.py` | テスト用の部品 (テストではない) |
| `run_all.py` | 上のテストを 1 つずつ順番に全部実行する |

## docs/ (報告書・解説)

| ファイル | 内容 |
|---|---|
| `PHASE5_GUIDE.md` | Phase5 の仕組みと `tools/phase5_live.py` の使い方 |
| `REFACTOR_REPORT.md` | main6.py を mission/ に分割した内容・設定値の対応表・スタック回避などの改善 |
| `REPORT.md` | Phase5 をカメラ方式にしたときの調査・テスト結果 |
| `PROGRESS.md` | 分割・改善・整理の作業記録 |

## その他

| ファイル / フォルダ | 何か |
|---|---|
| `config/bno055_calib.json` | BNO055 のキャリブレーション結果 (`tools/bno055_calibrate.py` で作る) |
| `logs/` | 走行ログ (CSV) と撮影画像 |
| `test_images/` | カメラの動作確認画像 |
| `discord.sh` / `discord.example` | Discord への通知用 |
