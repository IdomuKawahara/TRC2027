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
まず階層mainに入り
finishv2.pyを実行してください（あえて名前はそのままにしました）
言わずもがな各種センサを動かすコードです
これで右側にある二つの値
緯度，経度を写真を撮りメモしてください

メモした緯度経度を本番用コード
main.py
に編集します（もうすでに先輩が図った本番ゴール周辺の座標が入れられています）

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

**本番で動かすのは `main/main.py` です** (2026-11-10〜。`python3 main/main.py`)。
main.py はフェーズの切り替えだけを行い、中身は `main/mission/` パッケージに分かれています
(main6.py を分割したもの。詳しくは `REFACTOR_REPORT.md`)。
実機ができたらやることは **`main/FIELD_CHECKLIST.md`** にまとめてあります。

## main/ (本番プログラム)

| ファイル | 何をするか |
|---|---|
| **`main.py`** | **本番の入口。フェーズの切り替えだけ** (Phase0 → 1 → 2 → 3 → 4 ⇄ 5)。中身は mission/ |
| **`mission/config.py`** | **設定値はすべてここ** (目標座標・偏角プリセット・各フェーズの閾値・ピン番号)。実地で調整するのはこのファイルだけ |
| `mission/state.py` | フェーズ間で共有する状態 (センサ最新値 `g_*`・現在フェーズ・ログバッファ) |
| `mission/logger.py` | ログ出力・CSV 保存・LED 点滅 |
| `mission/hardware.py` | GPS 受信スレッド・超音波 (SonarSensor)・モータ (MotorController, ピボット旋回含む)・BNO055 の Axis Remap |
| `mission/navigation.py` | 距離・方位の計算 |
| `mission/camera.py` | カメラ・色検知器・0m 確認設定の組み立て |
| `mission/phase0_init.py` | Phase0: 初期化 (キャリブレーションプロファイルの書き戻し含む) |
| `mission/phase1_fall.py` | Phase1: 落下検知 |
| `mission/phase2_separation.py` | Phase2: 分離 (後退→停止→前進)。**パラシュート分離に関わるので変更禁止** |
| `mission/phase3_calibration.py` | Phase3: BNO055 キャリブレーションと GPS 安定化 |
| `mission/phase4_gps.py` | Phase4: GPS 誘導走行 (方位融合・カメラ見張り)。スタック処理は stuck.py |
| **`mission/stuck.py`** | **スタック検知 (加速度/超音波/GPS 無進行/片輪空転) と回避 (段差型: 後退→回転、轍型: ツイスト→ロッキング)** |
| `mission/phase5_camera.py` | Phase5: カメラ最終接近 + 超音波 0m 確認 (超音波スキャンの予備も) |
| `main6.py` | 分割前の最新版 (= main.py と同じ動作 + 改善前)。参照用。`main6_before_refactor.py` はそのバックアップ |
| `main2.py` | main6.py の 1 つ前の版 (方位の融合などは無い)。`main2_before_refactor.py` はそのバックアップ |
| `main2.py.bak` | (ローカルのみ) Phase5 をカメラ方式にする前の main2.py |
| `main3.py`, `main4.py`, `main5.py` | 過去の版 (Phase5 が超音波センサ方式の頃のもの)。中身の細かい違いは未整理 |
| `finishv2.py` | 全センサ (BNO055・BMP180・GPS) の値をリアルタイム表示。**ゴール座標を測るとき** に使う (落下中の終端速度の測定にも) |
| `runtest.py` | モータの前進・後退の動作確認 (test/test_run.py と同じ) |
| `README.md` | このファイル (セットアップ手順・ファイル一覧) |
| `FIELD_CHECKLIST.md` | 実機ができたらやることのチェックリスト |
| `REFACTOR_REPORT.md` (リポジトリ直下) | 分割の内容・設定値の対応表・改善点・テスト結果 |
| `PROGRESS.md` (リポジトリ直下) | 分割/改善作業の進捗メモ |

## sensor/ (センサのドライバ・部品)

| ファイル | 何をするか |
|---|---|
| `BNO055.py` | 9 軸センサ BNO055 (加速度・ジャイロ・地磁気) のドライバ。単体で動かすと値を表示 |
| `BMP180.py` | 気圧センサ BMP180 のドライバ |
| `micropyGPS.py` | GPS の文字列 (NMEA) を読み解くライブラリ (外部のもの) |
| `GPS.py` / `GPS2.py` | GPS の全情報を表示するプログラム (単体で動く) |
| `GPStest.py` | GPS の簡単な動作確認 |
| `HC-SR04.py` | 超音波距離センサのドライバ。単体で動かすと距離を表示 |
| **`goal_vision.py`** | **カメラでゴール (赤いコーン) を見つける処理。** 色検知 (OpenCV 版 / OpenCV が無いときの numpy 版)・形の判定・カメラの撮影・超音波での 0m 判定・Phase4 用の見張り (ConeWatcher / ConeLock)。単体で動かすと画像ファイルに対して検知 |
| **`heading_fusion.py`** | **機体の向きを、地磁気と GPS の進行方位 (COG) を組み合わせて求める** (main6.py の Phase4 で使用) |
| **`bno_calib.py`** | **BNO055 のキャリブレーション結果の読み書き・ファイル保存** (main6.py の Phase0/Phase3 と test/bno055_calibrate.py で使用) |
| `test_fall2.py` | 落下検知 → サブキャリア脱出 → 前進 の試験 (test/test_fall2.py と同じ) |
| `test_avoid.py` | モータで 10 秒前進する試験 (test/test_avoid.py と同じ) |
| `motor_thread.py` | 空ファイル |

## test/ (試験用プログラム)

### カメラ・Phase5 (今回追加)
| ファイル | 何をするか | モータ |
|---|---|---|
| **`test_phase5_live.py`** | **Phase5 だけを実機で試す。** 既定はカメラ (+超音波) で検知だけして、ゴールの方向・推定距離・「Phase5 なら次に何をするか」を表示。`--drive` で本物の Phase5 (mission パッケージ) を実行。使い方は `README_phase5_test.md` | 既定は動かない / `--drive` で動く |
| `mission_facade.py` | テスト用の部品。分割後の mission パッケージを、旧 main6 と同じ「M.名前」で扱えるようにする | |
| **`bno055_calibrate.py`** | **本番前に BNO055 をキャリブレーションして `config/bno055_calib.json` に保存** | 動かない |
| `README_phase5_test.md` | Phase5 の仕組みと test_phase5_live.py の使い方の解説 | |

### ソフトだけのテスト (実機不要・何も動かない)
| ファイル | 何をテストするか |
|---|---|
| `test_goal_vision.py` | 色検知・形の判定・超音波の 0m 判定 (51 件) |
| `test_phase5_camera.py` | Phase5 全体を偽のカメラ・モータ・超音波で動かすシミュレーション (21 件) |
| `test_heading_fusion.py` | 地磁気 + GPS COG の方位の融合 (9 件) |
| `test_main6_improvements.py` | main6.py の改修点: キャリブレーションの保存・Axis Remap・Phase4 の走行シミュレーション・カメラでのゴール確定 (21 件) |
| **`test_mission_equivalence.py`** | 上の 2 つのシミュレーション (42 件) を、本文を変えずに mission パッケージへ向けて実行 (分割前後で動作が同じことの確認) |
| **`test_mission_flow.py`** | main.py のフェーズ切り替え (状態遷移) をモックで確認 + 偏角プリセット (16 件) |
| **`test_stuck_recovery.py`** | スタック検知と回避動作のシミュレーション (段差・轍・片輪空転・空転・壁) (19 件) |

### 以前からある試験プログラム
| ファイル | 何をするか | モータ |
|---|---|---|
| `test_run.py` | 前進・後退の動作確認 | 動く |
| `test_go.py` | 60 秒前進 | 動く |
| `test_straight.py` / `test_avoid.py` | 10 秒前進 (まっすぐ走るかの確認) | 動く |
| `test_LED.py` | LED を 5 秒点灯 | |
| `test_GPS.py` | GPS の全情報を表示 | |
| `test_GPSrun.py` 〜 `test_GPSrun8.py` | GPS 誘導走行の試験 (版ちがい。`test_GPSrun6.py` は GPS + 地磁気併用) | 動く |
| `guided_navigation.py` | GPS + BNO055 での誘導走行 (Phase4 相当) | 動く |
| `test_fall.py` / `test_fall2.py` | 落下検知 → サブキャリア脱出 → 前進 | 動く |
| `test_finishv.py` / `test_finishv2.py` | 全センサの値の表示・落下中の終端速度の測定 (main/finishv2.py と同じ) | |
| `test_main.py`, `test_main2.py` 〜 `test_main15.py` | 過去のミッションプログラムの試作版 (main6.py とは別物) | 動く |
| `test_onoff.py` 〜 `test_onoff5.py` | SSH 接続のオン/オフ切り替え試験 (PC 側で動かすものを含む) | |
| `rpi_blink.py` | SSH が切れている間 LED を点滅させ、SSH を自動で復活させる (ラズパイ側) | |
| `test_wifi.py` | 別の Wi-Fi に切り替えても問題ないかの事前テスト | |
| `test_camera.py`, `test_endtoend.py` | 空ファイル | |

## その他

| ファイル / フォルダ | 何か |
|---|---|
| `REPORT.md` (リポジトリ直下) | Phase5 カメラ化・OpenCV・main6.py の作業報告 (調査内容・テスト結果・未検証の項目) |
| `config/bno055_calib.json` | BNO055 のキャリブレーション結果 (bno055_calibrate.py で作る) |
| `logs/` | 走行ログ (CSV) と撮影画像 |
| `test_images/` | カメラの動作確認画像 |
| `discord.sh` / `discord.example` | Discord への通知用 |
