# 機体のセットアップと切り替え 手順書

過去機体でプログラムを確かめ、本番機ができたら本番機をセットアップする、という流れの手順書です。
上から順に進めれば、迷わずできるように書いています。

- 仕組みの詳しい説明 (書き間違いの検査など): [`VEHICLES.md`](VEHICLES.md)
- 実機での確認項目の一覧: [`../main/FIELD_CHECKLIST.md`](../main/FIELD_CHECKLIST.md)
- ラズパイを一から用意する手順 (OS・ライブラリ): [`../main/README.md`](../main/README.md)

コマンドはすべて `cd ~/TRC2027` してから実行します。
**★モータが動きます★** と書いた手順は、機体を床に置き、周りに人や物がないことを確認してから行ってください。止めるときは Ctrl+C です。

---

## 0. 基本の考え方

| | 過去機体 (`old`) | 本番機 (`honban`) |
|---|---|---|
| **使う目的** | **プログラムが正しく動くかを確かめる** | **大会で走る。値を調整する** |
| 値の調整 | しない (動かすのに最低限必要な所だけ変える) | する (走行試験で決める) |
| 値を書く場所 | `config.py` の一番下の `if VEHICLE == "old":` の中 | `config.py` の上の方 (本番機の値) |
| キャリブレーション結果 | `config/bno055_calib_old.json` | `config/bno055_calib_honban.json` |

- 過去機体で調整した値は、本番機には持ち越しません。重さ・タイヤ・モータが違うので、本番機では本番機で決め直します。
- 過去機体で値を変えるのは「このままだと動かない・確かめられない」所だけにします (配線・センサの有無・試験を短くする設定など)。
- 過去機体での確認で「プログラムの間違い」が見つかったら、それは機体に関係なくプログラムを直します。

---

## 1. 機体の切り替え方

切り替えは **`main/mission/config.py` の一番上の 1 行** だけです。

```python
VEHICLE = "honban"    # 本番機
VEHICLE = "old"       # 過去機体
```

### 1-1. 今どちらになっているか確かめる

```bash
grep '^VEHICLE =' main/mission/config.py
```

`VEHICLE = "honban"` または `VEHICLE = "old"` と表示されます。
`main.py` を動かしたときも、起動ログの最初に `機体: old` のように表示されます。

### 1-2. 切り替える

1. `nano main/mission/config.py` で開く (VS Code などで開いても構いません)
2. 一番上の方にある `VEHICLE = "honban"` を `VEHICLE = "old"` に書き換える (戻すときは逆)
3. 保存する (nano なら Ctrl+O → Enter → Ctrl+X)
4. `grep '^VEHICLE =' main/mission/config.py` で確かめる

### 1-3. 切り替えるときの注意

- **過去機体で試したあとは、必ず `"honban"` に戻す。** 戻し忘れると本番で過去機体の設定のまま走ります。
  戻し忘れていると、起動ログに `★★★ これは【試験用の機体】の設定です ★★★` と出ます。
- `"old"` のまま git に push しないこと。自動テスト (`python3 test/run_all.py`) を回すと、`"honban"` になっていなければ ❌ になるので気づけます。
- `"olld"` のように打ち間違えると、起動時に「知らない機体です」とエラーで止まります (間違った設定で走ることはありません)。

---

## 2. 過去機体のセットアップ (プログラムを確かめるため)

### 2-1. 切り替える

`config.py` の一番上を `VEHICLE = "old"` にします (1-2 の手順)。

### 2-2. 過去機体で違う所を書く

`config.py` の一番下に、次のような所があります。例が `#` 付きで書いてあるので、**過去機体で本番機と違う所だけ** 行頭の `#` を外して値を直します。

```python
if VEHICLE == "old":
    # ── 1. 配線 (BCM ピン番号) ──
    # PIN_AIN1 = 6
    ...
    pass
```

書く必要があるのは、次の 3 種類だけです。

| 種類 | 例 | 書かないとどうなるか |
|---|---|---|
| **配線が違う** | `PIN_AIN1 = 5` | モータが逆に回る・違うピンが動く |
| **付いていないセンサ・付け方が違うセンサ** | `BMP180_REQUIRED = False` (気圧センサなし)、`CAM_ROTATE_180 = False` (カメラを正立で付けている)、`CAM_P4_ENABLE = False` (カメラなし)、`BNO_ORIENTATION = 2` | 起動できない・左右が逆になる |
| **試験を短くする** | `FALL_TIMEOUT_SEC = 20` (落下検知の待ちを 7 分 → 20 秒) | 毎回 7 分待つことになる |

書き方の注意:

- 行頭は **4 文字下げる** (例の行と同じ位置から書く)。下げ忘れると起動時にエラーで止まります。
- `True` / `False` は先頭を大文字にする。
- 最後の `pass` は消さない。
- 書き間違い (無い名前・型の違い) は起動時にエラーで止まり、何が違うか表示されます。

走り方の値 (スピンの秒数・パルスの長さなど) は、**過去機体では調整しません**。
ただし、過去機体が遅すぎて Phase3 のスピンで 1 回転できない、など「確かめられない」場合だけ、`tools/vehicle_measure.py` (2-5) の値を使います。

### 2-3. 書いた内容を確かめる (モータは動かない)

```bash
cd ~/TRC2027/main
python3 -c "from mission import config as C; print(C.VEHICLE); [print(' ', k, '=', n, ' (本番機:', o, ')') for k, o, n in C.VEHICLE_OVERRIDES]"
cd ~/TRC2027
```

`old` と、変えた値の一覧が出れば OK です。書き間違いがあると、ここでエラーの文が出ます。

### 2-4. センサとモータの向きを確かめる

`main/FIELD_CHECKLIST.md` の 0 章・1 章 (センサの確認) を行います。

モータの向きは `python3 tools/motor_test.py` で確かめます (★モータが動きます)。

> **注意**: `tools/motor_test.py` は、config.py ではなく **ファイルの中に書いたピン番号** (本番機と同じ配線) を使います。
> 過去機体の配線が本番機と違う場合、`motor_test.py` の結果は config.py に書いたピンとは一致しません。
> その場合は `python3 tools/vehicle_measure.py` (config.py のピンを使い、左右に旋回する) で、
> 「左旋回」で左に、「右旋回」で右に回るかを確かめてください。

### 2-5. キャリブレーション (モータは動かない)

```bash
python3 tools/bno055_calibrate.py
```

画面の指示に従って機体を動かし、4 つとも 3 になると `config/bno055_calib_old.json` に自動で保存されます。
くわしいやり方は `main/FIELD_CHECKLIST.md` の 2 章を見てください。

### 2-6. (必要なときだけ) 速さを測る ★モータが動きます★

過去機体が遅すぎて確かめられない動きがあるときだけ行います。

```bash
python3 tools/vehicle_measure.py               # 旋回だけ (屋内でも可)
python3 tools/vehicle_measure.py --forward 5   # 前進の速さも (屋外・GPS が必要)
```

最後に「今の値 → 目安の値」の一覧が出て、`この値を config.py に書き込みますか？ (yes/no)` と聞かれます。
`yes` と入力したときだけ `if VEHICLE == "old":` の中に書き込まれます (それ以外は何も変わりません)。

### 2-7. プログラムが正しく動くかを確かめる

`main/FIELD_CHECKLIST.md` の 3〜8 章に沿って行います。過去機体で見るのは「値が合っているか」ではなく「**プログラムが考えたとおりに動くか**」です。

| 確かめること | 方法 | 合格の目安 |
|---|---|---|
| 起動してエラーで止まらない | `python3 main/main.py` | 起動ログに `機体: old` と変えた値の一覧が出て、Phase0 → 1 と進む |
| フェーズが順に切り替わる | 通しで走らせる (FIELD_CHECKLIST 8 章) | Phase1 → 2 → 3 → 4 → 5 → 完了 と進む |
| キャリブレーション結果の扱い | 起動ログ | `キャリブレーションプロファイルを書き戻しました` と出る。`センサ固有 ID:` に 32 文字の英数字が出る (`読めません` でも動作は続く) |
| GPS 誘導 | FIELD_CHECKLIST 7 章 | ゴールの方へ向きを変えて進む (速さ・曲がり方の細かさは見なくてよい) |
| スタック検知と回避 | FIELD_CHECKLIST 7 章 | 止めると「スタック」と判定され、回避動作をして誘導に戻る |
| カメラの検知・Phase5 | FIELD_CHECKLIST 3・6 章 | コーンを見つけて近づき、超音波で止まる |
| ログ | `ls logs/` | `mission_old_日時.csv` ができていて、中に各フェーズの記録がある |

見つかった問題は、次のどちらかに分けて記録してください。

- **プログラムの問題** (フェーズが進まない・エラーで止まる・左右を間違える など) → プログラムを直す。本番機にも効く
- **過去機体だけの問題** (遅い・滑る・センサが無い など) → 本番機では起きないので、プログラムは直さない

### 2-8. 終わったら本番機に戻す

1. `config.py` の一番上を `VEHICLE = "honban"` に戻す
2. `grep '^VEHICLE =' main/mission/config.py` で確かめる
3. 一番下の `if VEHICLE == "old":` の中は、消さずに残しておいて構いません (本番機では使われません。次に過去機体で試すときにそのまま使えます)

---

## 3. 本番機のセットアップ (本番機ができたら)

### 3-1. 本番機の設定にする

```bash
grep '^VEHICLE =' main/mission/config.py      # VEHICLE = "honban" になっていること
```

本番機の値は `config.py` の上の方にあります。一番下の `if VEHICLE == "old":` の中は本番機では使われません。

### 3-2. 配線・センサの確認 (モータは動かない)

`main/FIELD_CHECKLIST.md` の 0 章・1 章。配線が config.py のピン番号 (`PIN_PWMA = 13` など) と違っていたら、**config.py の上の方の値** を直します。

### 3-3. モータの向き ★モータが動きます★

```bash
python3 tools/motor_test.py
```

前進で前に進むか確かめます。後ろに進む側は、config.py の IN1 と IN2 (`PIN_AIN1` と `PIN_AIN2`、または `PIN_BIN1` と `PIN_BIN2`) を入れ替えます。

### 3-4. キャリブレーション (モータは動かない・本番前に必須)

```bash
python3 tools/bno055_calibrate.py
```

`config/bno055_calib_honban.json` に保存されます。過去機体の結果 (`bno055_calib_old.json`) は、本番機では使われません。
機体の部品 (電池・モータ・配線) の位置を変えたら、やり直してください。

### 3-5. 速さを測って値を合わせる ★モータが動きます★

本番と同じような地面 (芝生・土など) で、電池を満充電にして行います。

```bash
python3 tools/vehicle_measure.py --forward 5
```

「今の値 → 目安の値」の一覧が出ます。一覧の上に `config.py の上の方の【本番機の値】を書き換えます` と出ていることを確かめてから、`yes` と入力します。

書き換わる値: `FIG8_SPIN_SEC` (Phase3 のスピン)、`CAM_SEARCH_TURN_SEC` / `CAM_SEARCH_PULSES_PER_LAP` / `CAM_SEARCH_MAX_PULSES_PER_LAP` (Phase5 の探索)、`STUCK_SPIN_GYRO_DPS` (片輪空転の検知)、`HEADING_COG_MIN_SPEED_MPS` (GPS 方位を使う速さ)、遅い機体では `STUCK_RECOVER_TURN_TIMEOUT_SEC` / `STUCK_NOPROGRESS_SEC` も。

変わった所は `git diff main/mission/config.py` で見られます。元に戻すには `git checkout main/mission/config.py`。

### 3-6. 走行試験で残りの値を調整する ★モータが動きます★

`main/FIELD_CHECKLIST.md` の 3〜8 章に沿って行います。`vehicle_measure.py` では決まらない値は、ここで実際に走らせて決めます。

| 値 | 決め方 |
|---|---|
| `GOAL_SONAR_DIST_M` | 機体の先端をゴールに当てたときの超音波の値 + 2〜3cm (FIELD_CHECKLIST 4 章) |
| `CAM_FWD_FAR_SEC` / `CAM_FWD_NEAR_SEC` | Phase5 の 1 回の前進。ゴールに近づくのが遅すぎず、行き過ぎない長さ (6 章) |
| `CAM_TURN_GAIN_SEC` / `CAM_TURN_MAX_SEC` | Phase5 でゴールを正面に向ける旋回。左右に振れるなら短く、なかなか向かないなら長く (6 章) |
| `STUCK_TWIST_PULSE_SEC` / `STUCK_ROCK_SEC` | スタック回避の強さ (7 章) |
| `PHASE2_FWD_AFTER_SEC` | 分離後にパラシュートから十分離れられる秒数 (8 章の通し試験で見る) |
| `TARGET_LAT` / `TARGET_LNG` | ゴール地点で測る (5 章) |

調整した値は、FIELD_CHECKLIST のメモ欄にも書き残してください。

### 3-7. 調整した値を保存する

```bash
git add main/mission/config.py config/bno055_calib_honban.json
git commit -m "本番機の値を調整"
git push
```

キャリブレーション結果もコミットしておくと、SD カードを作り直しても使えます。

---

## 4. 本番前の最終確認

| ☐ | 確認すること | 方法 |
|---|---|---|
| ☐ | 本番機の設定になっている | `grep '^VEHICLE =' main/mission/config.py` が `VEHICLE = "honban"` |
| ☐ | 自動テストが通る | `python3 test/run_all.py` がすべて ✅ |
| ☐ | 本番機のキャリブレーション結果がある | `python3 tools/bno055_calibrate.py --show` で値が表示される |
| ☐ | 起動ログが正しい | `機体: honban` と出て、「試験用の機体」の警告が出ない。`キャリブレーションプロファイルを書き戻しました` と出る |
| ☐ | 目標座標・会場 | 起動ログの `目標座標` と `会場 (地磁気偏角)` が本番の会場になっている |

---

## 5. 困ったとき

| 起きたこと | 原因と直し方 |
|---|---|
| 起動すると `... は知らない機体です` | `VEHICLE` の名前の打ち間違い。`"honban"` か `"old"` にする |
| `... は config.py にありません (名前の打ち間違い?)  もしかして: ...` | `if VEHICLE == "old":` の中の名前の打ち間違い。「もしかして」の名前に直す |
| `... の型が違います` | `True` の所に `0`、整数の所に `5.0` などを書いた。本番機の値と同じ形で書く |
| `本番機なのに、config.py の一番下で値が変わっています` | `if` の中の行頭の 4 文字下げを忘れた。4 文字下げる |
| `IndentationError` | 行頭の下げ方がそろっていない。例の行と同じ位置から書く |
| `別の機体 (...) のキャリブレーション結果です` / `別の BNO055 の...` | 別の機体・センサで取った結果。今の機体で `tools/bno055_calibrate.py` をやり直す (それまでは Phase3 で通常どおりキャリブレーションするので、走行はできる) |
| `BMP180 初期化失敗` で止まる | 気圧センサが無い・配線不良。過去機体で付いていないなら `BMP180_REQUIRED = False` |
| 起動ログに「試験用の機体」の警告が出る | `VEHICLE = "old"` のまま。本番機なら `"honban"` に戻す |
| `vehicle_measure.py` で `書き込んだ config.py が読み込めなかったので元に戻しました` | 測った値が config.py の検査を通らなかった。config.py は元のままなので、表示された理由を見て手で直すか、測り直す |
| 自動テストの `test_vehicle.py` が ❌ (`VEHICLE が "honban" になっていません`) | `VEHICLE = "old"` のまま。テストの前に `"honban"` に戻す |
