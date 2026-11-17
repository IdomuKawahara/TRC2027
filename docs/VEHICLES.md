# 機体プロファイル: 複数の機体で同じプログラムを使う

本番機ができるまで過去機体でコードを確認する、というように、**機体が変わっても同じプログラムを使える** ようにする仕組みです。
機体ごとに違う設定と、機体ごとのキャリブレーション結果を、機体の名前で分けて保存します。

## 1. 仕組み

```
config/
├── vehicles/
│   ├── honban/                  本番機
│   │   ├── vehicle.toml           設定 (config.py との違い。本番機は空)
│   │   └── bno055_calib.json      この機体の BNO055 キャリブレーション結果
│   └── old/                     過去機体 (コード確認用)
│       ├── vehicle.toml
│       └── bno055_calib.json
└── active_vehicle               この Pi が今どの機体か (git には入らない)
```

- 基本の設定は今までどおり `main/mission/config.py`。**本番機は config.py の値そのまま** で動きます。
- 過去機体など、ほかの機体は「config.py と違う所だけ」を `vehicle.toml` の `[overrides]` に書きます。config.py は書き換えません。
- どの機体として動くかは、次の順で決まります。
  1. 環境変数 `CANSAT_VEHICLE` (1 回だけ切り替えたいとき)
  2. `config/active_vehicle` (`tools/vehicle.py use 名前` で書かれる)
  3. どちらも無ければ **本番機 `honban`**
- `config/active_vehicle` は git に入れません。**SD カードごとの設定** です。だから `git pull` しても機体は切り替わらず、何も設定していない Pi は必ず本番機として動きます。

## 2. 使い方

```bash
cd ~/TRC2027
python3 tools/vehicle.py              # 今の機体と、使える機体の一覧
python3 tools/vehicle.py use old      # この Pi を過去機体として動かす
python3 tools/vehicle.py show         # 今の機体の設定 (config.py との違い) とキャリブレーションの有無
python3 tools/vehicle.py use honban   # 本番機に戻す
python3 tools/vehicle.py check        # 本番前の確認 (本番機の設定になっているか)
python3 tools/vehicle.py new mk3      # 新しい機体のプロファイルを作る (old をひな形にする)
```

1 回だけ別の機体として動かすとき: `CANSAT_VEHICLE=old python3 main/main.py`

### 過去機体で試すときの手順

1. `python3 tools/vehicle.py use old`
2. `config/vehicles/old/vehicle.toml` を開き、過去機体に合わせて `[overrides]` を書く (下の 3 章)。行頭の `#` を外すと有効になります
3. `python3 tools/vehicle.py show` で、書いた内容が読めているか確認 (書き間違いがあるとここでエラーになります)
4. `python3 tools/motor_test.py` で前進・後退の向きを確認
5. `python3 tools/bno055_calibrate.py` で **過去機体のキャリブレーションを取る** (保存先は自動で `config/vehicles/old/` になります)
6. `python3 tools/vehicle_measure.py` で旋回の速さを測り、表示された値を `vehicle.toml` に写す (★モータが動きます)
7. `python3 main/main.py` で走らせる。起動ログの最初に `機体: old` と試験用機体の警告が出ます

### 本番機に戻すとき (★忘れると本番で過去機体の設定のまま走ります)

```bash
python3 tools/vehicle.py use honban
python3 tools/vehicle.py check        # ✅ 本番機の設定です と出れば OK
```

## 3. 機体が違うと起きる問題と対策

| 起きる問題 | 対策 (入れてある仕組み) | 機体ごとにやること |
|---|---|---|
| **別の機体のキャリブレーション結果を使ってしまう** → 方位がずれたまま「キャリブレーション済み」として走る | 結果は機体ごとの場所に保存。ファイルに **機体名と BNO055 の固有 ID** を記録し、起動時に今の機体・今のセンサと違えば **書き込まずに** Phase3 で通常どおりキャリブレーションする | 機体ごとに `tools/bno055_calibrate.py` を 1 回 |
| **モータの配線が違う** (前進で後ろに進む・左右が逆) | ピン番号を `vehicle.toml` で変えられる | 後ろに進む側は IN1 と IN2 を入れ替え。左右が逆なら A と B の組を入れ替え。`tools/motor_test.py` で確認 |
| **旋回・走行の速さが違う** → 秒数で決めている動き (Phase3 のスピン 8 秒、Phase5 の探索 0.25 秒で約 30° など) が合わない | `tools/vehicle_measure.py` で実際に測り、その機体に合う値を表示する | 表示された値を `vehicle.toml` に写す |
| **機体が遅い** → GPS の進行方位が使われない / 進んでいるのにスタックと誤判定 | 同上 (`HEADING_COG_MIN_SPEED_MPS`, `STUCK_NOPROGRESS_SEC` の目安を出す) | 同上 |
| **センサの取り付け向きが違う** (BNO055 の向き・カメラが上下逆かどうか) | `BNO_ORIENTATION`, `CAM_ROTATE_180` を機体ごとに設定 | `tools/phase5_live.py` で右に置いた物が右と表示されるか確認 |
| **センサが付いていない** | 気圧センサ: `BMP180_REQUIRED = false` で、無くても起動する (今のミッションは気圧を使っていない)。カメラ: Phase5 は自動で超音波スキャンに切り替わる。Phase4 の見張りは `CAM_P4_ENABLE = false` で止める | 無いセンサを `vehicle.toml` に書く |
| **超音波の取り付け位置が違う** → 0m の判定距離が合わない | `GOAL_SONAR_DIST_M` を機体ごとに設定 | 先端をゴールに当てたときの超音波の値 + 2〜3cm |
| **過去機体には分離機構 (フック) が無い** → Phase2 で分離を検知できない | Phase2 のプログラムは変えていない。分離を検知できなくても 30 秒で先に進む | 待ち時間を短くしたければ `PHASE2_BACK_TIMEOUT_SEC` を過去機体だけ変える |
| **地上試験のたびに落下検知を 7 分待つ** | `FALL_TIMEOUT_SEC` などを過去機体だけ短くできる (本番機の値は変わらない) | `vehicle.toml` の 5 番の例 |
| **設定の書き間違い** (名前の打ち間違い・数値の所に文字) → 違うピンでモータを回すなどの事故 | 起動時にエラーで止める。config.py に無い名前・型の違いは受け付けない | `tools/vehicle.py show` で事前に確認 |
| **本番で過去機体の設定のまま走る** | 何も設定していない Pi は本番機。試験用の機体では起動時に警告。`tools/vehicle.py check` で確認できる。SD カードを本番機に載せ替えても、BNO055 が別物ならキャリブレーション結果は使われない | 本番前に `tools/vehicle.py check` |
| **どの機体のログか分からなくなる** | ログの最初に機体名と config.py との違いを表示。CSV のファイル名に機体名が入る (`mission_old_日時.csv`) | — |
| **過去機体に切り替えたまま自動テストを回して結果が変わる** | 自動テスト (`test/run_all.py`) は常に本番機の設定で動く | — |

### 過去機体で確認できること・できないこと

- **できる**: プログラムの流れ (フェーズの切り替え)、GPS 誘導、方位の融合、スタック検知と回避の動き、カメラの色検知、ログ
- **できない (本番機でやり直す)**: キャリブレーション、旋回・走行の速さに合わせた値、超音波の 0m 判定距離、分離機構の動作、カメラの見え方 (取り付け高さ・向き)

過去機体で調整した値は `config/vehicles/old/vehicle.toml` にしか入らないので、**本番機の設定には混ざりません**。
逆に、過去機体で「この値は機体に関係なく良かった」と分かったものは、config.py の方を書き換えてください。

## 4. vehicle.toml の書き方

```toml
[vehicle]
description = "過去機体 (コード確認用)"
role = "test"              # 本番機は "competition"、それ以外は "test"

[overrides]                # config.py と違う値だけを書く
PIN_AIN1 = 5               # 整数
CAM_ROTATE_180 = false     # true / false (小文字)
FIG8_SPIN_SEC = 12.0       # 数値
CAM_SIZE = [320, 240]      # 並びは [ ] で囲む
```

- 書ける名前は `main/mission/config.py` にあるものだけです。
- ほかの値から計算される値 (`WEAK_RATIO`, `MAG_DECLINATION` など) は直接は変えられません。元の値 (`SPEED_WEAK`, `MAG_SITE` など) を変えると自動で計算し直されます。
- 本番機 (`honban`) の `[overrides]` は空にしておき、本番機の設定は config.py に書きます。

## 5. 実機で未確認のこと

- BNO055 の固有 ID の読み取りは、偽のセンサでのテストだけで、実物では確認していません。読めなかった場合は ID の確認を飛ばして機体名だけで判断するので、動作は止まりません。起動ログの `センサ固有 ID:` で読めているか分かります。
- `tools/vehicle_measure.py` はモータを動かすため、作成時には実行していません (計算部分だけテスト済み)。
