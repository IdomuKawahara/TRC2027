# TRC2027 CanSat

地上走行型 CanSat のプログラム。落下 → パラシュート分離 → キャリブレーション → GPS 誘導 → カメラで 0m ゴール。

## まずどこを見ればいいか

| やりたいこと | 見る場所 |
|---|---|
| ラズパイを一からセットアップしたい | [`main/README.md`](main/README.md) の手順書 |
| 本番を動かしたい | `python3 main/main.py` |
| 設定 (目標座標・会場・閾値) を変えたい | [`main/mission/config.py`](main/mission/config.py) だけ |
| 実機でやることを確認したい | [`main/FIELD_CHECKLIST.md`](main/FIELD_CHECKLIST.md) |
| どの .py が何をするか知りたい | [`main/README.md`](main/README.md) の「ファイル一覧」 |
| プログラムを変えたあとに壊れていないか確かめたい | `python3 test/run_all.py` (実機不要・数分) |

## フォルダ構成

```
TRC2027/
├── main/
│   ├── main.py          本番の入口 (フェーズの切り替えだけ)
│   ├── mission/         本番プログラムの中身
│   │   ├── config.py      ★設定値はすべてここ
│   │   ├── phase0_init.py 〜 phase5_camera.py   フェーズごとに 1 ファイル
│   │   ├── stuck.py       スタック検知と回避
│   │   └── state.py / logger.py / hardware.py / navigation.py / camera.py
│   ├── README.md        セットアップ手順書・ファイル一覧
│   └── FIELD_CHECKLIST.md  実機チェックリスト
├── sensor/      センサのドライバ (BNO055, BMP180, GPS, HC-SR04) と部品 (カメラ検知・方位計算)
├── tools/       実機で手で動かす道具
│   ├── finishv2.py          全センサ表示 (ゴール座標の測定)
│   ├── bno055_calibrate.py  BNO055 のキャリブレーション保存 (本番前に必須)
│   ├── phase5_live.py       Phase5 の検知確認 (--drive で走行)
│   ├── motor_test.py / led_test.py
│   └── network/             SSH・Wi-Fi の試験
├── test/        自動テスト (実機不要)。run_all.py で全部実行
├── docs/        報告書・解説 (PHASE5_GUIDE / REFACTOR_REPORT / REPORT / PROGRESS)
└── test_images/ カメラの確認画像
```

## フェーズの流れ

```
Phase0 初期化 → Phase1 落下検知 → Phase2 分離 (後退→前進) → Phase3 キャリブレーション
  → Phase4 GPS 誘導 (スタック回避・10m 以内でカメラ見張り)
  ⇄ Phase5 カメラで最終接近 → 超音波で 0m を確認 → 完了
```

古いプログラム (main2〜6.py、試作版の test_main*.py など) は 2026-11-11 に削除した。
必要なら git の履歴から取り出せる (例: `git show 81b922f:main/main6.py`)。
