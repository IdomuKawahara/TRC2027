# 進捗 (リファクタリング + 改善)

ブランチ: `feature/refactor-stuck-recovery`  開始: 2026-11-10

分割の元ファイル: `main/main6.py` (= main2.py + 方位融合/キャリブ保存/Phase4カメラ。Phase1〜3,5 は main2.py と同一)
バックアップ: `main/main2_before_refactor.py`, `main/main6_before_refactor.py`

## 記録
- [x] ブランチ作成、バックアップ作成 (cmp で一致確認)
- [x] main6.py を main/mission/ に分割 (生成スクリプト: トークン単位で C./st. を付与、global 行を削除)
      config.py / state.py / logger.py / hardware.py / navigation.py / camera.py / phase0_init.py ...
      phase5_camera.py、main/main.py (run_phase4_and_5 + main のみ)
- [x] 全ファイル py_compile OK、import OK
- [x] 等価性の機械検証: 各モジュールのトークン列が main6.py の該当範囲と一致 (C./st. 除去後)、
      state.py は原文一致、config 154 値を AST 比較して一致・漏れなし (SENSOR_NAMES は state.py に配置)
- [ ] 既存シミュレーションテスト (Phase4/Phase5, 42 件) を mission パッケージに対して実行
- [ ] main.py の状態遷移テスト (test/test_mission_flow.py)
- [x] 既存シミュレーションテスト 42 件 (test_phase5_camera 21 + test_main6_improvements 21) を
      テスト本文を変えずに mission パッケージへ向けて実行 (test/test_mission_equivalence.py) → 全部 OK
- [x] main.py の状態遷移テスト test/test_mission_flow.py 12 件 OK
- [x] リファクタリングをコミット (動作変更なし)。ここから先が改善作業

## 改善作業 (リファクタリング後)
- [x] 地磁気偏角プリセット (仙台/能代/種子島, auto, override) を config.py に追加。国土地理院 2020.0 値 + 年変化で外挿
- [x] mission/stuck.py: StuckDetector (A/B 従来 + C: GPS 無進行, D: 片輪空転の窓判定) と
      StuckRecovery (段差型: 後退→ジャイロで回転確認→前進 / 轍型: ツイスト→ロッキング→回転確認 / 最大 3 ラウンド / cancel)
- [x] hardware.py: pivot_left / pivot_right (後退禁止ロック中は拒否)
- [x] navigation.py: calc_distance_between
- [x] phase4_gps.py: 検知/回避を stuck.py に差し替え、回避スレッドの二重起動防止と Phase4 終了時の中断
- [x] phase5_camera.py: 重要な瞬間の画像を別枠で保存 (CAM_SAVE_EVENT_MAX)、初検知/見失いも保存
- [x] test/test_stuck_recovery.py 19 件 OK (偽時計の GIL 問題と罠の配置を修正して通過)
- [ ] 既存テスト (equivalence 42 / flow 12 / goal_vision 51 / heading 9) の再実行
- [ ] REFACTOR_REPORT.md, README, FIELD_CHECKLIST 更新 → コミット → push
- [x] 偏角プリセットの単体テストを test_mission_flow.py に追加、test_phase5_live.py を mission パッケージ (Facade 経由) に切り替え
- [x] README (ファイル一覧) / FIELD_CHECKLIST (偏角・スタック試験) を更新
- [x] 回帰スイート一式: flow 16 / equivalence 42 / stuck 19 / heading 9 / goal_vision 51 → 全部 OK
- [x] REFACTOR_REPORT.md 作成
- [x] コミット → push (feature/refactor-stuck-recovery)

## 整理 (2026-11-11)
- [x] 削除: main2〜6.py・リファクタ前バックアップ・runtest.py / sensor の重複 (GPS2, GPStest, motor_thread, test_avoid, test_fall2) /
      test の試作版 (test_main*.py 16 本, test_GPSrun*.py 8 本, guided_navigation.py, test_fall*, test_finishv*, test_GPS, test_avoid/go/straight,
      空ファイル 2 本, test_mission_equivalence.py)。すべて git の履歴に残っている
- [x] 移動: 実機の道具 → tools/ (phase5_live.py, bno055_calibrate.py, finishv2.py, motor_test.py, led_test.py)、
      SSH/Wi-Fi 試験 → tools/network/、報告書 → docs/
- [x] test_phase5_camera.py → test_phase5_sim.py、test_main6_improvements.py → test_phase4_sim.py
      (main2/main6 ではなく mission パッケージを直接テストするように変更)
- [x] test/run_all.py を追加。整理後に全テスト 137 件 OK
- [x] README (ファイル一覧)・FIELD_CHECKLIST・PHASE5_GUIDE のパスを更新、リポジトリ直下に README.md を追加
