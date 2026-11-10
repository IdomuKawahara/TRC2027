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
