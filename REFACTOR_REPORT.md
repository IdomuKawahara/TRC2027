# REFACTOR_REPORT: main6.py の分割 と 改善 (2026-11-10)

ブランチ: `feature/refactor-stuck-recovery`
コミット: ① バックアップ (`43f14e3`) → ② 分割のみ・動作変更なし (`022b8ea`) → ③ 改善 (このコミット)

## 0. 要点

- **分割の元は `main/main6.py`** (= main2.py に前回依頼の「方位融合・キャリブ保存・Phase4 カメラ」を足した最新版。Phase1〜3・5 は main2.py と同一)。
  指示どおり `main2_before_refactor.py` を作り、あわせて `main6_before_refactor.py` も作った。
- 分割は **トークン単位の機械的な書き換え** (設定値 → `C.名前`、共有状態 → `st.名前`、`global` 文の削除) だけで、
  ロジック・判定条件・数値・処理順は変えていない。これを **3 通りの方法で検証** した (§4)。
- 分割後に行った改善 (§5) は、**別コミット** で、Phase2 (パラシュート分離) には一切触れていない。

## 1. 新しいファイル構成

```
main/main.py                 ← 入口。フェーズの切り替えだけ (run_phase4_and_5 / main)
main/mission/
  __init__.py                sensor/ を import パスに追加
  config.py                  ★ 設定値 (154 個 + 改善で追加分)。実地で変えるのはここだけ
  state.py                   フェーズ間で共有する状態 (g_* センサ最新値, phase, ログバッファ, センサ取得状況)
  logger.py                  log / log_data_status / log_sensor_row / save_log / blink_led
  hardware.py                GPS 受信スレッド, SonarSensor, MotorController (+pivot_left/right), set_bno_orientation
  navigation.py              calc_distance / calc_distance_between / calc_target_bearing / calc_azimuth(_with_front) / calc_direction_diff
  camera.py                  make_goal_detector / make_goal_check_config / make_camera
  phase0_init.py             Phase0 初期化
  phase1_fall.py             Phase1 落下検知
  phase2_separation.py       Phase2 分離 (後退→停止→前進)  ★変更禁止 (パラシュート分離)
  phase3_calibration.py      Phase3 キャリブレーション + GPS 安定化
  phase4_gps.py              Phase4 GPS 誘導 (方位融合・カメラ見張り)
  stuck.py                   ★新規: スタック検知 (StuckDetector) と回避 (StuckRecovery)
  phase5_camera.py           Phase5 カメラ最終接近 + 超音波 0m 確認 (+ 超音波スキャンの予備)
test/
  mission_facade.py          旧 main6 と同じ「M.名前」で mission を扱うテスト用アダプタ
  test_mission_equivalence.py 既存シミュレーション 42 件を無変更で mission に向けて実行
  test_mission_flow.py       main.py の状態遷移 (モック) + 偏角プリセット
  test_stuck_recovery.py     スタック検知/回避のシミュレーション
```

各モジュールは `from . import config as C` / `from . import state as st` で設定と状態を参照し、
関数は `from .logger import log` のように明示的に import する (循環 import なし: config ← state ← logger ← hardware/navigation/camera ← phase* ← main)。

### 各ファイルの役割と元の位置

| ファイル | 役割 | main6.py の元の行 |
|---|---|---|
| `config.py` | 設定値 | 248-249, 255-264, 311-654, 1097-1130 |
| `state.py` | 共有状態・センサ取得状況 (`mark_sensor_ok/fail`, `format_sensor_status`) | 218-247, 251-253, 266-309 (原文のまま) |
| `logger.py` | ログ・CSV・LED | 656-736 |
| `hardware.py` | GPS スレッド・超音波・モータ・Axis Remap | 186-191, 738-1031, 1132-1190 |
| `navigation.py` | 航法計算 | 1032-1095, 1192-1214 |
| `camera.py` | カメラ関連の組み立て | 2403-2427 |
| `phase0_init.py` | Phase0 | 1216-1312 |
| `phase1_fall.py` | Phase1 | 1313-1371 |
| `phase2_separation.py` | Phase2 | 1372-1493 |
| `phase3_calibration.py` | Phase3 | 1494-1749 |
| `phase4_gps.py` | Phase4 | 1750-2179 |
| `phase5_camera.py` | Phase5 (+ 予備) | 2180-2400, 2429-2807 |
| `main.py` | フェーズ切り替え | 1-169 (説明), 2809-2903 |

## 2. config.py に移した値の一覧 (元の場所と値)

| config.py の名前 | 値 | 元の main6.py 行 | 一致 |
|---|---|---|---|
| `TARGET_LAT` | `38.26052` | 318 | ✅ |
| `TARGET_LNG` | `140.8544151` | 319 | ✅ |
| `PHASE4_TO_5_RADIUS` | `2.0` | 323 | ✅ |
| `FALL_THRESHOLD` | `3.0` | 329 | ✅ |
| `FALL_COUNT_THRESHOLD` | `8` | 330 | ✅ |
| `FALL_TIMEOUT_SEC` | `7 * 60` | 331 | ✅ |
| `PHASE2_BACK_TIMEOUT_SEC` | `30.0` | 343 | ✅ |
| `PHASE2_BACK_MIN_SEC` | `3.0` | 344 | ✅ |
| `PHASE2_SONAR_JUMP_THRESH_M` | `0.5` | 346 | ✅ |
| `PHASE2_SONAR_LOST_TIMEOUT_SEC` | `5.0` | 348 | ✅ |
| `PHASE2_LOOP_DT` | `0.1` | 350 | ✅ |
| `PHASE2_PAUSE_SEC` | `5.0` | 351 | ✅ |
| `PHASE2_FWD_AFTER_SEC` | `10.0` | 352 | ✅ |
| `CALIB_MIN_LEVEL_SYS` | `1` | 375 | ✅ |
| `CALIB_MIN_LEVEL_GYRO` | `1` | 376 | ✅ |
| `CALIB_MIN_LEVEL_ACC` | `0` | 378 | ✅ |
| `CALIB_MIN_LEVEL_MAG` | `2` | 380 | ✅ |
| `CALIB_TIMEOUT_SEC` | `180.0` | 381 | ✅ |
| `CALIB_ACC_MEASURE_SEC` | `3.0` | 383 | ✅ |
| `CALIB_REPORT_INTERVAL_SEC` | `5.0` | 385 | ✅ |
| `CALIB_STALL_WARN_SEC` | `20.0` | 387 | ✅ |
| `CALIB_STILL_SEC` | `6.0` | 391 | ✅ |
| `CALIB_SETTLE_SEC` | `1.5` | 396 | ✅ |
| `FIG8_ENABLE` | `True` | 402 | ✅ |
| `FIG8_SPIN_SEC` | `8.0` | 403 | ✅ |
| `GPS_MIN_SATS` | `4` | 407 | ✅ |
| `GPS_CALIB_TIMEOUT_SEC` | `60.0` | 408 | ✅ |
| `GPS_CALIB_SAMPLES` | `10` | 409 | ✅ |
| `GPS_CALIB_SAMPLE_SEC` | `8.0` | 410 | ✅ |
| `LED_BLINK_SEC` | `10.0` | 413 | ✅ |
| `LED_BLINK_INTERVAL` | `0.3` | 414 | ✅ |
| `LOOP_DT` | `0.1` | 417 | ✅ |
| `TIMEOUT_SEC` | `15 * 60` | 418 | ✅ |
| `ANGLE_DEADBAND` | `10.0` | 421 | ✅ |
| `ANGLE_TURN_STRONG` | `45.0` | 422 | ✅ |
| `ANGLE_180_PREFER_RIGHT_DEG` | `170.0` | 438 | ✅ |
| `STUCK_HORIZON_ACCEL_THRESH` | `0.5` | 450 | ✅ |
| `STUCK_GPS_SPEED_THRESH_MPS` | `0.15` | 451 | ✅ |
| `KNOTS_TO_MPS` | `0.514444` | 452 | ✅ |
| `STUCK_SONAR_DIST_THRESH` | `0.3` | 453 | ✅ |
| `STUCK_COUNT_THRESHOLD` | `40` | 454 | ✅ |
| `STUCK_RECOVER_TURN_DEG` | `90.0` | 458 | ★変更: 元=180.0 |
| `STUCK_RECOVER_RETRY_TURN_DEG` | `90.0` | 459 | ✅ |
| `STUCK_RECOVER_TURN_TIMEOUT_SEC` | `15.0` | 460 | ✅ |
| `STUCK_RECOVER_FWD_SEC` | `10.0` | 461 | ✅ |
| `STUCK_RECOVER_MAX_RETRIES` | `5` | 462 | ✅ |
| `NAV_REPORT_INTERVAL_SEC` | `5.0` | 465 | ✅ |
| `FINAL_STOP_DIST_M` | `0.05` | 485 | ✅ |
| `FINAL_SCAN_SEC` | `0.3` | 486 | ✅ |
| `FINAL_APPROACH_TIMEOUT_SEC` | `5 * 60` | 487 | ✅ |
| `FINAL_NAV_REPORT_INTERVAL_SEC` | `3.0` | 488 | ✅ |
| `FINAL_SONAR_LOST_WARN_SEC` | `15.0` | 489 | ✅ |
| `FINAL_SCAN_LOOP_DT` | `0.02` | 490 | ✅ |
| `CAM_SIZE` | `(320, 240)` | 498 | ✅ |
| `CAM_FULL_FOV` | `True` | 499 | ✅ |
| `CAM_ROTATE_180` | `True` | 500 | ✅ |
| `CAM_SWAP_RB` | `False` | 502 | ✅ |
| `CAM_WARMUP_SEC` | `1.5` | 503 | ✅ |
| `CAM_FAIL_LIMIT` | `10` | 504 | ✅ |
| `CAM_HUE_RANGES` | `((0, 12), (165, 179))` | 506 | ✅ |
| `CAM_S_MIN` | `100` | 507 | ✅ |
| `CAM_V_MIN` | `70` | 508 | ✅ |
| `CAM_RED_MARGIN` | `30` | 509 | ✅ |
| `CAM_RED_MAX_GB_RATIO` | `0.55` | 510 | ✅ |
| `CAM_MIN_AREA_RATIO` | `0.0015` | 511 | ✅ |
| `CAM_NEAR_AREA_RATIO` | `0.20` | 512 | ✅ |
| `CAM_CONTACT_AREA_RATIO` | `0.55` | 513 | ✅ |
| `CAM_BACKEND` | `"auto"` | 514 | ✅ |
| `CAM_SHAPE_MIN_SCORE` | `0.25` | 515 | ✅ |
| `CAM_SETTLE_SEC` | `0.15` | 518 | ✅ |
| `CAM_CENTER_TOL` | `0.10` | 519 | ✅ |
| `CAM_TURN_MIN_SEC` | `0.08` | 520 | ✅ |
| `CAM_TURN_GAIN_SEC` | `0.6` | 521 | ✅ |
| `CAM_TURN_MAX_SEC` | `0.30` | 522 | ✅ |
| `CAM_FWD_FAR_SEC` | `0.8` | 523 | ✅ |
| `CAM_FWD_NEAR_SEC` | `0.25` | 524 | ✅ |
| `CAM_LOST_HOLD_FRAMES` | `2` | 526 | ✅ |
| `CAM_SEARCH_TURN_SEC` | `0.25` | 527 | ✅ |
| `CAM_SEARCH_LAP_DEG` | `360.0` | 528 | ✅ |
| `CAM_SEARCH_PULSES_PER_LAP` | `24` | 529 | ✅ |
| `CAM_SEARCH_MAX_PULSES_PER_LAP` | `48` | 530 | ✅ |
| `CAM_SEARCH_REPOSITION_SEC` | `2.0` | 531 | ✅ |
| `CAM_SEARCH_MAX_LAPS` | `6` | 532 | ✅ |
| `PHASE5_MAX_RETURNS_TO_GPS` | `3` | 538 | ✅ |
| `PHASE4_REENTRY_MIN_SEC` | `10.0` | 539 | ✅ |
| `PHASE5_RETURN_GPS` | `"RETURN_GPS"` | 542 | ✅ |
| `CAM_OBSTACLE_TURN_SEC` | `0.6` | 544 | ✅ |
| `CAM_OBSTACLE_FWD_SEC` | `1.0` | 545 | ✅ |
| `CAM_SAVE_INTERVAL_SEC` | `5.0` | 547 | ✅ |
| `CAM_SAVE_MAX` | `150` | 548 | ✅ |
| `CAM_REPORT_INTERVAL_SEC` | `3.0` | 549 | ✅ |
| `GOAL_CHECK_TRIGGER_DIST_M` | `0.40` | 554 | ✅ |
| `GOAL_SONAR_DIST_M` | `0.08` | 555 | ✅ |
| `GOAL_SONAR_SAMPLES` | `5` | 556 | ✅ |
| `GOAL_SONAR_MIN_VALID` | `3` | 557 | ✅ |
| `GOAL_SONAR_MAX_SPREAD_M` | `0.05` | 558 | ✅ |
| `GOAL_SONAR_INTERVAL_SEC` | `0.06` | 559 | ✅ |
| `GOAL_SETTLE_SEC` | `0.3` | 560 | ✅ |
| `GOAL_BACKOFF_SEC` | `0.25` | 561 | ✅ |
| `GOAL_BACKOFF_CONFIRM_DIST_M` | `0.20` | 562 | ✅ |
| `GOAL_FINAL_PUSH_SEC` | `0.30` | 563 | ✅ |
| `GOAL_PUSH_TOLERANCE_M` | `0.05` | 564 | ✅ |
| `GOAL_MAX_BLIND_RETRIES` | `4` | 565 | ✅ |
| `GOAL_MAX_CONTACT_FAR` | `5` | 566 | ✅ |
| `GOAL_REAPPROACH_BACK_SEC` | `1.0` | 568 | ✅ |
| `HEADING_COG_MIN_SPEED_MPS` | `0.30` | 571 | ✅ |
| `HEADING_STRAIGHT_MIN_SEC` | `1.5` | 572 | ✅ |
| `HEADING_GYRO_MAX_DPS` | `8.0` | 573 | ✅ |
| `HEADING_COG_MAX_AGE_SEC` | `1.5` | 574 | ✅ |
| `HEADING_OFFSET_ALPHA` | `0.25` | 575 | ✅ |
| `HEADING_MIN_OFFSET_SAMPLES` | `3` | 576 | ✅ |
| `HEADING_OUTLIER_DEG` | `45.0` | 577 | ✅ |
| `BNO_CALIB_FILE` | `_ROOT_DIR / "config" / "bno055_calib.json"` | 580 | ✅ |
| `BNO_CALIB_LOAD` | `True` | 581 | ✅ |
| `BNO_CALIB_AUTOSAVE` | `True` | 582 | ✅ |
| `CALIB_TIMEOUT_WITH_PROFILE_SEC` | `30.0` | 583 | ✅ |
| `CAM_P4_ENABLE` | `True` | 586 | ✅ |
| `CAM_P4_ACTIVATE_RADIUS_M` | `10.0` | 587 | ✅ |
| `CAM_P4_DEACTIVATE_RADIUS_M` | `15.0` | 588 | ✅ |
| `CAM_P4_INTERVAL_SEC` | `0.25` | 589 | ✅ |
| `CAM_P4_THREADED` | `True` | 590 | ✅ |
| `CONE_LOCK_FRAMES` | `3` | 591 | ✅ |
| `CONE_LOCK_CONSISTENCY_DEG` | `20.0` | 592 | ✅ |
| `CONE_LOCK_GPS_CHECK_MIN_M` | `4.0` | 593 | ✅ |
| `CONE_LOCK_GPS_BEARING_TOL_DEG` | `60.0` | 594 | ✅ |
| `PHASE4_TIMEOUT_P5_RADIUS_M` | `20.0` | 595 | ✅ |
| `RAMP_START_DUTY` | `0.8` | 605 | ✅ |
| `RAMP_END_DUTY` | `1.0` | 606 | ✅ |
| `RAMP_DURATION_SEC` | `3.0` | 607 | ✅ |
| `RAMP_UPDATE_INTERVAL_SEC` | `0.2` | 608 | ✅ |
| `SPEED_WEAK` | `0.4` | 613 | ✅ |
| `WEAK_RATIO` | `SPEED_WEAK / RAMP_START_DUTY` | 614 | ✅ |
| `PIN_PWMA` | `13` | 623 | ✅ |
| `PIN_AIN1` | `6` | 624 | ✅ |
| `PIN_AIN2` | `5` | 625 | ✅ |
| `PIN_PWMB` | `18` | 626 | ✅ |
| `PIN_BIN1` | `24` | 627 | ✅ |
| `PIN_BIN2` | `23` | 628 | ✅ |
| `PIN_STBY` | `11` | 629 | ✅ |
| `LED_PIN` | `21` | 632 | ✅ |
| `GPS_PORT` | `"/dev/serial0"` | 635 | ✅ |
| `GPS_BAUDRATE` | `9600` | 636 | ✅ |
| `PIN_TRIG` | `8` | 639 | ✅ |
| `PIN_ECHO` | `7` | 640 | ✅ |
| `SONAR_SETTLE` | `0.01` | 641 | ✅ |
| `SONAR_PULSE` | `10e-6` | 642 | ✅ |
| `SONAR_TIMEOUT` | `0.03` | 643 | ✅ |
| `SOUND_SPEED` | `343.0` | 644 | ✅ |
| `EARTH_RADIUS` | `6371000.0` | 651 | ✅ |
| `LOG_DIR` | `_ROOT_DIR / "logs"` | 654 | ✅ |
| `MAG_EMA_ALPHA` | `0.25` | 248 | ✅ |
| `PHASE_NAMES` | `{     0: "初期化",     1: "落下検知",     2: "初期後退・停止・前進",     3...` | 257 | ✅ |
| `BNO_ORIENTATION` | `1` | 1130 | ✅ |

移動した値: 153 個 (うち値が同一: 152 個)

### ★改善で新しく追加した設定 (main6.py には無かったもの)
| 名前 | 値 |
|---|---|
| `MAG_SITE` | `"auto"` |
| `MAG_SITE_PRESETS` | `{     #                 名前      緯度     経度     2020.0 偏角[°]  変化[°/年]...` |
| `MAG_DECLINATION_OVERRIDE` | `None` |
| `STUCK_NOPROGRESS_SEC` | `12.0` |
| `STUCK_NOPROGRESS_DIST_M` | `1.0` |
| `STUCK_SPIN_GYRO_DPS` | `25.0` |
| `STUCK_SPIN_WINDOW` | `60` |
| `STUCK_SPIN_COUNT` | `45` |
| `STUCK_BACK_SEC` | `1.5` |
| `STUCK_ROTATE_CHECK_SEC` | `3.0` |
| `STUCK_ROTATE_MIN_DEG` | `15.0` |
| `STUCK_ROTATE_OK_FRAC` | `0.6` |
| `STUCK_TWIST_CYCLES` | `6` |
| `STUCK_TWIST_PULSE_SEC` | `0.35` |
| `STUCK_TWIST_PULSE_GROWTH` | `0.5` |
| `STUCK_ROCK_CYCLES` | `4` |
| `STUCK_ROCK_SEC` | `0.4` |
| `STUCK_MAX_ROUNDS` | `3` |
| `STUCK_PAUSE_SEC` | `0.15` |
| `CAM_SAVE_EVENT_MAX` | `60` |


- `MAG_DECLINATION` は表に出ていない: 元 (318 行付近) の固定値 `-8.0` を、★改善で会場プリセットから計算する値に置き換えた
  (`MAG_SITE_RESOLVED, MAG_DECLINATION = _resolve_mag_declination(...)`。仙台・2026-11-10 で `-9.01`)。
  これは意図した変更 (§5-2)。分割のみのコミット (`022b8ea`) では `-8.0` のまま。
- `SENSOR_NAMES` (元 270 行) は state.py 内の関数だけが使うので state.py に置いた。
- `STUCK_RECOVER_TURN_DEG` 180 → 90 は ★改善 (§5-1) での意図した変更。分割のみのコミットでは 180 のまま。

## 3. 設定値・状態の参照方法の変更 (機械的な書き換え)

| 元 (main6.py) | 分割後 |
|---|---|
| `FALL_THRESHOLD` (モジュール定数) | `C.FALL_THRESHOLD` |
| `global g_acc` + `g_acc = bno.getAcc()` | `st.g_acc = bno.getAcc()` (`global` 文は削除) |
| `phase = 4` | `st.phase = 4` |
| `log(...)`, `calc_distance(...)` など関数 | 同名のまま (各モジュールで import) |

書き換えは Python の `tokenize` で NAME トークンだけを対象にしたので、文字列・コメント・docstring は一切変わっていない。

## 4. 「動作を変えていない」ことの検証 (分割コミット `022b8ea` 時点)

| 方法 | 内容 | 結果 |
|---|---|---|
| トークン列の照合 | 各モジュールから `C.` / `st.` を外したトークン列 (コメント・改行を除く) を、main6.py の該当行範囲 (`global` 行を除く) と比較 | 11 モジュール + main.py すべて一致 (計 11,383 トークン) |
| state.py | 原文のまま貼り付けたことを文字列比較 | 一致 |
| config の値 | 154 個の代入を AST で比較 | すべて一致・漏れなし |
| 既存シミュレーション | main6 向けテスト 42 件 (`test_phase5_camera.py` 21 + `test_main6_improvements.py` 21) を **本文を変えずに** Facade 経由で mission に向けて実行 | 42/42 OK |
| 状態遷移 | `test_mission_flow.py`: Phase0→1→2→3→4→5 の順、引数の受け渡し、Phase5→4 の戻り、タイムアウト/ABORT/例外/Ctrl+C 時の終了処理 | 12/12 OK |
| 構文・import | `py_compile` 全ファイル、全モジュール import、循環 import なし | OK |

## 5. 改善 (別コミット)

### 5-1. スタック検知と回避 (`mission/stuck.py`, `phase4_gps.py`, `hardware.py`)

**検知** (`StuckDetector`) — 従来の A/B に C/D を追加:

| 判定 | 条件 | 既定 |
|---|---|---|
| A (従来) | 前進中に水平加速度が静止時並み かつ GPS 速度 < 0.15 m/s が 4 s 連続 | 変更なし |
| B (従来) | 超音波 < 0.3 m が 4 s 連続 | 変更なし |
| **C 追加** | 走行指令 (前進・旋回) が続いているのに GPS 位置が 12 s で 1 m 動かない | 空転 (振動で A が成立しない) を拾う |
| **D 追加** | 走行指令中・GPS 速度なしで、直近 6 s のうち 4.5 s 以上ヨー角速度 ≥ 25°/s | 片輪空転でその場回転 → **轍型の疑い** |

**回避** (`StuckRecovery.run`) — 2 種類のスタックに分けた段階的な動作。**回避中だけ後退を許可** し、終わったら後退禁止に戻す:

```
STEP (段差・壁) 想定 : 後退 1.5 s → ジャイロで測りながら 90° 回転 → 前進 10 s (再スタック監視)
                       └ 回転チェック: 3 s で 15° も回れない or 目標の 60% 未満 → 「タイヤが地面を掴めていない」= RUT へ
RUT (轍・はまり込み)  : ツイスト (左ピボット⇄右ピボット 0.35 s × 6 回, ラウンドごとに 1.5 倍)
                       → ロッキング (後退⇄前進 0.4 s × 4 回) → 90° 回転チェック → 前進 10 s
最大 3 ラウンド。抜けられなければあきらめて誘導走行に戻る (無限に繰り返さない)
```

- **ピボット旋回** (`pivot_left/right`: 左右のタイヤを逆回転) を MotorController に追加。後退禁止ロック中は拒否する。
- **回転量はジャイロの積分**で測る (モータ通電中は地磁気が乱れるため)。ジャイロが読めないときだけ地磁気。
- 初回の回転角を **180° → 90°** に変更 (180° で 10 s 進むと同じ段差に同じ向きから戻ってしまう。90° で横に回り込み、再スタックのたびに +90°)。
- 回避スレッドの **二重起動を防止** (フラグを起動前に立てる) し、Phase4 終了時に **回避を中断して合流** する (以前は Phase5 と モータを取り合う可能性があった)。

### 5-2. 地磁気偏角のプリセット (`config.py`)

`MAG_SITE = "auto" | "sendai" | "noshiro" | "tanegashima"`。`auto` は `TARGET_LAT/LNG` に一番近い会場。
値は国土地理院「都道府県庁所在地の偏角値」(2020.0 年値) を元に、年変化で実行日まで外挿する。

| 会場 | 2020.0 (西偏) | 年変化 | 2026-11-10 の値 | 備考 |
|---|---|---|---|---|
| 仙台 | 8.6° | +0.06°/年 | **-9.01°** | 表の仙台の値。従来の固定値 -8.0 より約 1° 大きい |
| 能代 | 8.8° (概算) | +0.06°/年 | **-9.21°** | 秋田 8.8 / 青森 8.7 から補間 |
| 種子島 | 6.8° (概算) | +0.08°/年 | **-7.35°** | 鹿児島 7.2 / 那覇 5.5 から緯度で補間 |

`MAG_DECLINATION_OVERRIDE` に数値を入れるとプリセットより優先。走行中は GPS の進行方位との差を学習する (heading_fusion) ので、0.3° 程度の誤差は結果に影響しない。

### 5-3. その他

- Phase5 の画像保存: 重要な瞬間 (初めて見えた・見失った・0m 確認・後退確認・押し込み後) は定期保存の上限 (150 枚) とは **別枠** で `CAM_SAVE_EVENT_MAX` (60) 枚まで必ず保存。以前は上限に達すると最後の到達確認の画像が残らないことがあった。
- `navigation.calc_distance_between` (2 点間距離) を追加 (判定 C 用)。

## 6. テスト結果 (改善後)

| テスト | 件数 | 結果 | 内容 |
|---|---|---|---|
| `test/test_mission_flow.py` | 16 | ✅ | main.py の状態遷移 (12) + 偏角プリセット (4) |
| `test/test_mission_equivalence.py` | 42 | ✅ | 既存の Phase4/Phase5 シミュレーションを mission パッケージで (改善後も全部通る = 既存の走行・最終接近の動作は維持) |
| `test/test_stuck_recovery.py` | 19 | ✅ | 下表 |
| `test/test_heading_fusion.py` | 9 | ✅ | 方位融合 (変更なし) |
| `test/test_goal_vision.py` | 51 | ✅ | 色検知 (変更なし) |
| `python3 -m py_compile` 全ファイル | — | ✅ | |

`test_stuck_recovery.py` のシナリオ (モータ・センサは偽物。ゴールまで 14 m、経路上に罠を置く):

| シナリオ | 期待 | 結果 |
|---|---|---|
| 段差 (前にだけ進めない) | 4 s で判定=ACC → 後退 → 90° 回転 → 前進 → ゴール到達。ツイストはしない。後退禁止ロック中に後退していない | ✅ |
| 轍 (前後・片輪旋回が効かない) | まず後退を試す → 回転できない → 轍型と判断 → ツイスト+ロッキング → 抜けてゴール | ✅ |
| 片輪空転 (前進でその場回転) | 判定=SPIN → **最初から**ツイスト → ゴール | ✅ |
| 空転 (振動はあるが進まない) | 判定=NOPROGRESS (12 s) → ツイストで脱出 | ✅ |
| 壁 (超音波 0.2 m) | 判定=SONAR → 後退 → 回転 → ゴール | ✅ |
| 段差 → 轍 の 2 連続 | 2 回とも回避してゴール | ✅ |
| 抜けられない轍 | 3 ラウンドであきらめて誘導に戻る → 再検知 → 再試行 (Phase4 のタイムアウトまで無限に抜けないが止まりもしない) | ✅ |
| 何もない草地 | 誤検知しない (回避動作なし) | ✅ |
| 回避の途中で Phase4 タイムアウト | 回避スレッドが中断され、モータ停止・後退禁止ロック解除で終わる | ✅ |
| 検知器の単体 (A/B/C/D/境界/リセット/誘導の 180° 旋回を SPIN と誤判定しない) | | ✅ 8 件 |
| pivot_left/right が後退禁止ロックを守る | | ✅ |

テストを書く過程で直した問題: 回避スレッドの二重起動 (本体)、Phase4 終了時に回避スレッドが残る (本体)、
偽の時計を 2 スレッドで進めると走行ループが止まる (テスト側)、回避中の後退が後退禁止ロックに阻まれる設計上の矛盾 (本体: 回避中だけ解除)。

## 7. 実機で確認すべきこと (このリファクタリング/改善に関係する分。全体は `main/FIELD_CHECKLIST.md`)

1. **`python3 main/main.py` で起動し、Phase0 のログに `偏角` と会場名 (sendai など) が出ること**。会場が違えば `config.py` の `MAG_SITE` を指定する。
2. **段差に当てる**: 4 s 以内に「スタック検知 (判定=ACC)」→ 後退 1.5 s → 「回転 目標90° / 実測約○○° (ジャイロ)」→ 前進。実測角が 90° から大きく外れるなら `STUCK_RECOVER_TURN_TIMEOUT_SEC` や機体の旋回速度を確認。
3. **タイヤを溝・砂に落とす**: 「回れません → 轍型とみなしてツイスト」→ ピボットが実際に左右逆回転しているか (モータ A/B の配線が逆だとその場旋回にならない)。`STUCK_TWIST_PULSE_SEC`/`STUCK_ROCK_SEC` の長さを地面に合わせる。
4. **片輪を浮かせる**: 6 s 以内に「判定=SPIN」。出なければ `STUCK_SPIN_GYRO_DPS` を機体の実際のヨー角速度に合わせる。
5. 平らな場所で 20〜30 m 走って **誤検知しない** こと (特に判定 C: 速度 0.3 m/s 以上出ていれば 12 s で 3.6 m 進むので問題ないが、極端に遅い機体だと `STUCK_NOPROGRESS_SEC` を延ばす)。
6. 回避の後に通常走行で `backward()` の WARN が出ていない (後退禁止ロックが戻っている)。
7. 回避で後退するとき、後ろにパラシュート・サブキャリアが無い位置まで来ているか (Phase2 の前進 10 s で離れている前提。心配なら `STUCK_BACK_SEC` を短く)。

## 8. 変えていないこと

- Phase1 (落下検知)・Phase2 (分離) のロジックと数値 (指示どおり。Phase1 の「着地ではなく放出を検知している」問題は `REPORT.md` §9 / `FIELD_CHECKLIST.md` §9 に未対応として記載)。
- Phase5 の判定・数値 (画像の保存枚数の数え方だけ変更)。
- 既存の `main2.py` / `main6.py` / 旧テスト (`test_phase5_camera.py` は mission 版用の設定 2 行だけ追加)。
