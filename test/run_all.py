"""
run_all.py
==========
test/ の自動テスト (実機不要) をすべて実行する。

    cd ~/TRC2027
    python3 test/run_all.py

Pi のメモリが少ないので、テストファイルを 1 つずつ別のプロセスで順番に実行する
(同時には走らせない)。最後に結果の一覧を表示し、1 つでも失敗したら終了コード 1。
全部で数分かかる。
"""
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent

TESTS = [
    ("test_mission_flow.py",    "main.py のフェーズ切り替え・偏角プリセット"),
    ("test_heading_fusion.py",  "方位の融合 (地磁気 + GPS)"),
    ("test_goal_vision.py",     "カメラの色検知・超音波の 0m 判定"),
    ("test_stuck_recovery.py",  "スタック検知と回避"),
    ("test_phase4_sim.py",      "Phase4 (GPS 誘導) シミュレーション"),
    ("test_phase5_sim.py",      "Phase5 (カメラ最終接近) シミュレーション"),
]


def main():
    results = []
    for name, desc in TESTS:
        print(f"=== {name}  ({desc})", flush=True)
        t0 = time.time()
        proc = subprocess.run([sys.executable, str(HERE / name)],
                              capture_output=True, text=True)
        out = proc.stdout + proc.stderr
        ran = next((l for l in out.splitlines() if l.startswith("Ran ")), "Ran ? tests")
        ok = proc.returncode == 0
        results.append((name, desc, ok, ran, time.time() - t0))
        print(f"    {'OK' if ok else 'FAILED'}  {ran}  ({time.time() - t0:.0f}s)", flush=True)
        if not ok:
            tail = [l for l in out.splitlines() if l.startswith(("FAIL:", "ERROR:", "AssertionError"))]
            for l in tail[:10]:
                print("    " + l)
    print()
    print("結果:")
    for name, desc, ok, ran, sec in results:
        print(f"  {'✅' if ok else '❌'} {name:26s} {ran:16s} {desc}")
    failed = [r for r in results if not r[2]]
    print("すべて OK" if not failed else f"{len(failed)} 個のファイルで失敗")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
