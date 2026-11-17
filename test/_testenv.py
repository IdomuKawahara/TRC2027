"""
_testenv.py
===========
自動テストを、この Pi が今どの機体に設定されているかに関係なく、必ず本番機 (honban) の
設定で動かすための 1 行。mission を import する前に import すること。
(試験用の機体に切り替えたままテストすると、その機体のパルス長などで結果が変わってしまう)
"""
import os

os.environ["CANSAT_VEHICLE"] = "honban"
