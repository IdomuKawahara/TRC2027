"""
_testenv.py
===========
自動テストを、config.py の VEHICLE が何になっていても、必ず本番機 (honban) の
設定で動かすための 1 行。mission を import する前に import すること。
(過去機体に切り替えたままテストすると、その機体のパルス長などで結果が変わってしまう)
"""
import os

os.environ["CANSAT_VEHICLE"] = "honban"
