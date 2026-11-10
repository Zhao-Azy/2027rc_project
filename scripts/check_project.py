#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keil 工程一致性检查 —— 提前发现"编译不过 / 改了没编进去"这类坑。

检查两件事：
  1. R1.uvprojx 里引用的源文件是否都存在（缺文件 Keil 会直接报错，且报错位置很难找）
  2. 仓库里存在但没加进工程的 .c 文件（加了代码却忘了加工程 = 改了等于没改）

用法：python3 scripts/check_project.py
返回：0 = 没问题；1 = 有问题（会打印明细）
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJ = os.path.join(ROOT, "dm_demo", "MDK-ARM", "R1.uvprojx")
SCAN_DIRS = ("app", "bsp", "app_lib", "Core/Src")


def rel(path: str) -> str:
    return os.path.relpath(path, ROOT)


def main() -> int:
    if not os.path.isfile(PROJ):
        print(f"[错误] 找不到 Keil 工程：{rel(PROJ)}")
        return 1

    xml = open(PROJ, encoding="utf-8", errors="replace").read()
    refs = set()
    for raw in re.findall(r"<FilePath>([^<]+)</FilePath>", xml):
        w = raw.replace("\\", "/")
        refs.add(os.path.normpath(os.path.join(ROOT, "dm_demo", "MDK-ARM", w)))

    problems = 0

    missing = sorted(p for p in refs if not os.path.isfile(p))
    if missing:
        problems += len(missing)
        print(f"[错误] 工程引用了 {len(missing)} 个不存在的文件（Keil 编译会失败）：")
        for p in missing:
            print(f"        ✗ {rel(p)}")
    else:
        print(f"[通过] 工程引用的 {len(refs)} 个文件都存在")

    present = []
    for d in SCAN_DIRS:
        base = os.path.join(ROOT, "dm_demo", d)
        for dirpath, _, files in os.walk(base):
            for f in files:
                if f.endswith(".c"):
                    p = os.path.normpath(os.path.join(dirpath, f))
                    if p not in refs:
                        present.append(p)
    if present:
        print(f"[提示] {len(present)} 个 .c 存在但没加进工程（如果是要用的代码，改了不会生效）：")
        for p in sorted(present):
            print(f"        · {rel(p)}")

    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
