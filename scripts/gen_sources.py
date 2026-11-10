#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 Keil 工程 R1.uvprojx 生成源文件列表，给 Makefile 用。

这样做的原因：工程里哪些 .c 参与编译，是 uvprojx 说了算（有 16 个 .c 虽然
在仓库里但故意不参与编译）。与其在 Makefile 里再抄一份容易过期的清单，不如
每次都从工程文件里读。

用法：
    python3 scripts/gen_sources.py c    # 输出要编译的 .c（相对 dm_demo/ 的路径）
    python3 scripts/gen_sources.py s    # 输出汇编文件（已替换成 GCC 版启动文件）

两处必须替换，否则 GCC 编不了：
    · 启动文件：工程里是 ARM 汇编语法，换成 Drivers/.../Templates/gcc/ 下的 GNU 版
    · FreeRTOS port：工程里是 RVDS(AC5) 版，换成 portable/GCC/ARM_CM4F
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # 仓库根
DM = os.path.join(ROOT, "dm_demo")
PROJ = os.path.join(DM, "MDK-ARM", "R1.uvprojx")

# 工程里的 ARM 语法启动文件 -> 仓库里自带的 GCC 语法启动文件
STARTUP_REPLACE = {
    "MDK-ARM/startup_stm32h723xx.s":
        "Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h723xx.s",
}

# RVDS(AC5) port -> GCC port（内容在 portable/GCC/ARM_CM4F 里，版本同样是 V10.3.1）
GCC_PORT = "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c"


def project_files():
    xml = open(PROJ, encoding="utf-8", errors="replace").read()
    out = []
    for raw in re.findall(r"<FilePath>([^<]+)</FilePath>", xml):
        p = raw.replace("\\", "/")
        p = os.path.normpath(os.path.join("MDK-ARM", p)).replace(os.sep, "/")
        out.append(p)
    return out


def main() -> int:
    kind = sys.argv[1] if len(sys.argv) > 1 else "c"
    files = project_files()

    if kind == "c":
        res = []
        for p in files:
            if p.endswith(".c"):
                if "/portable/RVDS/" in p:
                    res.append(GCC_PORT)
                else:
                    res.append(p)
        print("\n".join(sorted(set(res))))
    else:
        res = []
        for p in files:
            if p.endswith(".s"):
                res.append(STARTUP_REPLACE.get(p, p))
        print("\n".join(res))
    return 0


if __name__ == "__main__":
    sys.exit(main())
