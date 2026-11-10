#!/usr/bin/env bash
# 编译 R1 固件，产出 dm_demo/MDK-ARM/R1/R1.hex（顺带 dm_demo/build/R1.elf）。
#
# 本机用的路线（已实测可编译 + 可烧录）：
#   arm-none-eabi-gcc + dm_demo/Makefile
#   工具链装在家目录里（不需要 root）：~/.local/opt/arm-gnu
#   没装就先跑 scripts/setup_toolchain.sh
#
# 第二条路线是 wine + Keil UV4，本机 wine 没装 Mono，跑不通，仅作保底提示。
#
# 退出码：0 编译成功；1 编译失败或缺工具链
set -uo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DM="$ROOT/dm_demo"
HEX="$DM/MDK-ARM/R1/R1.hex"
LOCAL_TC="$HOME/.local/opt/arm-gnu/usr/bin"

info() { printf '[信息] %s\n' "$*"; }
warn() { printf '[警告] %s\n' "$*"; }
err()  { printf '[错误] %s\n' "$*" >&2; }

# ---------- 0. 工程一致性检查（漏加文件 / 引用缺失） ----------
if command -v python3 >/dev/null 2>&1; then
    python3 "$ROOT/scripts/check_project.py" || warn "工程检查有问题，编译大概率会失败，见上面的明细"
    echo
fi

# ---------- 1. GCC 路线（本机主力） ----------
if [ -x "$LOCAL_TC/arm-none-eabi-gcc" ]; then
    export PATH="$LOCAL_TC:$PATH"
elif command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    info "使用 PATH 里的 arm-none-eabi-gcc：$(command -v arm-none-eabi-gcc)"
else
    err "没找到 arm-none-eabi-gcc。先跑：bash scripts/setup_toolchain.sh"
fi

if command -v arm-none-eabi-gcc >/dev/null 2>&1 && [ -f "$DM/Makefile" ]; then
    info "开始编译（make -C dm_demo）"
    if make -C "$DM" -j"$(nproc)"; then
        info "编译完成：${HEX#$ROOT/}"
        exit 0
    fi
    err "编译失败，看上面的报错"
    exit 1
fi

# ---------- 2. wine + Keil 路线（备选） ----------
UV4_WIN="${KEIL_UV4:-C:\\users\\UV4\\UV4.exe}"
UV4_LIN="$HOME/.wine/drive_c/users/UV4/UV4.exe"
CALLER="$HOME/.vscode/extensions/sthudy.keil-assistant-linux-2.0.0/bin/Uv4Caller.exe"

if command -v wine >/dev/null 2>&1 && [ -f "$UV4_LIN" ]; then
    if [ ! -d "$HOME/.wine/drive_c/windows/mono" ]; then
        warn "wine 里没装 Mono，扩展自带的 Uv4Caller.exe（.NET 程序）跑不起来"
    else
        info "尝试用 wine + UV4 批处理编译"
        ( cd "$DM/MDK-ARM" && \
          WINEDEBUG=-all wine "$CALLER" -o "$DM/MDK-ARM/uv4.log" \
              --uv4Path "$UV4_WIN" --prjPath "$DM/MDK-ARM/R1.uvprojx" --targetName R1 \
              -c "$UV4_WIN -f $DM/MDK-ARM/R1.uvprojx -j$(nproc) -t R1" ) && { info "编译完成"; exit 0; }
        err "wine 编译失败，看看 dm_demo/MDK-ARM/uv4.log"
    fi
fi

# ---------- 3. 都不行 ----------
cat <<'EOF' >&2

 没法在本机编译。可选做法：
 A. 装本机 GCC 工具链（不需要 root，约 100MB 下载）：
      bash scripts/setup_toolchain.sh
 B. 在 Windows + Keil 上编译，再把 R1.hex 拷回 dm_demo/MDK-ARM/R1/

 编译出新的 R1.hex 之后，用下面的命令烧录：
      bash scripts/flash.sh        # 或 VS Code 里 Ctrl+Shift+B
EOF
exit 1
