#!/usr/bin/env bash
# 用 ST-Link + st-flash 把 R1.hex 写到 STM32H723。
#
# 用法：
#   scripts/flash.sh            # 正常烧录（会先检查 hex 是否比源码新）
#   scripts/flash.sh --force    # 跳过时间戳检查，强制烧现有 hex
#   scripts/flash.sh --probe    # 只检查 ST-Link 是否连上，不烧录
#
# 退出码：0 成功；1 环境/硬件问题；2 参数错误
set -uo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
HEX="$ROOT/dm_demo/MDK-ARM/R1/R1.hex"
FORCE=0
PROBE_ONLY=0

while [ $# -gt 0 ]; do
    case "$1" in
        --force) FORCE=1 ;;
        --probe) PROBE_ONLY=1 ;;
        --hex)   shift; [ $# -gt 0 ] || { echo "[错误] --hex 后面要跟路径"; exit 2; }; HEX="$1" ;;
        -h|--help) sed -n '2,10p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "[错误] 未知参数：$1"; exit 2 ;;
    esac
    shift
done

info() { printf '[信息] %s\n' "$*"; }
warn() { printf '[警告] %s\n' "$*"; }
err()  { printf '[错误] %s\n' "$*" >&2; }

# ---------- 1. 找工具（VS Code 里的 PATH 可能不含 ~/.local/bin） ----------
find_tool() {
    local name="$1"
    if command -v "$name" >/dev/null 2>&1; then
        command -v "$name"
        return 0
    fi
    for p in "$HOME/.local/bin/$name" "$HOME/.local/opt/stlink/usr/bin/$name"; do
        if [ -x "$p" ]; then
            echo "$p"
            return 0
        fi
    done
    return 1
}

ST_FLASH=$(find_tool st-flash) || true
ST_INFO=$(find_tool st-info) || true
if [ -z "${ST_FLASH:-}" ] || [ -z "${ST_INFO:-}" ]; then
    err "找不到 st-flash / st-info。安装：sudo apt install stlink-tools"
    exit 1
fi

# ---------- 2. 检查 ST-Link ----------
probe_out=$("$ST_INFO" --probe 2>&1)
if ! printf '%s\n' "$probe_out" | grep -q "Found [1-9]"; then
    err "没有检测到 ST-Link。下面依次排查："
    err "  1) ST-Link 是否插好、板上是否上电（USB 供电即可）"
    err "  2) 是否被占用：关闭 Keil / STM32CubeProgrammer 后重试"
    err "  3) 换个 USB 口 / 换根线（要能传数据，不是只有电的线）"
    err "  4) 检查权限：当前用户应在 plugdev 组，udev 规则 49-stlink*.rules 应在 /etc/udev/rules.d/"
    echo "       探测输出：$probe_out" >&2
    exit 1
fi
info "ST-Link 已连接：$(printf '%s\n' "$probe_out" | grep 'Found')"

[ "$PROBE_ONLY" -eq 1 ] && exit 0

# ---------- 3. 检查 hex 与源码新旧 ----------
if [ ! -f "$HEX" ]; then
    err "找不到 hex：$HEX"
    err "先把工程编译一次（Keil Build），或确认输出目录是 dm_demo/MDK-ARM/R1/"
    exit 1
fi

stale=$(find "$ROOT/dm_demo" \( -name '*.c' -o -name '*.h' \) \
        -newer "$HEX" -not -path '*/MDK-ARM/R1/*' 2>/dev/null | head -5)
if [ -n "$stale" ] && [ "$FORCE" -eq 0 ]; then
    err "这个 hex 比下面的源码旧，烧进去不含它们的改动（只列前 5 个）："
    printf '%s\n' "$stale" | while IFS= read -r f; do
        printf '        · %s\n' "${f#"$ROOT"/}" >&2
    done
    err "先重新编译（scripts/build.sh 或 Windows 下的 Keil）；确定要烧旧 hex 就加 --force"
    exit 1
fi
[ -n "$stale" ] && warn "跳过时间戳检查（--force），可能烧的是旧程序"

# ---------- 4. 烧录 ----------
info "写入：$(basename "$HEX")（$(stat -c %y "$HEX" | cut -d. -f1)）"
if ! "$ST_FLASH" --format ihex write "$HEX"; then
    err "烧录失败。常见原因：芯片被读保护(option bytes)、SWD 线太长、供电不稳"
    exit 1
fi

info "复位运行"
"$ST_FLASH" reset || warn "复位命令失败，手动断电重上电即可"

info "烧录完成 ✓"
