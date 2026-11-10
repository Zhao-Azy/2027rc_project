#!/usr/bin/env bash
# 在家目录里装好 arm-none-eabi-gcc（不需要 root），供 dm_demo/Makefile 使用。
#
# 为什么不用 sudo apt install：
#   1) 这台机器没有免密 sudo；
#   2) 系统 apt 源指向 noble(24.04)，而系统是 22.04(glibc 2.35)，
#      直接装 noble 的包会因为 glibc 版本不够而跑不起来。
# 所以这里显式从 jammy(22.04) 源下载、解包到 ~/.local/opt/arm-gnu，全程不碰系统目录。
#
# 用法： bash scripts/setup_toolchain.sh
set -uo pipefail

DEST="$HOME/.local/opt/arm-gnu"
TMP="$(mktemp -d)"
MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ubuntu"
SUITE="jammy"

info() { printf '[信息] %s\n' "$*"; }
err()  { printf '[错误] %s\n' "$*" >&2; }

for cmd in curl apt-get dpkg; do
    command -v "$cmd" >/dev/null 2>&1 || { err "缺少命令：$cmd"; exit 1; }
done

info "准备临时 apt 源（$SUITE）"
mkdir -p "$TMP/lists/partial" "$TMP/cache/archives/partial"
echo "deb $MIRROR $SUITE main universe" > "$TMP/sources.list"
APTOPT=(-o "Dir::Etc::sourcelist=$TMP/sources.list"
        -o "Dir::Etc::sourceparts=/dev/null"
        -o "Dir::State::Lists=$TMP/lists"
        -o "Dir::Cache=$TMP/cache"
        -o "APT::Get::List-Cleanup=0")

info "更新包索引（约 47MB）"
apt-get "${APTOPT[@]}" update >/dev/null || { err "apt update 失败"; exit 1; }

info "下载工具链（约 90MB）"
( cd "$TMP" && apt-get "${APTOPT[@]}" download \
      gcc-arm-none-eabi binutils-arm-none-eabi \
      libnewlib-arm-none-eabi libnewlib-dev ) || { err "下载失败"; exit 1; }

info "解包到 $DEST"
rm -rf "$DEST"
mkdir -p "$DEST"
for d in "$TMP"/*.deb; do
    dpkg -x "$d" "$DEST" || { err "解包失败：$d"; exit 1; }
done

# Ubuntu 把这些东西拆在两个包里，路径也不是 GCC 默认查找的位置，这里补齐：
#   · newlib 头文件在 /usr/include/newlib  -> 编译时用 -isystem 加进 Makefile
#   · newlib 库在 /usr/lib/arm-none-eabi/newlib，但 GCC 默认找 .../arm-none-eabi/lib
ln -sfn newlib "$DEST/usr/lib/arm-none-eabi/lib"

rm -rf "$TMP"

info "验证"
"$DEST/usr/bin/arm-none-eabi-gcc" --version | head -1
printf '\n完成。现在可以编译：\n  bash scripts/build.sh\n  （或 cd dm_demo && make -j8）\n'
