#!/bin/bash
# 手动安装 PEAK PCAN chardev 驱动 + 库(已用 gcc-12 编译好)
set -e
SRC=/tmp/pcan_build/peak-linux-driver-8.20.0
KMOD_DIR=/lib/modules/$(uname -r)/kernel/drivers/misc

echo "[1/6] 安装内核模块 pcan.ko ..."
install -d "$KMOD_DIR"
# 先删掉系统里任何旧 pcan.ko
find /lib/modules/$(uname -r) -name 'pcan.ko*' -delete 2>/dev/null || true
install -m 644 "$SRC/driver/pcan.ko" "$KMOD_DIR/pcan.ko"
echo "    -> $KMOD_DIR/pcan.ko"

echo "[2/6] 安装运行库到 /usr/lib ..."
install -m 755 "$SRC/lib/lib/libpcan.so.6"                       /usr/lib/libpcan.so.6
install -m 755 "$SRC/lib/lib/libpcanfd.so.8"                     /usr/lib/libpcanfd.so.8
install -m 755 "$SRC/libpcanbasic/pcanbasic/lib/libpcanbasic.so.4.10.0" /usr/lib/libpcanbasic.so.4.10.0
# 建立 .so / 主版本号软链接(LoadLibrary("libpcanbasic.so") 需要)
ln -sf libpcan.so.6            /usr/lib/libpcan.so
ln -sf libpcanfd.so.8         /usr/lib/libpcanfd.so
ln -sf libpcanbasic.so.4.10.0 /usr/lib/libpcanbasic.so.4
ln -sf libpcanbasic.so.4      /usr/lib/libpcanbasic.so
echo "    -> /usr/lib/libpcanbasic.so"

echo "[3/6] 安装头文件 ..."
install -m 644 "$SRC/driver/pcan.h"   /usr/include/pcan.h
install -m 644 "$SRC/driver/pcanfd.h" /usr/include/pcanfd.h

echo "[4/6] 安装 udev 规则 ..."
install -m 644 "$SRC/driver/udev/45-pcan.rules" /etc/udev/rules.d/45-pcan.rules

echo "[5/6] 屏蔽内核自带 peak_usb (与 chardev 驱动冲突) ..."
cat > /etc/modprobe.d/blacklist-peak.conf <<'EOF'
# PEAK chardev 驱动接管,屏蔽 mainline SocketCAN 驱动
blacklist peak_usb
blacklist peak_pci
blacklist peak_pciefd
EOF
rmmod peak_usb 2>/dev/null || true

echo "[6/6] 刷新模块依赖与库缓存 ..."
depmod -a
ldconfig
udevadm control --reload-rules 2>/dev/null || true

echo "=== 验证 ==="
echo "-- pcan.ko 已装:"; find /lib/modules/$(uname -r) -name pcan.ko
echo "-- 库已注册:"; ldconfig -p | grep -i pcan
echo "-- peak_usb 当前状态:"; lsmod | grep peak_usb || echo "peak_usb 未加载(正确)"
echo "安装完成。"
