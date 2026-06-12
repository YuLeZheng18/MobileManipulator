# PCAN 机械臂 CAN 驱动配置指南

记录机械臂控制 GUI(`arm_control/joint_gui`)所依赖的 PEAK PCAN-USB 驱动环境的搭建过程,供换内核 / 重装系统时复现。

## 背景

- 控制代码走的是 **PEAK chardev API**(`PCANBasic.py` 在 Linux 上 `cdll.LoadLibrary("libpcanbasic.so")`),不是内核 SocketCAN。
- 因此必须安装 PEAK 官方 `peak-linux-driver`(chardev 模式)+ `libpcanbasic.so`,**不能**用内核自带的 `peak_usb`。
- 硬件:PEAK PCAN-USB 经典款,USB ID `0c72:000c`(非 FD)。代码默认通道 `PCAN_USBBUS1`、波特率 `PCAN_BAUD_500K`。

## 环境前提

| 项 | 要求 | 检查命令 |
|----|------|----------|
| 内核头文件 | `linux-headers-$(uname -r)` | `ls /lib/modules/$(uname -r)/build` |
| 编译器 | 必须与编译内核的 gcc 主版本一致 | 见下方「gcc 版本坑」 |
| Python 依赖 | PyQt5、pyqtgraph | `python3 -c "import PyQt5, pyqtgraph"` |

> **gcc 版本坑(关键)**:本机内核 `6.8.0-124-generic` 由 **gcc-12** 编译,而默认 `gcc` 指向 gcc-11。直接 `make` 会报
> `gcc: error: unrecognized command-line option '-ftrivial-auto-var-init=zero'`。
> 编译时必须显式指定 `CC=gcc-12`。

## 安装步骤

### 1. 下载驱动

实测官网最新可用稳定版是 **8.20.0**(8.20.2 / 8.23.0 的链接 404)。

```bash
cd /tmp && mkdir -p pcan_build && cd pcan_build
curl -O https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-8.20.0.tar.gz
tar xzf peak-linux-driver-8.20.0.tar.gz
cd peak-linux-driver-8.20.0
```

### 2. 编译(用 gcc-12,跳过 test/examples)

`make all` 会连 test(需 `libpopt-dev`)和 examples(需 `g++-12` 这个可执行名)一起编,本机两者都缺会报错。只编译三个核心组件即可:

```bash
make -C driver       CC=gcc-12          all   # -> driver/pcan.ko
make -C lib          CC=gcc-12          all   # -> lib/lib/libpcan.so.6, libpcanfd.so.8
make -C libpcanbasic CC=gcc-12 CXX=g++-12 all # -> libpcanbasic/pcanbasic/lib/libpcanbasic.so.4.10.0
```

> libpcanbasic 编译末尾会报 examples 的 `g++-12: 没有那个文件或目录`——**可忽略**,库本身已生成。

验证 `pcan.ko` 的 vermagic 与当前内核一致:

```bash
modinfo driver/pcan.ko | grep vermagic   # 应为 6.8.0-124-generic ...
```

### 3. 安装(需 root)

PEAK 的 `make install` 在本机会因 gcc 问题触发静默重编而失败,改用手动安装。安装脚本见本仓库 `src/docs/scripts/install_pcan.sh`,执行:

```bash
sudo bash src/docs/scripts/install_pcan.sh
```

脚本做的事:
1. 装 `pcan.ko` 到 `/lib/modules/$(uname -r)/kernel/drivers/misc/`
2. 装 `libpcan.so.6` / `libpcanfd.so.8` / `libpcanbasic.so.4.10.0` 到 `/usr/lib`,并建 `.so` 软链接
3. 装头文件 `pcan.h` / `pcanfd.h` 到 `/usr/include`
4. 装 udev 规则 `45-pcan.rules`
5. **屏蔽内核 `peak_usb`**(写 `/etc/modprobe.d/blacklist-peak.conf` 并 `rmmod peak_usb`)——否则它会抢占 USB 设备
6. `depmod -a` + `ldconfig`

### 4. 加载驱动并接管设备

```bash
sudo modprobe pcan
# 把 PCAN-USB 拔下再插回(让 chardev 驱动接管)
ls -l /dev/pcanusb*    # 应出现 /dev/pcanusb32
cat /proc/pcan         # 应列出 1 个 usb 接口
```

## 验证

- 节点:`/dev/pcanusb32` 存在,权限 `crw-rw-rw-`(普通用户可读写,**跑 GUI 不需 sudo**)。
- `/proc/pcan` 中能看到该 usb 接口。`--btr-` 栏在连接前是占位值 `0x001c`,**波特率 500K 在 GUI 点「连接 CAN」时由 `can.initialize()` 写入**,连接后该栏及 read/write 计数会变化。
- 启动 GUI:
  ```bash
  source install/setup.bash
  ros2 run arm_control joint_gui
  ```
  点「连接 CAN」→ 状态栏变绿 `CAN: 已连接 | 电机已使能`。

## 换内核 / 重装系统后

`pcan.ko` 绑定具体内核版本。内核升级后需 **重新执行第 2~4 步**(重编 + 重装 + modprobe)。
若 `/dev/pcanusb*` 不出现,先确认 `lsmod | grep peak_usb` 为空(被屏蔽),必要时 `sudo rmmod peak_usb && sudo modprobe pcan`。
