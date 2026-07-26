# Nano 串口设备 udev 固定软链规则

真机(Jetson Orin NX/Nano)上给串口设备配固定软链, 免 `ttyUSB*`/`ttyACM*` 随插拔顺序漂移。
launch 里 `--dev` / `serial_port` 用软链名, 不用裸设备号。

| 规则文件 | 设备 | VID:PID | 软链 |
|---|---|---|---|
| `99-rplidar.rules` | 思岚 A3 雷达 (经 CP2102 USB-TTL) | 10c4:ea60 | `/dev/rplidar` |
| `99-esp32-chassis.rules` | ESP32-S3 底盘下位机 (micro-ROS) | 303a:1001 | `/dev/esp32_chassis` |

## 部署到 Nano

```bash
sudo cp mm_bringup/udev/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
# 验证
ls -l /dev/rplidar /dev/esp32_chassis
```

## 备注

- **雷达为何走 CP2102 USB-TTL 而非排针 UART**: Jetson 内核无 CH340(ch341) 驱动;
  且 40 针排针 UART(ttyTHS1) 在 256000 波特下信号完整性不足(实测残缺乱码)。
  CP2102 内核自带 cp210x 驱动, 即插即用, 稳定出 /scan。A3 官方套件本就配 CP2102。
- **CP2102 序列号不唯一**(出厂常为 0001), 故按 VID/PID 匹配; 若将来再插第二个
  CP2102 设备, 需改用 `ATTRS{serial}` 区分。
- 雷达接线(已验证): 方向需交叉(雷达TX↔NanoRX), 波特 256000, 独立供电但信号地
  必须与 TX/RX 一起接(仅供电端共地不够, 高波特下 RX 会塌成低电平)。
