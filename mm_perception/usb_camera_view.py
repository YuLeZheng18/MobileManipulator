#!/usr/bin/env python3
"""显示 USB 相机画面的可视化窗口。

用法:
    python3 usb_camera_view.py            # 默认打开设备 0
    python3 usb_camera_view.py --device 1 # 指定设备号
    python3 usb_camera_view.py -d /dev/video2 --width 1280 --height 720

窗口内按 q 或 Esc 退出, 按 s 保存当前帧为 png。
"""
import argparse
import sys
import time

import cv2


def parse_args():
    p = argparse.ArgumentParser(description="显示 USB 相机画面")
    p.add_argument("-d", "--device", default="0",
                   help="设备号 (0/1/...) 或路径 (/dev/video0)")
    p.add_argument("--width", type=int, default=640, help="采集宽度")
    p.add_argument("--height", type=int, default=480, help="采集高度")
    p.add_argument("--fps", type=int, default=0, help="请求帧率, 0 为不设置")
    return p.parse_args()


def open_camera(device, width, height, fps):
    # 数字字符串按索引打开, 否则按设备路径打开
    src = int(device) if str(device).isdigit() else device
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps > 0:
        cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def main():
    args = parse_args()
    cap = open_camera(args.device, args.width, args.height, args.fps)
    if cap is None:
        print(f"[错误] 无法打开相机: {args.device}", file=sys.stderr)
        print("检查: 相机是否插好, ls /dev/video*, 是否被其他程序占用", file=sys.stderr)
        return 1

    win = "USB Camera (q/Esc 退出, s 保存)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    prev = time.time()
    fps_show = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[警告] 读取帧失败, 退出", file=sys.stderr)
            break

        frame = cv2.flip(frame, 0)  # 上下翻转

        now = time.time()
        dt = now - prev
        prev = now
        if dt > 0:
            fps_show = 0.9 * fps_show + 0.1 * (1.0 / dt)
        cv2.putText(frame, f"{fps_show:4.1f} FPS", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):  # q 或 Esc
            break
        if key == ord("s"):
            fname = f"capture_{int(now)}.png"
            cv2.imwrite(fname, frame)
            print(f"已保存: {fname}")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
