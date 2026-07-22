#!/usr/bin/env python3
"""YOLO 盒子识别 + 抓取位姿 (彩色检测 + 对齐深度定位, 契约 §1 抓取版).

分工:
  - YOLO 在彩色图上出目标框, 负责 "是不是盒子 / 在哪" (比纯深度分割更能滤掉
    手/支架等非盒子凸起);
  - 框内 ROI 在**对齐深度** (aligned_depth_to_color, 与彩色逐像素对齐) 上做深度
    分割 + minAreaRect 长轴, 负责 "顶面中心精确 xyz + 绕竖直轴 yaw".

数据流:
  订阅 color(rgb8) + aligned_depth_to_color(16UC1, 同分辨率同视角) + color camera_info
    -> YOLO 推理彩色图, 出目标框 (可按类别名过滤)
    -> 每框 ROI 内: 估台面深度 -> 取盒子顶面掩码 -> 最大轮廓 minAreaRect
    -> 顶面中心像素取对齐深度中值 d -> 反投影(光学系)-> 左乘固定旋转到 Link_30 机械系
    -> 查 TF Link_30->base_link 转到底盘系, 得 xyz
    -> minAreaRect 长轴两端同深反投影到 base_link 估 yaw (契约 §1 第 4 自由度)
    -> 发 /perception/object_pose (PoseStamped, base_link 系); 可视化画框/中心/朝向

抓取模型 (契约 §1): 4-DOF top-down. 吸盘接近方向恒竖直向下 (沿 Link_29 -Z),
  roll=pitch=0, 只有 x y z + yaw 变化. 本节点只输出盒子顶面中心 xyz + yaw, 打包成
  PoseStamped; 末端/吸盘偏置换算归任务层 (grasp_node).

坐标系约定 (与 box_detector / aruco_localizer 同源):
  反投影结果在相机光学系 (REP-104: z 朝前, x 朝右, y 朝下). URDF 的 Link_30 是相机
  机械系, 故默认 apply_optical_rotation=True: 左乘固定旋转 (rpy=-pi/2,0,-pi/2) 把光学系
  点搬到 Link_30 机械系, 再用 TF 转到 base_link.

内参说明:
  对齐深度与彩色共用彩色相机内参. 若本机彩色标定表损坏 (camera_info K=NaN),
  use_fallback_on_nan=True 时用估计内参兜底先跑通链路 (会告警), 但**距离/yaw 有系统
  误差**, 要达抓取精度须用 camera_calibration 重标定彩色相机后 K 才有效.

依赖:
  - 深度相机驱动 (RealSense) 已起, 发 color/image_raw + aligned_depth_to_color/image_raw
    + color/camera_info;
  - 整车 robot_state_publisher 已起 (TF 树含 Link_30 -> ... -> base_link), 否则查不到
    TF 只打印像素/深度, 不发 pose;
  - 通用 yolov8n.pt (COCO) 无 "盒子" 类, 仅跑通链路; 换自训练模型改 model_path,
    并用 class_names 过滤出盒子类.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import tf2_ros

# 必须在 import ultralytics 前注入 torchvision 兜底 (Jetson 系统 GPU torch 无匹配
# torchvision, 见 _tv_stub). 用系统 GPU torch 推理, 比 venv CPU 快 ~10x.
from mm_perception._tv_stub import install_torchvision_stub
install_torchvision_stub()
from ultralytics import YOLO  # noqa: E402

# 复用既有节点的图像解码与几何约定 (同一套坐标系约定, 避免重复实现)
from mm_perception.aruco_localizer import (
    imgmsg_to_bgr, rpy_to_rot_matrix, _OPTICAL_TO_MECH_RPY)
from mm_perception.box_detector import depthmsg_to_meters
from mm_perception.hand_eye_calibrator import quat_to_rot_matrix


class YoloBoxDetector(Node):
    def __init__(self):
        super().__init__('yolo_box_detector')

        # ---- 模型 ----
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf', 0.25)            # YOLO 置信度阈值
        self.declare_parameter('imgsz', 640)            # 推理输入尺寸, 越小越快
        self.declare_parameter('device', '0')           # '0'=GPU0, 'cpu'=CPU
        # 只保留这些类名 (自训练盒子模型填 ['box'] 等); 留空 [] = 不过滤(跑通用).
        self.declare_parameter('class_names', [''])

        # ---- 话题 (RealSense 默认命名空间 camera/camera) ----
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        # 深度流: 本机彩色标定坏(K/外参 NaN)导致 aligned_depth 全废, 故走原始深度流.
        #   use_raw_depth=True: 用 depth/image_rect_raw(与彩色不同分辨率/视角), 节点内
        #     用彩色内参->视线角->深度内参 近似映射彩色框到深度图(忽略几 cm 平移).
        #   use_raw_depth=False: 用 aligned_depth(与彩色逐像素对齐), 彩色 K 有效时更准.
        self.declare_parameter('use_raw_depth', True)
        self.declare_parameter('depth_topic',
                               '/camera/camera/depth/image_rect_raw')
        # 彩色内参: 仅用于把彩色框中心转成视线角(NaN 时用 fallback 兜底).
        self.declare_parameter('camera_info_topic',
                               '/camera/camera/color/camera_info')
        # 深度内参: 反投影/yaw 全用它(有效 K). raw 模式必需.
        self.declare_parameter('depth_info_topic',
                               '/camera/camera/depth/camera_info')
        self.declare_parameter('depth_is_best_effort', True)

        # ---- 坐标系 ----
        self.declare_parameter('camera_frame', 'Link_30')   # 深度相机 URDF link
        self.declare_parameter('base_frame', 'base_link')   # 目标系 (契约 §1)
        self.declare_parameter('apply_optical_rotation', True)  # 光学系->Link_30 机械系
        # 相机绕视线轴(光学Z)实际安装转角修正(度). 实测: 物理前->base -Y, 物理左->base +X,
        # 即投影绕竖直轴偏了90°. 在光学系补此旋转纠正. 装反了改±90/180. 0=不修正.
        self.declare_parameter('optical_roll_deg', -90.0)

        # ---- ROI 内深度分割 (相对台面取盒子顶面) ----
        self.declare_parameter('min_depth', 0.15)       # 有效深度下限(米)
        self.declare_parameter('max_depth', 3.0)        # 有效深度上限(米)
        self.declare_parameter('min_box_height', 0.02)  # 高于台面多少算盒子(米)
        self.declare_parameter('max_box_height', 0.5)   # 高出台面超此值视为噪声(米)
        self.declare_parameter('center_patch', 9)       # 取中心深度的邻域边长(像素,中值更稳)
        self.declare_parameter('roi_shrink', 0.08)      # ROI 四周向内收缩比例, 避开框边噪声
        self.declare_parameter('min_mask_px', 300)      # ROI 内盒子掩码最小像素, 太少判无效

        # ---- 内参兜底 (本机彩色 K 若为 NaN) ----
        self.declare_parameter('use_fallback_on_nan', True)
        self.declare_parameter('fallback_fx', 900.0)    # 1280x720 粗估, 仅跑通链路
        self.declare_parameter('fallback_fy', 900.0)

        # ---- 输出 (契约 §1) ----
        self.declare_parameter('publish_pose', True)
        self.declare_parameter('pose_topic', '/perception/object_pose')
        # 多目标: 所有盒子位姿数组 (PoseArray). 单目标 pose_topic 仍恒发最大框.
        self.declare_parameter('poses_topic', '/perception/object_poses')
        self.declare_parameter('only_largest', True)    # 单目标: 只发最大框

        # ---- 可视化 ----
        self.declare_parameter('show_window', True)

        gp = self.get_parameter
        self.conf = float(gp('conf').value)
        self.imgsz = int(gp('imgsz').value)
        self.device = str(gp('device').value)
        self.class_names = [s for s in gp('class_names').value if s]
        self.camera_frame = gp('camera_frame').value
        self.base_frame = gp('base_frame').value
        self.apply_optical_rotation = bool(gp('apply_optical_rotation').value)
        self.min_depth = float(gp('min_depth').value)
        self.max_depth = float(gp('max_depth').value)
        self.min_box_height = float(gp('min_box_height').value)
        self.max_box_height = float(gp('max_box_height').value)
        self.center_patch = max(1, int(gp('center_patch').value))
        self.roi_shrink = min(max(float(gp('roi_shrink').value), 0.0), 0.4)
        self.min_mask_px = int(gp('min_mask_px').value)
        self.use_fallback_on_nan = bool(gp('use_fallback_on_nan').value)
        self.fallback_fx = float(gp('fallback_fx').value)
        self.fallback_fy = float(gp('fallback_fy').value)
        self.use_raw_depth = bool(gp('use_raw_depth').value)
        self.publish_pose = bool(gp('publish_pose').value)
        self.only_largest = bool(gp('only_largest').value)
        self.show_window = bool(gp('show_window').value)

        # 预计算 光学系 -> Link_30 机械系 的固定旋转 (与 box_detector 同一约定)
        # 再右乘绕光学Z轴的安装转角修正: p_mech = R_mech_optical @ Rz(roll) @ p_opt.
        # 右乘=先在光学系内绕视线轴转正, 等效纠正相机绕视线轴装偏的角度.
        self._R_mech_optical = rpy_to_rot_matrix(*_OPTICAL_TO_MECH_RPY)
        roll = np.radians(float(gp('optical_roll_deg').value))
        cz, sz = np.cos(roll), np.sin(roll)
        Rz_opt = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
        self._R_mech_optical = self._R_mech_optical @ Rz_opt

        # ---- 模型加载 ----
        # model_task: NCNN/ONNX 等导出格式无法自动识别 task, 加载会误判成 detect
        #   (-> res.obb 变 None). 指到 *_ncnn_model 目录时必须显式传 task='obb'.
        #   .pt 原生权重能自带 task, 留空即可.
        self.declare_parameter('model_task', '')
        model_path = gp('model_path').value
        model_task = str(gp('model_task').value) or None
        self.get_logger().info('加载 YOLO 模型: %s (task=%s) ...'
                               % (model_path, model_task or '自动'))
        self.model = YOLO(model_path, task=model_task) if model_task \
            else YOLO(model_path)
        self.get_logger().info('YOLO 就绪, task=%s, 类别数=%d'
                               % (self.model.task, len(self.model.names)))

        # ---- 数据缓存 ----
        self._camera_matrix = None    # 彩色 3x3 K (框中心->视线角; NaN 时兜底)
        self._depth_matrix = None     # 深度 3x3 K (反投影/yaw 用; raw 模式必需)
        self._latest_depth = None     # 最新深度图(米)

        # ---- QoS ----
        depth_qos = QoSProfile(
            reliability=(ReliabilityPolicy.BEST_EFFORT
                         if bool(gp('depth_is_best_effort').value)
                         else ReliabilityPolicy.RELIABLE),
            history=HistoryPolicy.KEEP_LAST, depth=1)
        info_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=1)

        self._info_sub = self.create_subscription(
            CameraInfo, gp('camera_info_topic').value, self._on_info, info_qos)
        self.create_subscription(Image, gp('depth_topic').value, self._on_depth, depth_qos)
        self.create_subscription(Image, gp('color_topic').value, self._on_color, depth_qos)
        # raw 模式: 额外订阅深度内参 (反投影/yaw 用有效 K, 不用坏掉的彩色 K)
        self._depth_info_sub = None
        if self.use_raw_depth:
            self._depth_info_sub = self.create_subscription(
                CameraInfo, gp('depth_info_topic').value, self._on_depth_info, info_qos)

        # ---- TF ----
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- 位姿发布 (契约 §1) ----
        self._pose_pub = None
        self._poses_pub = None
        if self.publish_pose:
            self._pose_pub = self.create_publisher(
                PoseStamped, gp('pose_topic').value, 10)
            # 多目标数组: 所有有 base_link 坐标的盒子
            self._poses_pub = self.create_publisher(
                PoseArray, gp('poses_topic').value, 10)

        self.get_logger().info(
            'yolo_box_detector(抓取版) 就绪: 深度模式=%s, camera_frame=%s -> %s, 发位姿=%s, 类过滤=%s'
            % ('原始深度' if self.use_raw_depth else 'aligned',
               self.camera_frame, self.base_frame, self.publish_pose,
               self.class_names or '无'))

    # ---------------- 回调 ----------------

    def _on_info(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if np.all(np.isfinite(K)) and K[0, 0] > 0:
            self._camera_matrix = K
            self.get_logger().info('已获取相机内参 (fx=%.1f fy=%.1f).'
                                   % (K[0, 0], K[1, 1]))
            if self._info_sub is not None:      # 内参有效即退订
                self.destroy_subscription(self._info_sub)
                self._info_sub = None
        elif self.use_fallback_on_nan and self._camera_matrix is None:
            cx = (msg.width or 1280) / 2.0
            cy = (msg.height or 720) / 2.0
            self._camera_matrix = np.array([[self.fallback_fx, 0.0, cx],
                                            [0.0, self.fallback_fy, cy],
                                            [0.0, 0.0, 1.0]])
            self._warn_throttle(
                'camera_info K 无效(NaN), 用估计内参兜底 fx=%.1f fy=%.1f cx=%.1f cy=%.1f '
                '—— 距离/yaw 有系统误差, 抓取前须重标定彩色相机.'
                % (self.fallback_fx, self.fallback_fy, cx, cy))

    def _on_depth_info(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if np.all(np.isfinite(K)) and K[0, 0] > 0:
            self._depth_matrix = K
            self.get_logger().info('已获取深度内参 (fx=%.1f fy=%.1f cx=%.1f cy=%.1f).'
                                   % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
            if self._depth_info_sub is not None:    # 有效即退订
                self.destroy_subscription(self._depth_info_sub)
                self._depth_info_sub = None

    def _on_depth(self, msg: Image):
        try:
            self._latest_depth = depthmsg_to_meters(msg)
        except Exception as e:  # noqa: BLE001
            self._warn_throttle('深度图解码失败: %s' % e)

    def _color_px_to_depth_px(self, u, v):
        """彩色像素(u,v) -> 深度图像素. 用彩色内参转视线角(x/z,y/z), 再用深度内参投回.

        忽略彩色/深度模块间几 cm 平移 (近似, 近处有小误差). 本机彩色 K/外参 NaN,
        彩色 K 用 fallback 兜底; 深度 K 有效. 结果四舍五入到深度图整数像素.
        """
        Kc, Kd = self._camera_matrix, self._depth_matrix
        xn = (u - Kc[0, 2]) / Kc[0, 0]
        yn = (v - Kc[1, 2]) / Kc[1, 1]
        ud = Kd[0, 0] * xn + Kd[0, 2]
        vd = Kd[1, 1] * yn + Kd[1, 2]
        return int(round(ud)), int(round(vd))

    def _depth_px_to_color_px(self, ud, vd):
        """深度图像素 -> 彩色像素 (与 _color_px_to_depth_px 反向). 仅用于可视化画点."""
        Kc, Kd = self._camera_matrix, self._depth_matrix
        xn = (ud - Kd[0, 2]) / Kd[0, 0]
        yn = (vd - Kd[1, 2]) / Kd[1, 1]
        u = Kc[0, 0] * xn + Kc[0, 2]
        v = Kc[1, 1] * yn + Kc[1, 2]
        return int(round(u)), int(round(v))

    def _on_color(self, msg: Image):
        if self._camera_matrix is None:
            self._warn_throttle('等待 camera_info, 暂不处理...')
            return
        try:
            bgr = imgmsg_to_bgr(msg)
        except Exception as e:  # noqa: BLE001
            self._warn_throttle('彩色图解码失败: %s' % e)
            return
        depth = self._latest_depth
        if depth is None:
            self._warn_throttle('等待深度帧...')
            return
        if self.use_raw_depth and self._depth_matrix is None:
            self._warn_throttle('等待深度内参(depth/camera_info)...')
            return
        if not self.use_raw_depth and depth.shape[:2] != bgr.shape[:2]:
            self._warn_throttle(
                '深度(%dx%d)与彩色(%dx%d)分辨率不一致, 需 aligned_depth_to_color.'
                % (depth.shape[1], depth.shape[0], bgr.shape[1], bgr.shape[0]))
            return

        # YOLO 推理 -> 目标框 (可按类名过滤). 框坐标在彩色系.
        res = self.model.predict(bgr, conf=self.conf, imgsz=self.imgsz,
                                 device=self.device, verbose=False)[0]
        dets = self._collect_detections(res)
        if self.only_largest and dets:
            dets = [max(dets, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))]

        R, t = self._lookup_base_tf()            # 一帧查一次 TF, 复用给各框
        self._R_bc, self._t_bc = R, t            # 存给可视化画坐标系 (base<-cam)
        results = []
        for (x1, y1, x2, y2, name, cf, axis, corners, center) in dets:
            # raw 模式: 彩色框 -> 深度图坐标 (后续分割/反投影全在深度图+深度K上做)
            if self.use_raw_depth:
                dh, dw = depth.shape
                dx1, dy1 = self._color_px_to_depth_px(x1, y1)
                dx2, dy2 = self._color_px_to_depth_px(x2, y2)
                bx1, bx2 = sorted((dx1, dx2))
                by1, by2 = sorted((dy1, dy2))
                bx1, by1 = max(0, bx1), max(0, by1)
                bx2, by2 = min(dw, bx2), min(dh, by2)
            else:
                bx1, by1, bx2, by2 = x1, y1, x2, y2
            # OBB 框中心(彩色系)映射到深度图坐标, 作为中心点首选
            if center is not None and self.use_raw_depth:
                dc = self._color_px_to_depth_px(center[0], center[1])
            elif center is not None:
                dc = (int(round(center[0])), int(round(center[1])))
            else:
                dc = None
            pose = self._roi_to_pose(depth, bx1, by1, bx2, by2, R, t, axis, dc)
            # 保留彩色系框坐标 + 旋转框角点 (corners) 用于可视化 (画在彩色图上)
            results.append((x1, y1, x2, y2, name, cf, pose, corners))

        self._print_results(results)
        self._publish_pose(results, msg.header.stamp)
        if self.show_window:
            self._draw_and_show(bgr, results)

    # ---------------- 检测收集 ----------------

    def _collect_detections(self, res):
        """从 YOLO 结果收集 (x1,y1,x2,y2,name,conf,axis), 按 class_names 过滤.

        axis: OBB 长轴在彩色像素系的单位方向 (dx,dy), 供 yaw 用旋转框角度重投影;
              普通 detect 模型无旋转框时为 None (退回深度 minAreaRect 估 yaw).
        obb 模型: 用 res.obb.xywhr (cx,cy,w,h,rad); 轴对齐外接框由旋转角点算.
        """
        dets = []
        obb = getattr(res, 'obb', None)
        if obb is not None:              # obb 模型: 即便本帧 0 检测也不能落到 res.boxes(None)
            names = self.model.names
            for i in range(len(obb)):
                name = names[int(obb.cls[i])]
                if self.class_names and name not in self.class_names:
                    continue
                cx, cy, rw, rh, rad = (float(v) for v in obb.xywhr[i].tolist())
                # 旋转框四角 (彩色像素系): 给深度 ROI 分割算外接框 + 可视化画旋转框
                pts = cv2.boxPoints(((cx, cy), (rw, rh), float(np.rad2deg(rad))))
                x1, y1 = pts[:, 0].min(), pts[:, 1].min()
                x2, y2 = pts[:, 0].max(), pts[:, 1].max()
                # 长轴方向 (彩色像素系): 取长边角度. rad 是 w 边角度.
                ang = rad if rw >= rh else rad + np.pi / 2.0
                axis = (float(np.cos(ang)), float(np.sin(ang)))
                # center: OBB 框中心(彩色像素系), 中心点优先用它(比深度质心稳)
                dets.append((int(round(x1)), int(round(y1)),
                             int(round(x2)), int(round(y2)),
                             name, float(obb.conf[i]), axis,
                             pts.astype(np.int32), (cx, cy)))
            return dets
        # 兜底: 普通 detect 模型 (轴对齐框, 无旋转角/角点 -> axis/corners=None)
        for b in res.boxes:
            name = self.model.names[int(b.cls[0])]
            if self.class_names and name not in self.class_names:
                continue
            x1, y1, x2, y2 = (int(round(v)) for v in b.xyxy[0].tolist())
            center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)   # detect: 用框中心
            dets.append((x1, y1, x2, y2, name, float(b.conf[0]),
                         None, None, center))
        return dets

    # ---------------- ROI -> 位姿 ----------------

    def _roi_to_pose(self, depth, x1, y1, x2, y2, R, t, axis=None, center_px=None):
        """YOLO 框内深度分割 -> (中心像素 cu,cv, 深度 d, base_link xyz p, yaw, 相机系 xyz p_cam).

        axis: OBB 长轴在彩色像素系的单位方向 (dx,dy) 或 None. 有则 yaw 直接用该方向
              重投影 (旋转框比深度轮廓稳); None 退回 minAreaRect 深度轮廓估 yaw.
        center_px: OBB 框中心在深度图坐标 (u,v) 或 None. 中心点优先用它(比深度质心稳),
              仅当该点深度无效(空洞)时才回退到深度掩码轮廓质心.

        p_cam 是中心点在相机系(Link_30)的 3D 坐标, 纯几何不依赖 TF, 恒有效.
        p/yaw 依赖 TF, 未就绪时为 None. 无有效盒子掩码返回 None.
        """
        h, w = depth.shape
        # ROI 向内收缩, 避开框边背景/相邻物
        sx = int((x2 - x1) * self.roi_shrink)
        sy = int((y2 - y1) * self.roi_shrink)
        rx1, ry1 = max(0, x1 + sx), max(0, y1 + sy)
        rx2, ry2 = min(w, x2 - sx), min(h, y2 - sy)
        if rx2 - rx1 < 3 or ry2 - ry1 < 3:
            return None
        roi = depth[ry1:ry2, rx1:rx2]
        valid = (roi > self.min_depth) & (roi < self.max_depth)
        if not np.any(valid):
            return None

        # 台面深度: ROI 内有效深度直方图主峰 (盒子占框大半时用较远的峰更稳, 这里取
        # 中位数偏远侧: 台面在框内往往是最远的连续层). 简化用 75 分位近似台面.
        table = float(np.percentile(roi[valid], 75))
        # 盒子顶面: 比台面近 [min,max]_box_height (凸起朝相机 -> 深度更小)
        box_mask = (valid
                    & (roi < table - self.min_box_height)
                    & (roi > table - self.max_box_height))
        if int(np.count_nonzero(box_mask)) < self.min_mask_px:
            # ROI 内无明显凸起 (盒子几乎占满/贴台) -> 退化为整个有效 ROI 当盒子
            box_mask = valid
        mask_u8 = (box_mask.astype(np.uint8)) * 255
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, k)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, k)

        contours, _ = cv2.findContours(
            mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        c = c + np.array([[rx1, ry1]])           # 轮廓坐标搬回整图系

        M = cv2.moments(c)
        if M['m00'] <= 0:
            return None
        # 深度掩码轮廓质心 (兜底中心)
        cu = int(round(M['m10'] / M['m00']))
        cv_ = int(round(M['m01'] / M['m00']))
        # 中心点优先用 OBB 框中心(贴合检测框, 不受深度掩码残缺影响); 仅当其深度
        # 无效(空洞)时才回退到轮廓质心 —— "OBB中心 + 深度校验" 策略.
        if center_px is not None:
            ou, ov = center_px
            h, w = depth.shape
            if 0 <= ou < w and 0 <= ov < h:
                d_obb = self._center_depth(depth, ou, ov)
                if d_obb > 0.0:
                    cu, cv_ = ou, ov
        d = self._center_depth(depth, cu, cv_)   # 顶面中心深度(米)
        if d <= 0.0:
            return None

        p_cam = self._pixel_to_cam(cu, cv_, d)   # 相机系 3D 坐标, 不依赖 TF
        p_base = (R @ p_cam + t) if R is not None else None
        yaw = self._estimate_yaw(c, cu, cv_, d, R, axis)
        return (cu, cv_, d, p_base, yaw, p_cam)

    def _center_depth(self, depth, u, v):
        """取 (u,v) 邻域内有效深度中值. 无有效值返回 0."""
        r = self.center_patch // 2
        h, w = depth.shape
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        patch = depth[v0:v1, u0:u1]
        good = patch[(patch > self.min_depth) & (patch < self.max_depth)]
        return float(np.median(good)) if good.size else 0.0

    def _pixel_to_cam(self, u, v, d):
        """像素(u,v)+深度d -> camera_frame(Link_30 机械系) 3D 点. 纯几何, 无 TF.

        先反投影到相机光学系(z 朝前), 再左乘固定旋转搬到 Link_30 机械系.
        raw 模式下 (u,v) 在深度图坐标系, 必须用深度内参反投影; 否则用彩色内参.
        """
        K = self._depth_matrix if self.use_raw_depth else self._camera_matrix
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        p_opt = np.array([(u - cx) * d / fx, (v - cy) * d / fy, d])
        return self._R_mech_optical @ p_opt if self.apply_optical_rotation else p_opt

    def _lookup_base_tf(self):
        """查 TF camera_frame -> base_frame, 返回 (R, t). 查不到返回 (None, None)."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self._warn_throttle('查 TF %s->%s 失败: %s'
                                % (self.base_frame, self.camera_frame, e))
            return None, None
        tr = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        return R, np.array([tr.x, tr.y, tr.z])

    def _estimate_yaw(self, contour, cu, cv_, d, R, axis=None):
        """估盒子绕 base_link 竖直轴的 yaw (契约 §1 第 4 自由度).

        长轴图像方向来源: 优先用 OBB 旋转框角度 (axis, 彩色像素系); 无则退回顶面
        深度掩码的 minAreaRect. 沿该方向中心两侧各取一采样点, 用同一中心深度 d
        反投影到 base_link (顶面近似水平, 等深假设成立), 两点连线在 base_link
        xy 投影方位角即 yaw. 直接用图像角度会差一个相机->base 旋转, 故必经反投影.
        矩形长轴无向 (±180°等价), 折叠到 (-pi/2, pi/2]; 顶视吸盘对 180° 不敏感.
        """
        if R is None:
            return None
        if axis is not None:
            dx, dy = axis
            # axis 在彩色像素系; raw 模式下采样点在深度像素系, 按内参比例转方向
            if self.use_raw_depth:
                Kc, Kd = self._camera_matrix, self._depth_matrix
                dx *= Kd[0, 0] / Kc[0, 0]
                dy *= Kd[1, 1] / Kc[1, 1]
                n = np.hypot(dx, dy)
                if n < 1e-9:
                    return None
                dx, dy = dx / n, dy / n
        else:
            rect = cv2.minAreaRect(contour)      # ((cx,cy),(w,h),angle)
            (rw, rh), ang = rect[1], rect[2]
            if rw <= 0 or rh <= 0:
                return None
            theta = np.deg2rad(ang if rw >= rh else ang + 90.0)
            dx, dy = np.cos(theta), np.sin(theta)
        L = 30.0                                  # 采样臂长(像素), 等深下不影响方位角
        p_pos = self._pixel_to_cam(cu + L * dx, cv_ + L * dy, d)
        p_neg = self._pixel_to_cam(cu - L * dx, cv_ - L * dy, d)
        v = R @ (p_pos - p_neg)                   # base_link 下的长轴向量
        if abs(v[0]) < 1e-9 and abs(v[1]) < 1e-9:
            return None
        yaw = np.arctan2(v[1], v[0])
        if yaw > np.pi / 2:
            yaw -= np.pi
        elif yaw <= -np.pi / 2:
            yaw += np.pi
        return float(yaw)

    # ---------------- 输出 ----------------

    def _print_results(self, results):
        valid = [r for r in results if r[6] is not None]
        if not valid:
            self._info_throttle('未检测到可定位盒子.')
            return
        lines = ['检测到 %d 个盒子 (顶面中心):' % len(valid)]
        for i, (x1, y1, x2, y2, name, cf, pose, _corners) in enumerate(valid):
            cu, cv_, d, p, yaw, p_cam = pose
            # 中心点坐标+深度恒有效 (相机系 Link_30, 纯几何不依赖 TF)
            base = ('   base_link=[%.3f, %.3f, %.3f]m yaw=%s'
                    % (p[0], p[1], p[2],
                       ('%.1f°' % np.rad2deg(yaw)) if yaw is not None else 'n/a')
                    ) if p is not None else '   (整车 TF 未就绪, 无 base_link 坐标)'
            lines.append(
                '  #%d %s(%.2f) 中心像素(%d,%d) 深度=%.3fm 相机系=[%.3f, %.3f, %.3f]m%s'
                % (i + 1, name, cf, cu, cv_, d, p_cam[0], p_cam[1], p_cam[2], base))
        self.get_logger().info('\n'.join(lines))

    def _publish_pose(self, results, stamp):
        """发布盒子顶面中心位姿 (契约 §1). 取面积最大且有 base_link 坐标的框作单目标.

        frame_id=base_link, position=顶面中心, orientation roll=pitch=0,
        yaw=盒子绕竖直轴转角 -> 四元数 (0,0,sin(yaw/2),cos(yaw/2)). yaw 估不出退回 0.
        """
        if self._pose_pub is None:
            return
        cand = [r for r in results if r[6] is not None and r[6][3] is not None]
        if not cand:
            return

        def to_pose(r):
            _cu, _cv, _d, p, yaw, _p_cam = r[6]
            yaw = yaw if yaw is not None else 0.0
            ps = Pose()
            ps.position.x = float(p[0])
            ps.position.y = float(p[1])
            ps.position.z = float(p[2])
            ps.orientation.z = float(np.sin(yaw / 2.0))   # roll=pitch=0
            ps.orientation.w = float(np.cos(yaw / 2.0))
            return ps

        # 单目标 (契约 §1): 面积最大框, 供抓取选当前目标
        target = max(cand, key=lambda r: (r[2] - r[0]) * (r[3] - r[1]))
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.pose = to_pose(target)
        self._pose_pub.publish(msg)

        # 多目标数组: 所有有 base_link 坐标的盒子 (面积降序)
        if self._poses_pub is not None:
            arr = PoseArray()
            arr.header.stamp = stamp
            arr.header.frame_id = self.base_frame
            arr.poses = [to_pose(r) for r in sorted(
                cand, key=lambda r: (r[2] - r[0]) * (r[3] - r[1]), reverse=True)]
            self._poses_pub.publish(arr)

    def _draw_and_show(self, bgr, results):
        for i, (x1, y1, x2, y2, name, cf, pose, corners) in enumerate(results):
            # OBB 模型: 画旋转框 (四角点); 无角点(普通 detect)退回轴对齐框
            if corners is not None:
                cv2.polylines(bgr, [corners], True, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            if pose is None:
                cv2.putText(bgr, '%s %.2f (no depth)' % (name, cf), (x1, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2, cv2.LINE_AA)
                continue
            cu, cv_, d, p, yaw, p_cam = pose
            # cu,cv_ 在深度图坐标系(raw 模式); 画到彩色图需转回彩色像素
            pcu, pcv = self._depth_px_to_color_px(cu, cv_) if self.use_raw_depth else (cu, cv_)
            # 物体坐标系 (X=红沿长轴, Y=绿, Z=蓝竖直向上): 有 base 位姿+yaw 才画
            if p is not None and yaw is not None:
                self._draw_object_frame(bgr, p, yaw, (pcu, pcv))
            cv2.circle(bgr, (pcu, pcv), 5, (0, 0, 255), -1)
            # 标签: 恒显示深度; 有 base_link 坐标则显示 xyz+yaw, 否则显示相机系 xyz
            if p is not None:
                yaw_s = ('%.0f' % np.rad2deg(yaw)) if yaw is not None else 'na'
                label = '#%d %s d=%.2fm base[%.2f,%.2f,%.2f] y=%s' % (
                    i + 1, name, d, p[0], p[1], p[2], yaw_s)
            else:
                label = '#%d %s d=%.2fm cam[%.2f,%.2f,%.2f]' % (
                    i + 1, name, d, p_cam[0], p_cam[1], p_cam[2])
            cv2.putText(bgr, label, (x1, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.imshow('yolo_box_detector', bgr)
        cv2.waitKey(1)

    def _base_point_to_color_px(self, p_base):
        """base_link 3D 点 -> 彩色像素 (u,v). 需 TF(base<-cam) 已就绪, 否则 None.

        p_base -> 相机机械系(Link_30): p_cam = R^T (p_base - t)
        -> 相机光学系: 左乘 R_mech_optical^T
        -> 用相机内参投影到像素. raw 模式深度/彩色内参不同, 画在彩色图上用彩色内参.
        """
        R, t = getattr(self, '_R_bc', None), getattr(self, '_t_bc', None)
        if R is None or t is None:
            return None
        p_cam = R.T @ (np.asarray(p_base, float) - t)     # base -> Link_30 机械系
        p_opt = (self._R_mech_optical.T @ p_cam
                 if self.apply_optical_rotation else p_cam)  # 机械系 -> 光学系
        if p_opt[2] <= 1e-6:                               # 在相机后方, 不投影
            return None
        K = self._camera_matrix                            # 彩色内参(画在彩色图上)
        u = K[0, 0] * p_opt[0] / p_opt[2] + K[0, 2]
        v = K[1, 1] * p_opt[1] / p_opt[2] + K[1, 2]
        return int(round(u)), int(round(v))

    def _draw_object_frame(self, bgr, p, yaw, origin_px, axis_len=0.05):
        """在盒子顶面中心画物体坐标系三轴 (X=红沿长轴, Y=绿, Z=蓝竖直向上).

        轴在 base_link 系定义: X=(cos yaw, sin yaw, 0), Y=(-sin yaw, cos yaw, 0),
        Z=(0,0,1). 各取 axis_len(米) 端点, 投影回彩色像素, 从中心画到端点.
        """
        p = np.asarray(p, float)
        c, s = np.cos(yaw), np.sin(yaw)
        axes = (                                           # (方向向量, BGR 颜色)
            (np.array([c, s, 0.0]),  (0, 0, 255)),         # X 红
            (np.array([-s, c, 0.0]), (0, 255, 0)),         # Y 绿
            (np.array([0.0, 0.0, 1.0]), (255, 0, 0)),      # Z 蓝
        )
        for vec, color in axes:
            end = self._base_point_to_color_px(p + vec * axis_len)
            if end is not None:
                cv2.arrowedLine(bgr, origin_px, end, color, 2,
                                cv2.LINE_AA, tipLength=0.25)

    # ---------------- 节流日志 ----------------

    def _warn_throttle(self, text, period_ns=2_000_000_000):
        now = self.get_clock().now().nanoseconds
        if now - getattr(self, '_last_warn_ns', 0) >= period_ns:
            self._last_warn_ns = now
            self.get_logger().warn(text)

    def _info_throttle(self, text, period_ns=2_000_000_000):
        now = self.get_clock().now().nanoseconds
        if now - getattr(self, '_last_info_ns', 0) >= period_ns:
            self._last_info_ns = now
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = YoloBoxDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
