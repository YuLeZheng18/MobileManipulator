"""torchvision 缺失兜底 (Jetson 离线环境, 系统 GPU torch 无匹配 torchvision).

本机 JetPack6 系统 python 装的是 Jetson 原生 GPU torch 2.5 (cuda=True), 但无匹配的
GPU torchvision 轮子, 外网又不通装不了. ultralytics 只在 NMS 一处用 torchvision.ops.nms,
其余仅做版本存在性检查. 故在 import ultralytics 前 install_torchvision_stub() 注入一个
最小 torchvision: 纯 torch 实现的 ops.nms + 假 metadata 版本号, 让推理全程走 GPU.

用法 (务必在 import ultralytics/YOLO 之前调用):
    from mm_perception._tv_stub import install_torchvision_stub
    install_torchvision_stub()
    from ultralytics import YOLO

若系统已装好真正的 torchvision, 本函数直接跳过, 不覆盖.
"""
import sys
import types
import importlib.util


def _pure_torch_nms(boxes, scores, iou_thres):
    """纯 torch 的 NMS, 逻辑等价 torchvision.ops.nms. boxes: (N,4) xyxy."""
    import torch
    if boxes.numel() == 0:
        return torch.empty((0,), dtype=torch.int64, device=boxes.device)
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1).clamp(min=0) * (y2 - y1).clamp(min=0)
    order = scores.argsort(descending=True)
    keep = []
    while order.numel() > 0:
        i = order[0]
        keep.append(i.item())
        if order.numel() == 1:
            break
        rest = order[1:]
        xx1 = torch.maximum(x1[i], x1[rest])
        yy1 = torch.maximum(y1[i], y1[rest])
        xx2 = torch.minimum(x2[i], x2[rest])
        yy2 = torch.minimum(y2[i], y2[rest])
        w = (xx2 - xx1).clamp(min=0)
        h = (yy2 - yy1).clamp(min=0)
        inter = w * h
        iou = inter / (areas[i] + areas[rest] - inter + 1e-9)
        order = rest[iou <= iou_thres]
    return torch.tensor(keep, dtype=torch.int64, device=boxes.device)


def install_torchvision_stub(fake_version='0.20.0'):
    """torchvision 不可用时注入最小兜底. 返回 True=注入了 stub, False=已有真包."""
    # 真 torchvision 可用则不动 (import 成功且带 ops.nms)
    if importlib.util.find_spec('torchvision') is not None:
        try:
            import torchvision  # noqa: F401
            from torchvision.ops import nms  # noqa: F401
            return False
        except Exception:  # noqa: BLE001
            pass  # 装了但坏 (ABI 不匹配等) -> 用 stub 覆盖

    tv = types.ModuleType('torchvision')
    tv.__version__ = fake_version
    ops = types.ModuleType('torchvision.ops')
    ops.nms = _pure_torch_nms
    tv.ops = ops
    sys.modules['torchvision'] = tv
    sys.modules['torchvision.ops'] = ops

    # ultralytics 用 importlib.metadata 查版本, 打补丁让它查得到
    import importlib.metadata as md
    _orig = md.version
    def _patched(name):
        if name == 'torchvision':
            return fake_version
        return _orig(name)
    md.version = _patched
    return True
