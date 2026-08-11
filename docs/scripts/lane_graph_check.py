#!/usr/bin/env python3
"""车道图离线校验 (不需要机器人, 不需要 ROS 运行时).

判据 = **圆形 robot_radius**: 用精确欧氏距离变换(EDT)算出每个自由格到最近障碍的净距,
再要求"每个节点 + 整条圆角路径逐点"的净距 >= robot_radius。圆各向同性, 故"站得住"
与"能原地自转"是同一个判据, 不必再扫朝向。

⚠️ 两条曾经踩过的口径错误, 别重犯:
  1. **别用外包络长方形当 footprint**。旧值
     [[0.19,0.195],[0.19,-0.195],[-0.30,-0.195],[-0.30,0.195]] 的后两角实际是空的 ——
     STL 实测托盘 Link_11 只到 y=±0.11。那两个幽灵角把外接圆撑到 0.3578(真值 0.3097),
     偏保守 4.8cm, 曾误判出"南北通道不能自转""place1 揉头只剩 2.5cm"等**已作废**的结论。
  2. **别用 4 连通 BFS 当净距场**。那算出来是曼哈顿距离, 系统性高估最多 28%,
     用它筛出来的坐标全部偏乐观。本脚本用 Felzenszwalb 两趟一维法算精确 EDT。

用法:
    python3 src/docs/scripts/lane_graph_check.py                # 全量校验
    python3 src/docs/scripts/lane_graph_check.py --radius 0.28  # 换口径试算
    python3 src/docs/scripts/lane_graph_check.py --edge pick1 pick3   # 只看一条候选边
"""
import argparse
import heapq
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))   # -> 工作区 src/
MAP_YAML = os.path.join(ROOT, 'mm_navigation/maps/room_real.yaml')
MAP_PGM = os.path.join(ROOT, 'mm_navigation/maps/room_real.pgm')
GRAPH = os.path.join(ROOT, 'mm_navigation/config/lane_graph.yaml')
NAV2 = os.path.join(ROOT, 'mm_navigation/config/nav2_params.yaml')

# 真实几何 (STL 实测顶点, 非估算): base_link 0.3614长 x 0.3745宽;
# 托盘 Link_11 经 Joint_5 变换后 x∈[-0.2893,-0.1313] y∈[-0.1099,+0.1106]
TRUE_CIRCUMRADIUS = math.hypot(0.2893, 0.1106)   # = 0.3097, 含托盘的真外接圆
CORNER_RADIUS = 0.3      # 与 real_bringup.launch.py 传给 lane_navigator 的值一致
POINT_SPACING = 0.05     # 与 lane_graph.yaml 的 point_spacing 一致


def load_pgm(path):
    """读 P5 灰度图. 注释行以 # 开头, 可出现在任意 header 字段之间."""
    with open(path, 'rb') as f:
        data = f.read()
    idx, fields = 0, []
    while len(fields) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] not in (b'\n', b''):
                idx += 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        fields.append(data[start:idx])
    magic, w, h = fields[0], int(fields[1]), int(fields[2])
    assert magic == b'P5', magic
    idx += 1
    return w, h, data[idx:idx + w * h]


def _edt_1d(f):
    """Felzenszwalb 一维平方距离变换 (下包络法), O(n)."""
    n = len(f)
    d = np.empty(n)
    v = np.zeros(n, dtype=int)
    z = np.empty(n + 1)
    k = 0
    z[0], z[1] = -np.inf, np.inf
    for q in range(1, n):
        while True:
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
            if s <= z[k]:
                k -= 1
            else:
                break
        k += 1
        v[k], z[k], z[k + 1] = q, s, np.inf
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + f[v[k]]
    return d


def exact_edt(obstacle_mask):
    """精确欧氏距离变换: 每格到最近 True(障碍)格的距离, 单位=格."""
    f = np.where(obstacle_mask, 0.0, 1e12)
    tmp = np.empty_like(f)
    for c in range(f.shape[1]):
        tmp[:, c] = _edt_1d(f[:, c])
    out = np.empty_like(tmp)
    for r in range(tmp.shape[0]):
        out[r, :] = _edt_1d(tmp[r, :])
    return np.sqrt(out)


class Clearance:
    """净距场: 世界坐标 -> 到最近障碍的距离(米)."""

    def __init__(self):
        meta = yaml.safe_load(open(MAP_YAML))
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.w, self.h, px = load_pgm(MAP_PGM)
        arr = np.frombuffer(px, dtype=np.uint8).reshape(self.h, self.w)
        # room_real.pgm 是 trinary 图, 实测只有三个灰度值:
        #   0 = 占据(实体墙) / 205 = unknown(未观测) / 254 = 空闲
        # nav2 侧 allow_unknown:true 且 free_thresh 0.25 -> unknown 判可通行, 这里对齐它。
        # (若要保守口径把 unknown 也当障碍, 改成 arr != 254。)
        self.obstacle = (arr == 0)
        self.dist = exact_edt(self.obstacle) * self.res
        self.free_cells = int((arr != 0).sum())

    def at(self, x, y):
        cx = int((x - self.ox) / self.res)
        cy = int((y - self.oy) / self.res)
        if not (0 <= cx < self.w and 0 <= cy < self.h):
            return 0.0
        return float(self.dist[self.h - 1 - cy, cx])   # pgm 首行 = 最大 y

    def along(self, p, q):
        """线段上的最小净距."""
        ln = math.hypot(q[0] - p[0], q[1] - p[1])
        n = max(2, int(ln / (self.res * 0.4)))
        return min(self.at(p[0] + (q[0] - p[0]) * k / n,
                           p[1] + (q[1] - p[1]) * k / n) for k in range(n + 1))

    def standable_fraction(self, r):
        return float((self.dist >= r).sum()) / self.free_cells


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def build_rounded_xy(pts, r):
    """与 lane_navigator.build_rounded_xy 同算法: 顶点序列倒圆角后加密成点列.

    校验必须复用同一算法, 否则算的不是车实际会走的那条线。
    """
    def sample_line(xy, p0, p1):
        seg = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        n = max(1, int(seg / POINT_SPACING))
        for k in range(1, n + 1):
            t = k / n
            xy.append((p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t))

    n = len(pts)
    fillets = {}
    for i in range(1, n - 1):
        A, V, B = pts[i - 1], pts[i], pts[i + 1]
        v1 = (A[0] - V[0], A[1] - V[1])
        v2 = (B[0] - V[0], B[1] - V[1])
        l1, l2 = math.hypot(*v1), math.hypot(*v2)
        if l1 < 1e-6 or l2 < 1e-6:
            continue
        u1 = (v1[0] / l1, v1[1] / l1)
        u2 = (v2[0] / l2, v2[1] / l2)
        dot = max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))
        alpha = math.acos(dot)
        if alpha > math.pi - 0.05 or alpha < 0.05:
            continue
        half = alpha / 2.0
        T = min(r / math.tan(half), 0.45 * l1, 0.45 * l2)
        r_eff = T * math.tan(half)
        p1 = (V[0] + u1[0] * T, V[1] + u1[1] * T)
        p2 = (V[0] + u2[0] * T, V[1] + u2[1] * T)
        bx, by = u1[0] + u2[0], u1[1] + u2[1]
        bl = math.hypot(bx, by)
        if bl < 1e-6:
            continue
        cdist = r_eff / math.sin(half)
        cx, cy = V[0] + bx / bl * cdist, V[1] + by / bl * cdist
        a1 = math.atan2(p1[1] - cy, p1[0] - cx)
        a2 = math.atan2(p2[1] - cy, p2[0] - cx)
        fillets[i] = (p1, p2, cx, cy, a1, norm_angle(a2 - a1), r_eff)

    xy = [pts[0]]
    cur = pts[0]
    for i in range(1, n):
        if i in fillets:
            p1, p2, cx, cy, a1, dtheta, r_eff = fillets[i]
            sample_line(xy, cur, p1)
            na = max(1, int(abs(dtheta) * r_eff / POINT_SPACING))
            for k in range(1, na + 1):
                ang = a1 + dtheta * (k / na)
                xy.append((cx + r_eff * math.cos(ang), cy + r_eff * math.sin(ang)))
            cur = p2
        else:
            sample_line(xy, cur, pts[i])
            cur = pts[i]

    min_gap = max(1e-4, POINT_SPACING / 5.0)
    dedup = [xy[0]]
    for p in xy[1:]:
        if math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > min_gap:
            dedup.append(p)
    return dedup


def load_graph():
    data = yaml.safe_load(open(GRAPH))
    nodes = {k: (float(v['x']), float(v['y'])) for k, v in data['nodes'].items()}
    edges = [tuple(e) for e in data['edges']]
    return nodes, edges, data


def dijkstra(nodes, edges, s, t):
    adj = {k: [] for k in nodes}
    for a, b in edges:
        w = math.hypot(nodes[a][0] - nodes[b][0], nodes[a][1] - nodes[b][1])
        adj[a].append((b, w))
        adj[b].append((a, w))
    dist, prev = {s: 0.0}, {}
    pq = [(0.0, s)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == t:
            path = [t]
            while path[-1] != s:
                path.append(prev[path[-1]])
            return d, list(reversed(path))
        if d > dist.get(u, 1e9):
            continue
        for v, w in adj[u]:
            if d + w < dist.get(v, 1e9):
                dist[v], prev[v] = d + w, u
                heapq.heappush(pq, (d + w, v))
    return float('inf'), []


def read_radius():
    """从 nav2_params.yaml 读 robot_radius, 保证校验口径与实跑一致."""
    d = yaml.safe_load(open(NAV2))
    lc = d['local_costmap']['local_costmap']['ros__parameters']
    gc = d['global_costmap']['global_costmap']['ros__parameters']
    r_l, r_g = lc.get('robot_radius'), gc.get('robot_radius')
    if r_l is None or r_g is None:
        raise SystemExit('nav2_params.yaml 里没有 robot_radius (还在用 footprint?)')
    if r_l != r_g:
        raise SystemExit(f'local/global robot_radius 不一致: {r_l} vs {r_g}')
    infl = lc['inflation_layer']['inflation_radius']
    if infl < r_l:
        print(f'⚠️ inflation_radius {infl} < robot_radius {r_l} —— 代价地图会允许路径贴到'
              f'比车还近的地方, 必然蹭障')
    return float(r_l)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--radius', type=float, default=None,
                    help='试算用的 robot_radius; 缺省则从 nav2_params.yaml 读')
    ap.add_argument('--edge', nargs=2, metavar=('A', 'B'),
                    help='只校验一条候选边(可以是图里还没有的)')
    args = ap.parse_args()

    cl = Clearance()
    R = args.radius if args.radius is not None else read_radius()
    nodes, edges, raw = load_graph()

    print(f'地图 {cl.w}x{cl.h} @ {cl.res}m   自由格 {cl.free_cells}')
    print(f'robot_radius = {R}   真外接圆(含托盘) = {TRUE_CIRCUMRADIUS:.4f}'
          f'   corner_radius = {CORNER_RADIUS}')
    if R < TRUE_CIRCUMRADIUS:
        print(f'  注: R 比真外接圆小 {(TRUE_CIRCUMRADIUS - R) * 100:.1f}cm '
              f'-> 托盘那部分不受 costmap 保护, 仅在**离开路网节点原地自转**时才可能吃到')
    print(f'可站面积占比 {cl.standable_fraction(R) * 100:.1f}%')

    if args.edge:
        a, b = args.edge
        pa = nodes.get(a) or (_ for _ in ()).throw(SystemExit(f'未知节点 {a}'))
        pb = nodes.get(b) or (_ for _ in ()).throw(SystemExit(f'未知节点 {b}'))
        m = cl.along(pa, pb)
        ln = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
        print(f'\n{a}-{b}: 长 {ln:.2f}m  最小净距 {m:.3f}  '
              f'余量 {(m - R) * 100:+.1f}cm  {"OK" if m >= R else "✗ 过不去"}')
        return 0 if m >= R else 1

    fails = []

    print('\n=== 节点 (圆形口径: 净距 >= R 即可站可自转) ===')
    print(f'{"node":<9}{"净距":>7}{"余量":>8}{"真外接圆余量":>13}   判定')
    for k in sorted(nodes, key=lambda n: -cl.at(*nodes[n])):
        c = cl.at(*nodes[k])
        ok = c >= R
        if not ok:
            fails.append(f'node {k}')
        print(f'{k:<9}{c:>7.3f}{(c - R) * 100:>7.1f}cm{(c - TRUE_CIRCUMRADIUS) * 100:>11.1f}cm'
              f'   {"OK" if ok else "✗ 站不住"}'
              f'{"" if c >= TRUE_CIRCUMRADIUS else "  (真几何也转不开!)"}')

    print(f'\n=== {len(edges)} 条边 ===')
    print(f'{"edge":<18}{"len":>7}{"净距":>8}{"余量":>8}   判定')
    worst, worst_name = 9.0, ''
    for a, b in edges:
        m = cl.along(nodes[a], nodes[b])
        ln = math.hypot(nodes[a][0] - nodes[b][0], nodes[a][1] - nodes[b][1])
        if m < worst:
            worst, worst_name = m, f'{a}-{b}'
        ok = m >= R
        if not ok:
            fails.append(f'edge {a}-{b}')
        print(f'{a + "-" + b:<18}{ln:>6.2f}m{m:>8.3f}{(m - R) * 100:>7.1f}cm'
              f'   {"OK" if ok else "✗ 过不去"}')
    print(f'最紧边: {worst_name} 净距 {worst:.3f} -> 中线可用带宽 {(worst - R) * 200:.1f}cm')
    if (worst - R) * 2 < 0.06:
        print('  ⚠️ 带宽小于 AMCL 实测落点误差 5~6cm, 规划易失败')

    # 连通性: 任务点必须都能从 home 到达
    adj = {k: set() for k in nodes}
    for a, b in edges:
        adj[a].add(b)
        adj[b].add(a)
    seen, stack = {'home'}, ['home']
    while stack:
        for v in adj[stack.pop()]:
            if v not in seen:
                seen.add(v)
                stack.append(v)
    if len(seen) != len(nodes):
        iso = set(nodes) - seen
        fails.append(f'孤立节点 {iso}')
        print(f'\n✗ 从 home 不可达: {iso}')
    else:
        print(f'\n连通性 OK: home 可达全部 {len(nodes)} 个节点')

    # 整条路线: 圆角会切拐角, 与逐边校验不同, 必须单独验
    print('\n=== 整条路线 (倒圆角后逐点, 与实跑一致) ===')
    tasks = ['home', 'pick1', 'pick2', 'pick3', 'pick4', 'place1', 'place2']
    tasks = [t for t in tasks if t in nodes]
    print(f'{"route":<18}{"len":>7}{"净距":>8}{"余量":>8}   判定')
    for a in tasks:
        for b in tasks:
            if a >= b:
                continue
            _, path = dijkstra(nodes, edges, a, b)
            if len(path) < 2:
                continue
            xy = build_rounded_xy([nodes[p] for p in path], CORNER_RADIUS)
            m = min(cl.at(*p) for p in xy)
            ln = sum(math.hypot(xy[i + 1][0] - xy[i][0], xy[i + 1][1] - xy[i][1])
                     for i in range(len(xy) - 1))
            ok = m >= R
            if not ok:
                fails.append(f'route {a}->{b}')
            if not ok or m - R < 0.05:
                print(f'{a + "->" + b:<18}{ln:>6.2f}m{m:>8.3f}{(m - R) * 100:>7.1f}cm'
                      f'   {"OK(最紧)" if ok else "✗ 撞"}')
    print('  (只列最紧的; 余量 >5cm 的路线省略)')

    # 设计不变量: 通道段内不得有路网节点
    print('\n=== 设计不变量 ===')
    tight = [k for k in nodes if cl.at(*nodes[k]) < TRUE_CIRCUMRADIUS]
    if tight:
        print(f'✗ 这些节点净距 < 真外接圆 {TRUE_CIRCUMRADIUS:.4f}, 车在那儿物理上转不开: {tight}')
        print('   -> 要么挪点, 要么给它设 hold_yaw 让终点 cspin 跳过自转')
        fails.append(f'节点转不开 {tight}')
    else:
        print(f'OK: {len(nodes)} 个节点净距全部 >= 真外接圆 {TRUE_CIRCUMRADIUS:.4f}, 都能原地自转')
        print(f'    (瓶颈通道净距 {worst:.3f} < {TRUE_CIRCUMRADIUS:.4f} 转不开, '
              f'但通道内没有节点 -> 只直行穿过)')

    print()
    if fails:
        print(f'✗ {len(fails)} 项未通过: {fails}')
        return 1
    print('✓ 全部通过')
    return 0


if __name__ == '__main__':
    sys.exit(main())
