// mm_grasp / grasp_node
// S4 三段抓取执行器 (被 mm_task 状态机调用, 本身不是状态机):
//   ① 粗定位 (闭环, MoveIt 规划): setPoseTarget suction_tip 到盒上方预抓取位
//   ② 精修   (闭环, 笛卡尔步进): 相机系盒心追标定目标, 实测雅可比解位移, 逐步走到位
//   ③ 末段   (开环, 相对直插): 沿当前 suction_tip -Z 相对下插固定行程 + 气泵吸
//   放置: 相对抬起 -> 规划到 tray(Link_11) 上方 -> 释放
// 纪律(§5): 末段严禁用 FK 重算盒子 base_link 绝对坐标再 setPoseTarget.
//           末段 waypoint 只由"当前实测 suction_tip 位姿 + 固定 -Z 偏置"得来.
//
// 所有 MoveGroup 位姿运算统一在 base_link 系 (setPoseReferenceFrame), 与 object_pose
// 一致, 从而与 move_group 内部 planning_frame 归属解耦.
//
// M4: std_srvs/Trigger 服务 /grasp/execute 触发一整轮.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mutex>
#include <vector>

using namespace std::chrono_literals;

namespace mm_grasp
{

constexpr int8_t PUMP_STOP = 0;
constexpr int8_t PUMP_SUCK = 1;
constexpr int8_t PUMP_RELEASE = 2;

double quatYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// 折到 (-90°,90°]: 矩形长轴无向, 差 180° 是同一朝向, 比较长轴夹角必须先折.
double wrapAngle90(double a)
{
  a = wrapAngle(a);
  if (a > M_PI / 2.0) a -= M_PI;
  else if (a <= -M_PI / 2.0) a += M_PI;
  return a;
}

double clampAbs(double v, double lim)
{
  return std::max(-lim, std::min(lim, v));
}

class GraspNode
{
public:
  explicit GraspNode(const rclcpp::Node::SharedPtr & node)
  : node_(node), logger_(node->get_logger())
  {
    planning_group_ = node_->declare_parameter<std::string>("planning_group", "arm");
    ee_link_ = node_->declare_parameter<std::string>("ee_link", "suction_tip");
    base_frame_ = node_->declare_parameter<std::string>("base_frame", "base_link");
    tray_frame_ = node_->declare_parameter<std::string>("tray_frame", "Link_11");
    // 吸盘**本体**链接 (带网格的那个). ee_link_=suction_tip 是它下面的空坐标系, 没有几何,
    // 所以 attachBox 的 touch_links 必须把这个也放进去, 否则盒子与吸盘本体照样判碰撞.
    suction_body_link_ = node_->declare_parameter<std::string>("suction_body_link", "suction_link");
    object_topic_ = node_->declare_parameter<std::string>("object_topic", "/perception/object_pose");
    pump_topic_ = node_->declare_parameter<std::string>("pump_topic", "/pump_cmd");

    pregrasp_height_ = node_->declare_parameter<double>("pregrasp_height", 0.12);
    // 末段行程夹取区间: 实测行程(TCP z - 锁定盒顶 z)超出此区间说明锁定值或 TF 有问题, 夹住.
    // ②不再降高, 所以③要从 pregrasp 高度(12cm)一路插到盒顶, 上限要盖住整段行程.
    insert_stroke_min_ = node_->declare_parameter<double>("insert_stroke_min", 0.02);
    insert_stroke_max_ = node_->declare_parameter<double>("insert_stroke_max", 0.16);
    // base_link 原点离地高度 (URDF mm_robot_gazebo: 约 0.0476m) -> 地面在 base_link 系 z.
    // 盒子平放在地面上, 盒顶高度 = ground_z_ + 该类别标定厚度, 纯几何算, 视觉不参与:
    // 视觉测高要过深度+相机外参, 实测偏差 28mm 且厚度估计本身在 3~21mm 间跳; 而地面高度
    // 是 URDF 常量、盒厚是卡尺实测值, 两个都比测量可信.
    ground_z_ = node_->declare_parameter<double>("ground_z", -0.0476);
    // ③ 行程少走这么多米 (吸盘停在盒顶上方这个高度, 不压到盒顶). 单独一个参数而不是改
    // thickness: thickness 同时用来算托盘 release z (tray_z + 累计 + 本层), 动它会把堆叠
    // 高度一起算错.
    // 2026-07-28 实跑后从 +0.002 改成 -0.001 (即多插 3mm): 按 +2mm 算出的行程下到位时
    // 吸盘离盒顶仍有可见空隙. 说明 ground_z 与厚度这组标定值合起来把盒顶估高了几 mm ——
    // 两者当初是靠"吸住瞬间 TCP z"同一组数反推的, 彼此自洽但都不独立. 这里用负值压过去,
    // 靠吸盘海绵/波纹吃掉多压的量.
    // 2026-07-30 值移到 place.yaml (类别 2 盒实跑没吸住, 要再多插 5mm). 故这里必须
    // getOrDeclare: yaml 给了值时 automatically_declare_parameters_from_overrides 已声明,
    // 裸 declare_parameter 会撞 AlreadyDeclared 崩 (release_duration 踩过, exit -6).
    insert_shortfall_ = getOrDeclare<double>("insert_shortfall", -0.001);
    // 吸取抽真空时长(秒), 到时转 PUMP_STOP 保压.
    // 2026-08-03 用户实机要求回到 3.0: 此前的 1.0 是按"3s 里负压早已建立"推的, 但那个判断
    // 没有独立证据 (吸不上的几轮最后都是靠调 insert_shortfall 多插几 mm 解决的, 与抽气时长
    // 无关), 而用户现场看到 1s 偏紧. 代价只是每次抓取多等 2s.
    // 值改由 place.yaml 给, 故必须 getOrDeclare —— 裸 declare_parameter 会与
    // automatically_declare_parameters_from_overrides 的自动声明撞 AlreadyDeclared,
    // 整个节点 exit -6 崩掉 (release_duration 2026-07-28 踩过).
    suck_duration_ = getOrDeclare<double>("suck_duration", 3.0);
    // 释放开阀时长(秒), 到时立刻 PUMP_STOP 关阀. 见 releasePulse(): 阀持续通电会发烫.
    // 2026-07-28 真机降到 1.0(place.yaml 给值): 破负压几百 ms 够, 3s 拖节拍. 值写在 yaml,
    // 故这里必须 getOrDeclare(裸 declare_parameter 会与 yaml 自动声明撞 AlreadyDeclared 崩).
    release_duration_ = getOrDeclare<double>("release_duration", 3.0);
    // 末段下插降速倍率: 实测下降段抖动. 降速让每个轨迹点的关节增量变小, 抖动幅度随之变小.
    // 这是压制不是根治(根因待查: 疑在笛卡尔路径的时间参数化/该构型雅可比条件数).
    // 2026-07-30: 0.3 -> 0.4 (用户要求各段一律全速 0.4, 去掉这段单独的慢档).
    // ⚠️ 抖动若复现就退回 0.3: 这一段正是吸取瞬间, 抖动直接决定吸不吸得上.
    insert_velocity_scaling_ = node_->declare_parameter<double>("insert_velocity_scaling", 0.4);
    // 全局规划降速倍率 (原来写死 0.2). 抬到 0.4: 前面几轮 0.2 下各段都稳, 抓取/放置路径
    // 也已实跑验证过, 没必要一直按最初的试探速度爬.
    plan_velocity_scaling_ = node_->declare_parameter<double>("plan_velocity_scaling", 0.4);
    // 静置轮询周期(秒). 要连续两次采样都判"没动"才算停稳, 所以每段最少停 2 个周期 ——
    // 这是精修步进之间那个停顿的主要来源. 40ms 把下限从 200ms 压到 80ms.
    settle_poll_sec_ = node_->declare_parameter<double>("settle_poll_sec", 0.04);
    // 每段执行后静置多久再规划下一段 (秒). 见 settle() 注释: 控制器报完成时关节尚未停到位.
    settle_sec_ = node_->declare_parameter<double>("settle_sec", 3.0);
    // 判"停稳"的关节变化阈值(弧度): 比 MoveIt 执行前起点校验的 0.01 小一档, 停稳后残余
    // 漂移不至于再把下一段顶出容差.
    settle_eps_ = node_->declare_parameter<double>("settle_eps", 0.002);
    // 转 yaw 段独立降速倍率: 该段是绕吸盘自身轴原地转, 实测 TCP xy 漂移 0.0mm, 不搬运,
    // 不必按放置/抓取的搬运速度走. 默认 0.6, 比全局默认 0.2 快三倍(实测 180° 从 8.4s 降下来).
    rotate_velocity_scaling_ = node_->declare_parameter<double>("rotate_velocity_scaling", 0.6);
    // 吸取后先抬多高再搬. 保留这一段(而非吸完直接规划到托盘)是因为它的起点决定了后续
    // 路径形状: 不抬则起点是"吸盘贴地", 规划器可能选一条把盒沿地面拖过去、或为避让绕大圈
    // 甩臂的路. 抬 3cm 已脱离接触面, 耗时只有原 10cm 的三成.
    lift_height_ = node_->declare_parameter<double>("lift_height", 0.03);
    // 放托盘"正上方"抬高量: 到此高度盒远离托盘边框, 规划器不绕圈, 再垂直直下入位.
    tray_clearance_ = node_->declare_parameter<double>("tray_clearance", 0.06);
    // 携盒高空 transit 航点 z (绝对, base_link 系): 搬运段先 plan 到此高度再垂直直下.
    // 两端都在自由空间, 规划器不用钻障碍缝, 失败率≈0 (2026-07-29 替代原"直接 plan 到
    // tray_clearance_ 低空", 那版在左盘已有盒时规划会被 placed_t1 盒挡到 abort).
    // 选 0.30: 远高于托盘已放盒顶 (~0.10), 且在臂可达包络内. 直下段变长但纯垂直, 不慢.
    // getOrDeclare: yaml 写了 transit_z 会触发 auto-declare, 裸 declare_parameter 撞崩.
    transit_z_ = getOrDeclare<double>("transit_z", 0.30);
    // (原 transit_cart_z 已删: 那条"抬到低空 0.14 再纯笛卡尔横切入位"的抄近道 8/8 轮覆盖
    //  只有 9~13%, 即离开起点就断 —— 保持工具朝向横切那条直线上大半没有 IK 解, 是可达性
    //  问题, 不是被碰撞体挡的. 且盒 attach 时 touch_links 含托盘, 低空蹭围栏侧壁规划器不报
    //  执行不停, 只能靠人看. 收益为零而有无声风险, 整条删掉, 放置统一走高空 transit_z_.)

    // ---- 手动 jog (标定用): /grasp/jog 每次相对当前 TCP 走 (jog_dx,dy,dz) 一步 ----
    // 标定 cam_target 的操作手段: coarse_only 开到 12cm 后, 设这三个参数再 call /grasp/jog
    // 下插/微调 xy 对准盒心; 对准后设 jog_dz=+累计下插量 纯竖直抬回 12cm 再 calib_cam_target.
    // Trigger 带不了数值, 故走参数. 每次 call 读当前值走一步, 不清零(连续同向可反复 call).
    jog_dx_ = node_->declare_parameter<double>("jog_dx", 0.0);
    jog_dy_ = node_->declare_parameter<double>("jog_dy", 0.0);
    jog_dz_ = node_->declare_parameter<double>("jog_dz", 0.0);
    // 转腕增量(度, 绕 base 竖直轴, 即吸盘自身轴). 标定 yaw_ref 时手动转吸盘长轴到与盒边
    // 贴合. 单独走"直转"(不走 yawEquivalents 90°/180° 折叠), 保证转的就是你要的角度.
    jog_dyaw_deg_ = node_->declare_parameter<double>("jog_dyaw_deg", 0.0);

    // /grasp/seed_placed 用: 补登重启前已物理存在的盒. Trigger 带不了数值, 走参数.
    node_->declare_parameter<int64_t>("seed_tray", 0);
    node_->declare_parameter<double>("seed_thickness", 0.022);
    // /grasp/unload_tray 卸哪个托盘 (Trigger 带不了数值, 同上走参数).
    node_->declare_parameter<int64_t>("unload_tray", 0);

    // ---- 双托盘放置 + 按类别堆叠 (标定值在 place.yaml, base_link 系) ----
    // 托盘"空载吸盘接触托盘中心"位姿: xyz=空载 suction_tip 贴托盘中心时的位置, quat=工具朝下
    // 标定朝向(非单位姿态). release z = tray_z + 累计下层厚度 + 本层厚度 (吸盘吸盒顶, 盒底落
    // 在托盘/下层盒顶). 索引 0=右托盘, 1=左托盘. 缺省两组占位, 真机以 place.yaml 覆盖.
    num_trays_ = static_cast<int>(getOrDeclare<int64_t>("num_trays", 2));
    tray_x_  = getOrDeclare<std::vector<double>>("tray_x",  {-0.235, -0.235});
    tray_y_  = getOrDeclare<std::vector<double>>("tray_y",  {0.047, -0.062});
    tray_z_  = getOrDeclare<std::vector<double>>("tray_z",  {0.079, 0.081});
    tray_qx_ = getOrDeclare<std::vector<double>>("tray_qx", {0.009, 0.009});
    tray_qy_ = getOrDeclare<std::vector<double>>("tray_qy", {0.0, 0.0});
    tray_qz_ = getOrDeclare<std::vector<double>>("tray_qz", {1.0, 1.0});
    tray_qw_ = getOrDeclare<std::vector<double>>("tray_qw", {0.0, 0.0});
    tray_capacity_ = getOrDeclare<std::vector<int64_t>>("tray_capacity", {2, 2});
    // 每层额外抬高量(米), 逐层累加: 第 L 层释放高度多加 layer_gap * L.
    // 与"把 tray_z 抬 5mm"的区别: 那样只是整摞一起上移, 层间距不变; 这个让每层各自多留
    // 5mm, 第 1 层 +5mm、第 2 层 +10mm. 2026-08-04 用户实机要求(见 place.yaml 同名注释).
    // 两盘共用一个值 (用户确认"同一个"), 不做成数组.
    layer_gap_ = getOrDeclare<double>("layer_gap", 0.005);

    // 类别 -> 托盘映射: category_ids[i] 的盒子放到 category_tray[i] 号托盘(0=右,1=左).
    category_ids_  = getOrDeclare<std::vector<int64_t>>("category_ids",  {1, 2, 3, 4});
    category_tray_ = getOrDeclare<std::vector<int64_t>>("category_tray", {0, 0, 1, 1});
    // 厚度识别兜底(米, 按 category_ids 顺序): B 路线话题无有效厚度时回退到此.
    fallback_thickness_ = getOrDeclare<std::vector<double>>(
      "fallback_thickness", {0.025, 0.025, 0.025, 0.025});

    // ---- 四步渐进安全测试 (Task #1) ----
    // dry_run: 放置只 plan+打印不 execute; place_velocity_scaling: 放置段整体降速.
    dry_run_ = getOrDeclare<bool>("dry_run", false);
    place_velocity_scaling_ = getOrDeclare<double>("place_velocity_scaling", 1.0);

    // ---- 类别+厚度话题 (B 路线, Task #4) ----
    // 相机系闭环: 盒心在相机系(Link_30)的 3D 坐标. 精修不再用 base_link 坐标比对 ——
    // 手眼标定误差(安装误差 + 增量式零位与 URDF 零位不一致)让 base_link 坐标偏 2~3cm 且
    // 随构型变, 闭环追它会变成"臂动->标定偏差变->目标跑"的追逐(实测 20s 不收敛, 末端在
    // 原地抽). 相机系坐标只依赖深度与相机内参, 不过外参, 臂动它不会跳.
    cam_point_topic_ = getOrDeclare<std::string>(
      "cam_point_topic", "/perception/object_point_cam");
    // 相机机械系 link (视觉侧 camera_frame, p_cam 的 frame_id). 精修只用它的旋转.
    cam_frame_ = getOrDeclare<std::string>("cam_frame", "Link_30");
    // 横向分量是 (y,z) 不是 (x,y): Link_30 机械系的 x 是光轴 —— 实测 p_cam.x 与深度完全
    // 相等 (0.204 vs d=0.204), 即 R_mech_optical 把光学 z 映到机械 x. 拿 x 当横向会让
    // 闭环去追"距离".
    // 对准目标: 吸盘正对盒心时, 盒心应出现在相机系的哪个 (y,z). 实机标定值, 见 place.yaml.
    cam_target_y_ = getOrDeclare<double>("cam_target_y", 0.0);
    cam_target_z_ = getOrDeclare<double>("cam_target_z", 0.0);
    cam_tol_ = getOrDeclare<double>("cam_tol", 0.006);
    // ①的固定吸盘 yaw (base_link 系, 弧度). 取 look 位的腕朝向(实测 -90.0°), 使 look->①->②
    // 全程相机朝向不变 —— cam_target 就是在这个姿态附近标的, 换姿态它就不成立.
    coarse_yaw_ = getOrDeclare<double>("coarse_yaw", -M_PI / 2.0);
    // 发散保护倍率: 误差涨到起始值的这么多倍就中止精修 (见 stageRefine 里的判据).
    refine_diverge_ratio_ = getOrDeclare<double>("refine_diverge_ratio", 2.0);

    // ---- 吸盘朝向对齐: 闭环追长轴图像角 θ_img, 不过手眼外参也不解算腕角 ----
    // 视觉发的 base_link 系盒 yaw = R @ 相机系长轴, R(base_link<-Link_30) 含手眼外参旋转
    // 与 FK, 随构型漂: 2026-07-27 实测检测报盒 +36.2°(腕在 coarse_yaw -90° 时), 按它转到
    // 腕 -143.7° 后人工再修 6° 到 -149.7° 才与盒边贴合. 近正方形盒斜 6° 进带围栏托盘会蹭卡.
    // 相机固连腕(Link_30 挂 Link_29), 所以"吸盘长轴在图像里占多少度"是与构型无关的常数
    // θ_img*: 盒长轴与吸盘长轴平行 <=> θ_img == θ_img*. 于是对齐是个终点判据, 不用解算.
    axis_angle_topic_ = getOrDeclare<std::string>(
      "axis_angle_topic", "/perception/object_axis_angle");
    // 贴合判据(度): 吸盘长轴自身在图像里的角度. 用 /grasp/calib_yaw_ref 读取填入.
    yaw_target_theta_img_ = getOrDeclare<double>("yaw_target_theta_img", -0.4);
    yaw_align_tol_deg_ = getOrDeclare<double>("yaw_align_tol_deg", 2.0);
    // 试探步幅(度): 每轮开头先转这么多, 实测 dθ_img/dψ (符号+增益一起量到), 再解剩余步长.
    // 不用预设常数: 2026-07-28 实测该斜率在不同角区连符号都会翻(见 stageAlignYaw 注释).
    // 10° 够用: 检测噪声 ±1°, 信噪比 10:1, 且猜错方向也只多绕 10°.
    yaw_probe_deg_ = getOrDeclare<double>("yaw_probe_deg", 10.0);
    // 先验斜率: 第一步直接按它下发, 省掉无条件试探那 2s. 被单调性检查证伪才回退去实测.
    yaw_slope_prior_ = getOrDeclare<double>("yaw_slope_prior", 0.8);
    // 实测斜率下限: |dθ_img/dψ| 低于此判"检测不跟随腕转"(视野甩飞/盒子出画), 中止.
    yaw_slope_min_ = getOrDeclare<double>("yaw_slope_min", 0.3);
    // 解出的步长打这个折再走, 留余量吸收斜率的局部非线性, 宁可多迭代一步也不过冲.
    yaw_step_gain_ = getOrDeclare<double>("yaw_step_gain", 0.9);
    // 单步转动上限(度): 斜率量歪时兜住, 不让一步就把腕甩过去.
    yaw_max_step_deg_ = getOrDeclare<double>("yaw_max_step_deg", 60.0);
    yaw_align_max_steps_ = static_cast<int>(
      getOrDeclare<int64_t>("yaw_align_max_steps", 4));
    // θ_img 单帧有噪声, 取几帧均值再判. 检测 ~11Hz, 5 帧约 0.5s.
    axis_avg_frames_ = static_cast<int>(getOrDeclare<int64_t>("axis_avg_frames", 3));
    // 采 θ_img 前先丢弃这么多秒内到达的帧 (排空视觉管线). settle() 只等关节停稳, 不等
    // 检测管线 —— 相机曝光到 NCNN 推理出结果有一整段延迟, 臂停下那一刻管线里积压的还是
    // 运动中拍的画面. 2026-07-28 实测: ② 结束后 0.36s 采到 +10.9°, 而同一姿态停稳后稳定
    // 读 -45.1°, 那个坏数把试探斜率算成 6.21(真值 ~0.8), 整段对齐随之失败.
    // 时间戳是接收时刻不是曝光时刻, 所以判不出某帧"拍于何时", 只能按延迟上界等过去.
    axis_flush_sec_ = getOrDeclare<double>("axis_flush_sec", 0.3);
    // θ_img 帧间一致性下限 (2θ 空间合成向量的集中度, 全一致=1). 见 avgAxisAngle:
    // 低于此判长短轴互换致均值被抵消, 拒掉重采. 0.9 对应帧间散布约 ±25°, 远松于实测
    // 的 ±0.5°, 只拦真正的互换.
    axis_min_concentration_ = getOrDeclare<double>("axis_min_concentration", 0.9);
    // 一致性不过时最多重采几次 (每次隔 axis_flush_sec_). 互换往往只持续一两帧, 重采
    // 常能拿到干净的一批; 全失败才中止.
    axis_resample_tries_ = static_cast<int>(getOrDeclare<int64_t>("axis_resample_tries", 4));
    // 坏观测门限(度): |θ_img| 超此值判为 OBB 退化, 不采信. 见 stageAlignYaw 注释.
    yaw_bad_theta_deg_ = getOrDeclare<double>("yaw_bad_theta_deg", 85.0);
    // 本段累计转动上限(度): 防坏观测把腕净转到乱姿态.
    yaw_align_max_turn_deg_ = getOrDeclare<double>("yaw_align_max_turn_deg", 120.0);

    // 可抓候选数组 (PoseArray): 感知侧发的**过了 pick_z_max 的**盒子, 长度就是"还剩几个
    // 盒要抓". /grasp/look 把它报给调用方, 状态机据此决定同一货架循环抓几次, 不必在
    // mission.yaml 里按盒数写死几条 task.
    // 只用长度不用坐标: 坐标缓存下来跳过第二次识别会拆掉 ② 精修的目标锁 —— 那个锁必须在
    // 无遮挡的 look 位建立 (2026-07-30 目标中途被换掉的成因), 臂悬在盒上方时框会被吸盘
    // 遮掉一块, 重新锁定按面积最大就可能锁到旁边那个盒.
    poses_topic_ = getOrDeclare<std::string>("poses_topic", "/perception/object_poses");
    class_topic_ = getOrDeclare<std::string>("class_topic", "/perception/object_class");
    thickness_topic_ = getOrDeclare<std::string>("thickness_topic", "/perception/object_thickness");
    default_category_ = static_cast<int>(getOrDeclare<int64_t>("default_category", 1));
    // /grasp/look 摆到位后等一个**新鲜**候选数最多多久 (秒). 检测 ~11Hz, 但摆臂刚停那刻
    // 管线里积压的还是运动中拍的帧, 得等它过去 (同 axis_flush_sec / object_flush_sec 的
    // 道理). 1.5s 够: 实测 look 停稳后 0.3s 内就有新帧.
    look_count_wait_sec_ = getOrDeclare<double>("look_count_wait_sec", 1.5);

    // 运行时堆叠状态: 每托盘已放盒数 + 累计厚度. reset 服务清零(新一轮).
    tray_layers_.assign(num_trays_, 0);
    tray_stack_h_.assign(num_trays_, 0.0);
    unload_seq_ = 0;
    // 已放盒持久碰撞体按托盘分组, 与堆叠状态同生命周期 (reset_stack 一并清).
    placed_ids_.assign(num_trays_, {});
    placed_poses_.assign(num_trays_, {});
    placed_release_.assign(num_trays_, {});
    placed_th_.assign(num_trays_, {});

    // 卸货目的地 (base_link 系, top-down, 写死车右侧地面): 从托盘取盒后, 先到上方,
    // 再笛卡尔直下到吸盘末端 z=place_z_ (盒底离地 ~5mm) 才释放, 盒子落稳而非半空抛.
    // place_x/y 写死车右侧 (base_link -y = 车右); place_z_ = 释放时吸盘末端高度:
    // 盒高 0.025, 盒底离地 5mm -> 末端 = 0.005 + 0.025 = 0.030.
    place_x_ = node_->declare_parameter<double>("place_x", 0.0);
    place_y_ = node_->declare_parameter<double>("place_y", -0.38);
    place_z_ = node_->declare_parameter<double>("place_z", 0.030);
    place_clearance_ = node_->declare_parameter<double>("place_clearance", 0.12);
    // /grasp/level (手柄 △ 回正) 抬到的离地高度. 与 pregrasp_height 无关: 那个是"盒子上方
    // 12cm", 这个是"离地 20cm"的绝对高度, 点动待命位.
    level_height_ = getOrDeclare<double>("level_height", 0.20);

    // ---- 按托盘卸货 (/grasp/unload_tray): 两个卸货点沿 base x 轴对称分布 ----
    // 2026-07-30 实标 (RViz 拖臂到吸盘接地, 读 base_link->suction_tip):
    //   点1 (-0.090, -0.275, -0.044)   点2 (+0.087, -0.275, -0.044)   两点姿态一致 yaw≈-90°
    // 中心 x=-0.0015, 半间距 88.5mm. 基准写死实标值而非用 look 位 FK 现算: look 只是
    // "相机俯视货物"的姿态, 它的 TCP 位置 (y≈-0.201) 压根不是想要的落点, 绕一层 FK 只是
    // 引入不确定性. base_link 是底盘基系、臂固连底盘, 故整车移动时这两点跟着走, 不用重标.
    unload_base_x_ = getOrDeclare<double>("unload_base_x", -0.0015);
    unload_base_y_ = getOrDeclare<double>("unload_base_y", -0.275);
    unload_x_offset_ = getOrDeclare<double>("unload_x_offset", 0.0885);
    // 卸货点地面高度 (吸盘末端贴地时的 z). 释放高度 = 这个值 + 该盒厚度, 逐盒现算 ——
    // 一个托盘的两个盒厚度可能不同 (如 0 号盘装类别 3/4 = 12/15mm), 固定释放高度会有一个
    // 放偏. 厚度口径沿用 fallback_thickness 那套实跑反推的等效值 (非卡尺物理盒高), 与取盒
    // 共用同一体系, 取放两头一起对.
    unload_z_ = getOrDeclare<double>("unload_z", -0.044);
    // 取盒时吸盘要压到已落盒顶面. 释放位姿并不等于盒子落定后的高度: 放置时刻意在盒底与
    // 托盘面间留了空隙 (tray_z 那次"抬 1cm 让盒子松松落进去而不是被压着塞"), 释放后盒子
    // 还要自己往下掉这一截, 原路返回必然差这么多.
    // 按 category_ids 顺序逐类别给 (负值=多压): 差多少是这个类别盒子自己的性质 (盒面软硬、
    // 落进托盘沉多少), 与它摞在第几层无关.
    // 2026-07-30 实跑标定: 先全取 -0.010, 0 号盘两盒观察到类别3(12mm) 压多了要抬 2mm ->
    //   -0.008, 类别4(15mm) 还要再压 2mm -> -0.012. 类别 1/2 尚无观察, 暂留 -0.010.
    // 2026-08-13 一度再下压 5mm 后撤回, 理由记的是"卸货整段更贴地 -> 放地释放的竖直退开
    //   被判起始态碰撞(覆盖 0%)". 2026-08-14 该归因**已推翻**: 覆盖 0% 的根因是碰撞体
    //   没清干净(见下方 hideTrayBoxes 注释的 removeCollisionObjects 异步竞态), 与下压
    //   深度无关, 那次撤回属误判.
    // 2026-08-14 恢复下压 5mm 并"写死": 与 place.yaml 逐字同步 ——
    //   ⚠️ 只改这里不改 yaml 是**无效**的 (getOrDeclare: yaml 有值就不用默认), 反之亦然.
    unload_pick_shortfall_ = getOrDeclare<std::vector<double>>(
      "unload_pick_shortfall", {-0.015, -0.015, -0.013, -0.017});

    // 卸货点释放朝向 (吸盘朝下, yaw≈-90°), 2026-07-30 RViz 实标两点读得且两点一致.
    // 必须与托盘释放朝向 (tray_q*, yaw≈-175°) 区分: 后者为对齐托盘围栏左旋过, 沿用它
    // 盒子会歪 85° 落地 (实跑观察). 这个 yaw 也正是 look 位的 TCP 朝向.
    // 卸货这一段的横移高度, 替代抓取放置用的 transit_z_(0.26). transit_z_ 那么高是为了
    // "携盒越过别盘那摞 2 层盒的顶", 而卸货目的地在地面、路上没有那摞盒, 白飞 10cm.
    // 下限由托盘围栏顶(实测 ~0.120)定: 盒 attach 时 touch_links 含 tray_frame_, 盒↔托盘
    // 碰撞全程豁免, 低于围栏顶横穿会蹭侧壁而规划器不报、执行不停, 只能靠人看.
    // 2026-07-30 实跑发现挑不出一个定值: 0.16 时 transit 规划成功(68点)但直下只覆盖 87%
    // (< 0.9 阈值), 抬到 0.20 反而 transit 本身规划不出来, 0.26 两段都通. 卸货点这一带的
    // IK 可达区间很窄且两段各有各的约束, 中间高度落在两个可行构型之间的空隙里.
    // 故改成候选列表从低到高试, 取第一个两段都过的 —— 想走低的意图由排序表达, 可行性交给
    // 规划器判, 不再靠人猜一个数.
    // 2026-07-30 收尾: 候选只留 0.26 —— 试了 0.16/0.18/0.20 一路都不成. 0.16 那档最接近:
    // 取盒段 100% 通过, 放置段试算 92% 也过, 但执行完 transit 后按实测起点重算掉到 87%
    // (试算用的是 transit 终点理论关节值, 实际停位有误差, 92% 的裕度一吃就破阈值).
    // 即低高度是"理论上刚过、实测就不过"的边缘可行, 不是稳定可用. 0.26 是唯一实跑验证过的
    // 高度(四个盒两轮全成功), 空行程多几厘米换稳定, 值得. 候选机制留着, 以后要试新高度
    // 往这个列表前面加一档即可, 不用改代码.
    unload_transit_candidates_ = getOrDeclare<std::vector<double>>(
      "unload_transit_candidates", {0.26});
    // 装货(放托盘)的 transit 高度候选, 同样从低到高试. 默认只给 transit_z_ 那一档, 由
    // place.yaml 显式配多档 —— 默认值保守是为了参数没配时行为与改动前完全一致.
    transit_candidates_ = getOrDeclare<std::vector<double>>(
      "transit_candidates", {transit_z_});

    // 纯 yaw -90° (roll/pitch 实标 ≈0, 吸盘严格朝下): q = (0,0,sin-45°,cos-45°).
    unload_quat_.x = getOrDeclare<double>("unload_qx", 0.0);
    unload_quat_.y = getOrDeclare<double>("unload_qy", 0.0);
    unload_quat_.z = getOrDeclare<double>("unload_qz", -0.707107);
    unload_quat_.w = getOrDeclare<double>("unload_qw", 0.707107);

    // 被抓盒子尺寸 (world grasp_box: 0.09x0.055x0.025), 吸取后 attach 到吸盘作碰撞体,
    // 让放置规划知道吸盘下挂着盒 -> 绕开托盘边框, 不再侧向蹭入。
    // 2026-07-29: xy 从 90x55 缩到 88x53 给围栏开口留 2mm 余量 (实测 90x55 进开口直下覆盖
    //   只 87%, 盒微偏就蹭墙). z 不在此用 — attachBox 改用按类别传入的真实厚度.
    // getOrDeclare: yaml 可覆盖, 调参不用重编.
    box_size_x_ = getOrDeclare<double>("box_size_x", 0.088);
    box_size_y_ = getOrDeclare<double>("box_size_y", 0.053);
    box_size_z_ = node_->declare_parameter<double>("box_size_z", 0.025);

    // 看货姿势: ready 位基础上把 J1(Joint_11)+90°, 让手眼相机转向货物侧, 供闭环抓取前
    // "让视觉看见". Joint_11 限位 [0,4.747], ready=2.417 +1.571=3.988 在限位内.
    j1_name_ = node_->declare_parameter<std::string>("j1_name", "Joint_11");
    look_j1_offset_ = node_->declare_parameter<double>("look_j1_offset", 1.5707963);

    // ② 精修步进参数. 收敛判据是相机系横向误差 cam_tol_.
    // 试探位移(米): 开头量映射用的两次小幅水平移动幅度. 太小则检测噪声占比高, 量不准;
    // 太大则一次试探就把盒子推出视野边缘. 20mm 对 1~2mm 噪声有 10:1 信噪比.
    probe_step_ = node_->declare_parameter<double>("probe_step", 0.02);
    refine_max_steps_ = node_->declare_parameter<int>("refine_max_steps", 6);
    // 步长折扣: 解出来的位移乘这个再走. 留 20% 余量吸收映射误差, 宁可多走一步也不过冲.
    refine_step_gain_ = node_->declare_parameter<double>("refine_step_gain", 0.8);
    // 单步位移上限(米). 2026-07-29 从 0.04 抬到 0.08: ② 起始误差偶发到 118~152mm 时,
    // 前 3 步全顶在 40mm 天花板上白耗, 5 步才收敛, 逼近 refine_max_steps 的 6 步余量.
    // 抬到 80mm 后同样开局 2~3 步收完. 上限本身只是防"雅可比量歪时一步冲出去".
    // ⚠️ 别再照"误差与盒离相机距离成正比"那条解释调参 —— 已被数据否掉: 误差不随距离平滑
    // 放大而是跳变(前 6cm 稳在 30~39mm), 且同一位置(相距 6mm)出现过 32.8 与 127.4mm 两种
    // 结果. 大误差的真身疑为感知订阅被饿死后拿到运动中的积压旧帧, 见订阅回调组处注释.
    refine_max_step_ = node_->declare_parameter<double>("refine_max_step", 0.08);
    // 步进段规划降速. 0.15 是最初怕抖留的余量; 实测步进本身自带加减速, 不抖, 提到 0.4
    // 与全局同速, 精修那几步的起停停顿明显变短.
    refine_velocity_scaling_ = node_->declare_parameter<double>("refine_velocity_scaling", 0.4);
    // 每次取位置前平均几帧检测: 单帧 p_cam 横向有 1~2mm 噪声, 不平均则 6mm 阈值会被噪声
    // 反复触发又取消.
    cam_avg_frames_ = node_->declare_parameter<int>("cam_avg_frames", 3);
    cam_wait_timeout_ = node_->declare_parameter<double>("cam_wait_timeout", 3.0);
    refine_timeout_ = node_->declare_parameter<double>("refine_timeout", 60.0);
    object_stale_sec_ = node_->declare_parameter<double>("object_stale_sec", 0.5);
    // 取 object_pose 前先丢弃这么多秒内到的帧, 排空视觉管线. 与 axis_flush_sec_ 同一道理:
    // 相机曝光到 NCNN 出框有一整段延迟, 摆臂刚停那刻管线里积压的仍是运动中拍的画面, 而
    // 感知端用**当前** TF 把它转到 base_link -> 图像姿态与 TF 姿态错配, 坐标整体偏掉.
    // 时间戳是接收时刻不是曝光时刻, 判不出某帧拍于何时, 只能按延迟上界等过去.
    // 2026-07-29 实测(look 位, 盒静止): 摆臂中连续帧报 base_link (-0.199,-0.257) ->
    // (-0.249,-0.227) 一路滑 5cm, 停稳后稳定在 (-0.035,-0.351). 而 look 一返回就抓第一帧
    // 正落在这个瞬态里 -> ① 粗定位开偏 12~16cm, ② 精修被迫从 118~146mm 往回爬.
    object_flush_sec_ = getOrDeclare<double>("object_flush_sec", 0.3);
    // 采信前要连采几帧一致值, 及帧间散布上限(米). 排空是按上界等, 一致性门才是真判据:
    // 运动中的帧彼此在滑(上例每帧滑 ~8mm), 静止帧则完全重合(实测连续帧位数全同).
    object_consist_frames_ = getOrDeclare<int>("object_consist_frames", 3);
    object_consist_tol_ = getOrDeclare<double>("object_consist_tol", 0.010);
    object_resample_tries_ = getOrDeclare<int>("object_resample_tries", 4);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 感知订阅单独一个 Reentrant 回调组, 不能留在默认组.
    // 默认组在 MultiThreadedExecutor 下是**互斥**的, 而 move_group 的 action 客户端回调
    // (Execute request / current_state_monitor) 也落在默认组: ① 执行那几秒它占住默认组,
    // 同组的感知订阅回调只能排队. 2026-07-30 实测: ① 执行完到 ② 取样的 3 秒里 yolo 每秒
    // 稳定发 11~12 帧 (日志逐秒计数), grasp_node 一帧未收, avgCamPoint 等满 cam_wait_timeout_
    // 报 "② 无新鲜 p_cam". 时钟无偏差(实测 age 0.14~0.19s), 检测未断流, 手调
    // calib_cam_target 立刻能读到新鲜值.
    // ⚠️ 2026-07-30 修正: 当初据上面三条断言"只有执行期间被饿死能解释", 那个推断是错的 ——
    // 同日定位到"② 无新鲜 p_cam"的真身在感知侧: yolo 的 pick_z_max 把候选全滤掉时会连
    // object_point_cam 一起哑, 而托盘上有已放盒时候选=2, 旧的单候选兜底正好失效
    // (已由 yolo 侧改最近邻跟踪修掉). 分组本身仍然要留: 默认组互斥是事实, 感知回调与
    // move_group action 回调同组确实会排队, 只是它不是那次故障的成因.
    // 另一种可能的表现(仍是**未证实**的猜测, 别当结论): 挤进来的是积压的旧帧(① 运动中拍的),
    // ② 拿它当起点, 起始误差虚高. 这条能自洽解释 2026-07-29 那批 118~152mm 异常, 以及
    // "Δp_cam 模长 96mm 对应 Δp_base 模长 11mm"的矛盾(base = R·p_cam + t, R 保长度, 同一 R
    // 压不出这个比例; 若两帧来自不同臂构型则 R 不同, 矛盾消解). 但 2026-07-30 改完分组 +
    // 感知侧最近邻跟踪后连跑五轮, ② 起始误差稳定在 36~41mm, 那批异常再没复现过, 所以到底
    // 是分组治好的还是感知侧治好的, 无法区分. 别据此再推新结论.
    // 用 Reentrant 而非 MutuallyExclusive: 五个感知回调彼此无共享状态(各自锁各自的 mutex),
    // 允许并发进入, 一个都不会被另一个挡住.
    perc_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions perc_opt;
    perc_opt.callback_group = perc_cb_group_;

    object_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      object_topic_, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(obj_mtx_);
        last_object_ = *msg;
        have_object_ = true;
      }, perc_opt);

    // 可抓候选数: 只记数组长度与到达时刻, 供 /grasp/look 报给调用方.
    // 空数组也要收 (感知侧刻意在"一个都没检测到"时也发): 抓完最后一个盒正是靠它归 0,
    // 若只在非空时更新, 计数会永远停在 1, 状态机的同货架循环停不下来.
    poses_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
      poses_topic_, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(poses_mtx_);
        last_pick_count_ = static_cast<int>(msg->poses.size());
        last_poses_stamp_ = node_->now();
      }, perc_opt);

    // B 路线: 队友视觉侧额外发的"当前目标类别 + 厚度(米)". 回调只缓存, 放置时取最新.
    // 收不到则 execute 用 default_category_ + fallback_thickness_ 兜底, 不阻塞抓取.
    class_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      class_topic_, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Int32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(cls_mtx_);
        last_category_ = msg->data;
        have_category_ = true;
      }, perc_opt);
    cam_point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      cam_point_topic_, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(cam_mtx_);
        last_cam_point_ = *msg;
        have_cam_point_ = true;
      }, perc_opt);
    // θ_img: OBB 长轴在彩色图像里的角度(度). 只在 ① 之前(腕仍在 coarse_yaw_、相机没被
    // 吸盘遮挡)采样一次; ② 之后臂已贴近, 吸盘悬在盒上方挡住视野, 此时的检测不可信.
    axis_angle_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      axis_angle_topic_, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(axis_mtx_);
        last_axis_angle_ = msg->data;
        last_axis_stamp_ = node_->now();
        have_axis_angle_ = true;
      }, perc_opt);
    thickness_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      thickness_topic_, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(cls_mtx_);
        last_thickness_ = msg->data;
        have_thickness_ = true;
      }, perc_opt);

    pump_pub_ = node_->create_publisher<std_msgs::msg::Int8>(pump_topic_, 10);
    // /collision_object 直发兜底: psi_->removeCollisionObjects 是异步且偶发不生效
    // (2026-07-29 实测 placed_t1_l1 在 reset_stack 后仍留在场景, 挡下次放置直下段).
    // reset_stack 调完 PSI 后再用这个 publisher 同步发一遍 REMOVE, 绕过 PSI 异步链路.
    co_pub_ = node_->create_publisher<moveit_msgs::msg::CollisionObject>("/collision_object", 10);
    // move_group 的 apply_planning_scene 是**同步**服务: 返回即表示 diff 已并进场景.
    // 上面 PSI 与 /collision_object 两条都是异步且实测都可能不生效, 故 reset_stack 以它兜底.
    // 单独一个回调组: reset_stack 回调在 srv_cb_group_ 里阻塞等这个响应, 同组会自锁.
    scene_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    apply_scene_cli_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene", rmw_qos_profile_services_default, scene_cb_group_);
    get_scene_cli_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene", rmw_qos_profile_services_default, scene_cb_group_);

    // /grasp/execute 回调整轮阻塞几十秒(精修 20s 循环等). 若与 object_sub / servo 客户端
    // 同处默认互斥组, 阻塞期间它们全被饿死: object_pose 不更新→精修永远判过期超时;
    // start_servo 响应回调跑不了→fut.wait_for 直接超时. 故给服务单独一个互斥组, 配合
    // MultiThreadedExecutor: onExecute 占自己线程阻塞时, 订阅与客户端响应在别的线程照常跑.
    // execute 与 unload 共用同一 MutuallyExclusive 组: 二者都整轮阻塞几十秒, 且绝不能并发
    // (共用一条机械臂); 同组保证互斥, 又与 object_sub/servo 客户端分处不同线程不互相饿死.
    srv_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/execute",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) { onExecute(req, res); },
      rmw_qos_profile_services_default, srv_cb_group_);
    unload_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/unload",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) { onUnload(req, res); },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 底盘行进前把臂摆 ready (mm_task S0 调): 臂收身前, 底盘不拖着伸出的臂走.
    ready_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/ready",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = moveToReady();
        res->message = res->success ? "arm at ready" : "move to ready failed";
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 收拢姿势 (SRDF group_state "home"): 手柄遥控 HOME 态 / 断电前收臂用.
    // 与 ready 的分工: ready 是"底盘可走"的行进姿态, home 是停机姿态, 二者都不伸出。
    home_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/home",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = moveToHome();
        res->message = res->success ? "arm at home" : "move to home failed";
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 姿态回正 + 抬到固定高度 (手柄 △ 键): 点动把腕转歪之后一键找回可用状态.
    // 腕姿态回到 coarse_yaw_ —— 必须与①粗定位同一个朝向, 否则 cam_target 标定不成立
    // (相机装在腕上, 换姿态就偏; 见 stageCoarse 注释里 -177° 偏 55mm 那次).
    // x/y 保持不动: 只回姿态和高度, 不替人挪位置.
    level_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/level",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = levelAndLift(res->message);
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 放回地面 (手柄 ○ 键): 落点取当前 TCP 正下方 —— 盒本来就是从那儿抓起来的, 原地放回
    // 不引入新的可达性风险. 释放高度/离地间隙用既有标定 place_z_ / place_clearance_.
    place_ground_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/place_ground",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        geometry_msgs::msg::PoseStamped cur;
        if (!currentTcp(cur)) {
          res->success = false; res->message = "取当前位姿失败"; return;
        }
        const double x = cur.pose.position.x, y = cur.pose.position.y;
        RCLCPP_WARN(logger_, "==== /grasp/place_ground: 原地放回地面 (%.3f,%.3f) ====", x, y);
        res->success = placeAt(x, y, place_z_, place_clearance_);
        res->message = res->success ? "已放回地面并释放" : "放回地面失败(盒可能仍吸着)";
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 看货姿势 (mm_task 抓取前 S3 调): ready + J1+90°, 相机转向货物再做闭环抓取.
    // message 带上"看见几个可抓盒": 状态机据此决定同一货架循环抓几次, 不必在 mission.yaml
    // 里按盒数写死几条 task. 格式固定为 "arm at look pose, pickable=N", 调用方按前缀解析.
    // 摆到 look 位后要等一会儿再读: moveToLook 收尾的 settle() 只等关节停稳, 而相机曝光到
    // NCNN 出框有一整段延迟, 那一刻缓存里还是运动中拍的画面 (同 axis_flush_sec 的道理).
    look_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/look",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = moveToLook();
        if (!res->success) { res->message = "move to look failed"; return; }
        int n = 0;
        const bool fresh = waitPickCount(n, look_count_wait_sec_);
        const int free_slots = trayFreeTotal();
        char buf[128];
        std::snprintf(buf, sizeof(buf), "arm at look pose, pickable=%d, tray_free=%d%s",
                      n, free_slots, fresh ? "" : " (计数已过期, 感知侧可能没在发)");
        RCLCPP_INFO(logger_, "看货姿势: 可抓候选 %d 个, 托盘余位 %d%s",
                    n, free_slots, fresh ? "" : " [过期]");
        res->message = buf;
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 只抓不放 (真机分段验证用): 跑完①粗定位②精修③直插吸取就停, 盒仍吸着不放置.
    // 抓取三段与放置段分开验, 避免第一次就连跑到"已吸着盒、臂折在肩后"才暴露问题.
    // 停下后由人判断: 抓稳了就调 /grasp/unload 或手动松气泵, 不满意就急停.
    // 刻意不看 dry_run: 保留 dry_run=true 拦住 /grasp/execute 的整轮真跑, 同时能单验抓取.
    pick_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/pick_only",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        RCLCPP_WARN(logger_, "==== /grasp/pick_only: 只抓不放, 吸取后停住 ====");
        std::string err;
        res->success = pickCycle(err, true);
        res->message = res->success ? "pick done (盒仍吸着, 未放置)" : err;
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 只放不抓 (真机分段验证的另一半, 与 pick_only 对称): 盒已吸在吸盘上时调,
    // 跳过抓取三段, 直接抬起->到托盘正上方->垂直直下->释放, 放成功才计堆叠.
    // 没有它就只能靠 /grasp/execute 验放置, 而那个会先去抓下一个盒, 手上这个放不掉.
    // 前提: 盒确实吸着. 本服务自己补 attach (pick_only 已 attach 过, 重复 attach 幂等).
    place_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/place_only",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        RCLCPP_WARN(logger_, "==== /grasp/place_only: 只放不抓 (假定盒已吸着) ====");
        PlaceTarget pt;
        if (!resolvePlaceTarget("/grasp/place_only", pt, res->message)) {
          res->success = false; return;
        }
        attachBox(true, pt.thickness);
        if (!placeAtPose(pt.pose, pt.what.c_str(), pt.tray)) {
          res->success = false;
          res->message = std::string("放置失败: ") + pt.what;
          return;
        }
        if (dry_run_) {
          res->success = true;
          res->message = std::string("[dry_run] 放置轨迹规划通过: ") + pt.what;
          detachBox();
          return;
        }
        pushLayer(pt.tray, pt.thickness, pt.pose);
        if (!moveToReady()) {
          res->success = false; res->message = "放置后回 ready 失败"; return;
        }
        RCLCPP_INFO(logger_, "==== place_only 完成 (%s, 累计高 %.1fmm) ====",
                    pt.what.c_str(), trayStackH(pt.tray) * 1000);
        res->success = true;
        res->message = std::string("place done: ") + pt.what;
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 只跑①粗定位就停 (标定 cam_target 用, 也可单验①): 开到盒上方 pregrasp 高度不动手.
    // 标定必须停在与工作一致的高度上, 否则 p_cam 目标值换个高度就不成立.
    coarse_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/coarse_only",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        geometry_msgs::msg::PoseStamped obj;
        if (!waitObject(obj, 3.0)) {
          res->success = false; res->message = "无 object_pose"; return;
        }
        res->success = stageCoarse(obj);
        res->message = res->success ? "① 到盒上方, 已停住 (未精修/未吸取)" : "① 粗定位失败";
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 标定对准目标: 人工把吸盘对准盒心后调一次, 读当前 p_cam 打印出来, 填进 place.yaml
    // 的 cam_target_x/y. 只读不动臂.
    cam_cal_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/calib_cam_target",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        geometry_msgs::msg::PointStamped cp;
        if (!latestCamPoint(cp)) {
          res->success = false; res->message = "无新鲜 p_cam"; return;
        }
        char buf[192];
        std::snprintf(buf, sizeof(buf),
                      "cam_target_y: %.5f  cam_target_z: %.5f  (光轴 x=%.4f, frame=%s)",
                      cp.point.y, cp.point.z, cp.point.x, cp.header.frame_id.c_str());
        RCLCPP_WARN(logger_, "标定对准目标 -> 填进 place.yaml: %s", buf);
        res->success = true; res->message = buf;
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 手动 jog (标定用): 每次 call 读实时参数 jog_dx/dy/dz, 相对当前 TCP 走一步纯笛卡尔
    // 直线, 朝向不变. 用于 coarse_only 到 12cm 后: 下插(dz<0)接近盒 -> 微调 xy 对准盒心 ->
    // 纯竖直抬回(dz=+累计下插量) -> calib_cam_target 读值. 打印走前/走后 TCP 便于记累计量.
    jog_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/jog",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        const double dx = node_->get_parameter("jog_dx").as_double();
        const double dy = node_->get_parameter("jog_dy").as_double();
        const double dz = node_->get_parameter("jog_dz").as_double();
        const double dyaw = node_->get_parameter("jog_dyaw_deg").as_double();
        geometry_msgs::msg::PoseStamped before;
        if (!currentTcp(before)) {
          res->success = false; res->message = "取 TCP 失败"; return;
        }
        RCLCPP_WARN(logger_, "jog: 请求 (dx=%.4f dy=%.4f dz=%.4f dyaw=%.1f°) 从 TCP z=%.4f",
                    dx, dy, dz, dyaw, before.pose.position.z);
        if (!moveRelativeXYZ(dx, dy, dz)) {
          res->success = false; res->message = "jog 笛卡尔移动失败(覆盖不足/执行失败)"; return;
        }
        // 有转腕增量则原地绕吸盘轴转 (位置不变). 平移在前, 转腕在后, 一次 call 可只给其一.
        if (std::abs(dyaw) > 1e-6) {
          move_group_->setMaxVelocityScalingFactor(rotate_velocity_scaling_);
          move_group_->setMaxAccelerationScalingFactor(rotate_velocity_scaling_);
          const bool rok = jogRotateYaw(dyaw * M_PI / 180.0);
          move_group_->setMaxVelocityScalingFactor(1.0);
          move_group_->setMaxAccelerationScalingFactor(1.0);
          if (!rok) { res->success = false; res->message = "jog 转腕失败(规划/执行)"; return; }
        }
        geometry_msgs::msg::PoseStamped after;
        currentTcp(after);
        char buf[192];
        std::snprintf(buf, sizeof(buf),
                      "jog done: TCP base_link=[%.4f,%.4f,%.4f]",
                      after.pose.position.x, after.pose.position.y, after.pose.position.z);
        RCLCPP_WARN(logger_, "%s", buf);
        res->success = true; res->message = buf;
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 标定对齐判据: 人工把吸盘长轴转到与盒长轴贴合后调一次, 读回当时的 θ_img 就是
    // yaw_target_theta_img (吸盘长轴自身的图像角). 只读不动臂.
    // 相机固连腕, 故该值与腕在哪个构型无关; 腕 yaw 一并打印仅供记录, 不再是标定量.
    // 标定时相机必须能看清盒(吸盘别压在盒上), 否则 θ_img 不可信.
    yaw_cal_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/calib_yaw_ref",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        double theta = 0.0;
        if (!avgAxisAngle(theta)) {
          res->success = false; res->message = "无新鲜 θ_img"; return;
        }
        geometry_msgs::msg::PoseStamped tcp;
        if (!currentTcp(tcp)) {
          res->success = false; res->message = "取 TCP 位姿失败"; return;
        }
        char buf[192];
        std::snprintf(buf, sizeof(buf),
                      "yaw_target_theta_img: %.1f  (此刻腕 yaw %.1f°, 仅记录)",
                      theta, quatYaw(tcp.pose.orientation) * 180.0 / M_PI);
        RCLCPP_WARN(logger_, "标定对齐判据 -> 填进 place.yaml: %s", buf);
        res->success = true; res->message = buf;
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 堆叠计数清零 (mm_task 新一轮搬运开始时调): 每托盘层数与累计厚度归零.
    reset_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/reset_stack",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> lk(stack_mtx_);
        std::fill(tray_layers_.begin(), tray_layers_.end(), 0);
        std::fill(tray_stack_h_.begin(), tray_stack_h_.end(), 0.0);
        // 卸货落点序号一起归零: 下面把 placed_unloaded_* 也清了, 地上没盒了, 序号不归零
        // 会让下一轮第一个盒莫名去到第二个点.
        unload_seq_ = 0;
        // 已落盒碰撞体一并清出规划场景. PSI removeCollisionObjects 是异步, 偶发不生效
        // (2026-07-29 实测 placed_t1_l1 残留挡下次放置). 兜底: 紧接着直接发一遍 REMOVE
        // 到 /collision_object, 同步链路绕过 PSI. 即便 placed_ids_ 漏追踪(历史 bug),
        // 这条仍按已知命名 placed_t{T}_l{L} 扫一遍 0..capacity-1 兜掉.
        std::vector<std::string> all_ids;
        for (auto & ids : placed_ids_) {
          if (!ids.empty()) psi_->removeCollisionObjects(ids);
          for (const auto & id : ids) all_ids.push_back(id);
          ids.clear();
        }
        for (auto & poses : placed_poses_) poses.clear();
        for (auto & rel : placed_release_) rel.clear();   // 卸货取回用的吸盘位姿
        for (auto & th : placed_th_) th.clear();
        // 下面的命名规则盲扫 + removePlacedViaService 已经把 placed_unloaded_* 清了,
        // 自知清单也一并清空, 否则下一次 unloadTray 会拿着已经不存在的 id 去删,
        // 白白多打一次(无害但吵)日志.
        ground_placed_ids_.clear();
        // 兜底: 即便 placed_ids_ 空(计数漏追踪), 也按命名规则扫一遍清残留.
        // placed_unloaded_* 也要扫: 它是卸到**地面**的盒, 从不进 placed_ids_(那是托盘的账),
        // 所以上面那圈 placed_ids_ 完全覆盖不到它, 只能靠这里按命名清。
        for (int t = 0; t < num_trays_; ++t) {
          for (int L = 0; L < 64; ++L) {
            char buf[40]; std::snprintf(buf, sizeof(buf), "placed_t%d_l%d", t, L);
            all_ids.push_back(buf);
            std::snprintf(buf, sizeof(buf), "placed_unloaded_%d_%d", t, L);
            all_ids.push_back(buf);
          }
        }
        for (const auto & id : all_ids) {
          moveit_msgs::msg::CollisionObject co;
          co.id = id;
          co.header.frame_id = base_frame_;
          co.operation = co.REMOVE;
          co_pub_->publish(co);
        }
        // 上面两条(PSI + /collision_object 直发)都是异步且实测都可能不生效:
        // 2026-07-29 重启 grasp_node 后场景残留 placed_t* 挡放置, 两条路都没删掉,
        // 最终只有 apply_planning_scene 服务(同步)删得掉. 故以它为准再补一刀.
        const bool applied = removePlacedViaService();
        RCLCPP_INFO(logger_, "堆叠计数与已落盒碰撞体已清零 (%d 托盘, REMOVE %zu 条, 服务同步 %s)",
                    num_trays_, all_ids.size(), applied ? "成功" : "失败");
        res->success = applied;
        res->message = applied ? "stack counters reset"
                              : "计数已清零, 但场景残留碰撞体未删净 (见日志)";
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 重启后补登已物理存在的盒 (restart 会清空运行时堆叠状态, 但托盘上盒还在):
    // 读 seed_tray/seed_thickness 参数, 按当前该托盘 release z 公式补一个碰撞体+计数.
    // 每 call 一次补一个 (对应一个实体盒); 连续同盘补多个就多 call 几次.
    seed_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/seed_placed",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        const int tray = static_cast<int>(node_->get_parameter("seed_tray").as_int());
        const double th = node_->get_parameter("seed_thickness").as_double();
        if (tray < 0 || tray >= num_trays_) {
          res->success = false; res->message = "seed_tray 越界"; return;
        }
        // release 位姿 = 该托盘标定接触位姿, z = tray_z + 已累计 + 本盒厚 + 本层间隙
        // (与放置同式, 见 resolvePlaceTarget).
        geometry_msgs::msg::Pose pose = trayContactPose(tray);
        pose.position.z = tray_z_[tray] + trayStackH(tray) + th + layer_gap_;
        pushLayer(tray, th, pose);
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "补登 %d 号托盘第 %d 层 (厚 %.0fmm, 累计高 %.1fmm)",
                      tray, trayLayers(tray), th * 1000, trayStackH(tray) * 1000);
        RCLCPP_WARN(logger_, "%s", buf);
        res->success = true; res->message = buf;
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 按托盘卸货: 把 unload_tray 号托盘上的盒全卸到地面两个卸货点 (不用视觉, 见 unloadTray).
    // 与 execute/unload 同一个互斥回调组: 三者都整轮阻塞几十秒且共用一条臂, 绝不能并发.
    unload_tray_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/unload_tray",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        const int tray = static_cast<int>(node_->get_parameter("unload_tray").as_int());
        std::string err;
        res->success = unloadTray(tray, err);
        res->message = res->success
          ? std::to_string(tray) + " 号托盘已卸完"
          : err;
      },
      rmw_qos_profile_services_default, srv_cb_group_);
    // 单个卸货: 只卸栈顶一个, 不回 ready. 遥控 L1/L2 走这条 —— 一按一个, 人跟着把盒取走。
    // 与 unload_tray 分成两个服务而不是加参数: mm_task 状态机依赖"一次调用卸完整盘"
    // (mission_real.yaml 明写), 改那条的语义会连带改状态机的一轮时长与重试逻辑。
    // 同一个互斥回调组, 与 execute/unload_tray 三者绝不并发 (共用一条臂).
    unload_one_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/unload_one",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        const int tray = static_cast<int>(node_->get_parameter("unload_tray").as_int());
        std::string err;
        res->success = unloadTray(tray, err, true);
        res->message = res->success
          ? std::to_string(tray) + " 号托盘卸下 1 个, 余 " +
            std::to_string(trayLayers(tray)) + " 层"
          : err;
      },
      rmw_qos_profile_services_default, srv_cb_group_);

    RCLCPP_INFO(logger_,
                "grasp_node 就绪: group=%s ee=%s base=%s 订 %s 发 %s, "
                "服务 /grasp/execute /grasp/unload_tray /grasp/ready /grasp/look",
                planning_group_.c_str(), ee_link_.c_str(), base_frame_.c_str(),
                object_topic_.c_str(), pump_topic_.c_str());
  }

  void initMoveGroup()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_);
    psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    move_group_->setEndEffectorLink(ee_link_);
    move_group_->setPoseReferenceFrame(base_frame_);   // 所有位姿统一 base_link 系
    restorePlanScaling();
    // 放托盘目标 (Link_11 在肩正后方 ~(-0.18,0,0.076)) 需吸盘严格朝下, 手臂折回身后,
    // IK 硬、规划耗时: 默认 5s 常 abort. 加大规划时间与尝试次数 (也惠及粗定位/放置).
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    RCLCPP_INFO(logger_, "MoveGroup planning_frame=%s pose_ref=%s ee=%s",
                move_group_->getPlanningFrame().c_str(),
                move_group_->getPoseReferenceFrame().c_str(),
                move_group_->getEndEffectorLink().c_str());
  }

private:
  // 参数取值: 节点用 automatically_declare_parameters_from_overrides, place.yaml 里的键
  // 已被自动声明, 再 declare_parameter 会抛 already-declared. 已声明则直接取, 否则声明带默认.
  template <typename T>
  T getOrDeclare(const std::string & name, const T & def)
  {
    if (node_->has_parameter(name)) return node_->get_parameter(name).get_value<T>();
    return node_->declare_parameter<T>(name, def);
  }

  // 本轮放置目标: 由当前类别解出托盘号/本层厚度/释放位姿. execute 与 place_only 共用,
  // 保证两条入口算出的落点完全一致 —— place_only 验过的位姿就是 execute 会去的位姿.
  struct PlaceTarget
  {
    int tray{0};
    double thickness{0.0};
    geometry_msgs::msg::Pose pose;
    std::string what;
  };

  bool resolvePlaceTarget(const char * caller, PlaceTarget & out, std::string & err)
  {
    const int category = placeCategory();
    const int ci = categoryIndex(category);
    if (ci < 0) {
      err = "未知类别 " + std::to_string(category) + " (不在 category_ids)";
      RCLCPP_ERROR(logger_, "%s", err.c_str());
      return false;
    }
    const int tray = static_cast<int>(category_tray_[ci]);
    if (tray < 0 || tray >= num_trays_) { err = "类别映射托盘号越界"; return false; }
    const double thickness = currentThickness(ci);

    RCLCPP_INFO(logger_, "==== %s: 类别 %d -> %d号托盘, 厚度 %.1fmm, 已有 %d 层 ====",
                caller, category, tray, thickness * 1000, trayLayers(tray));

    // 满盘保护: 防撞已摞满的盒.
    // 前缀 TRAY_FULL: 是给状态机的机器可读信号 —— 货架盒数不定(1~4), 同货架连抓的终止
    // 条件有两个: 识别数归零, 或托盘装满. 后者不是故障而是"该去卸货了", 状态机不能把它
    // 当异常中止整条任务. 判据是**本类别映射到的那个盘**满没满, 不是总余量: 4 个同类盒
    // 只进一个盘(capacity 2), 装 2 个就得走, 另一个盘空着也没用.
    if (trayLayers(tray) >= static_cast<int>(tray_capacity_[tray])) {
      err = "TRAY_FULL: " + std::to_string(tray) + "号托盘已满(" +
            std::to_string(tray_capacity_[tray]) + ")";
      RCLCPP_WARN(logger_, "%s", err.c_str());
      return false;
    }

    out.tray = tray;
    out.thickness = thickness;
    // release z = 托盘空载接触 z + 累计下层高度 + 本层厚度 + 本层间隙.
    // (吸盘吸盒顶, 盒底落下层顶). trayStackH 里已含下面每一层各自的 layer_gap_, 故这里
    // 只补本层那一份 —— 第 1 层 +1×gap, 第 2 层 +2×gap, 逐层累加.
    out.pose = trayContactPose(tray);
    out.pose.position.z = tray_z_[tray] + trayStackH(tray) + thickness + layer_gap_;
    out.what = std::to_string(tray) + "号托盘第" +
               std::to_string(trayLayers(tray) + 1) + "层";
    return true;
  }

  // ---- 主流程 ----
  // /grasp/execute: 三段抓取源盒 -> 放到自己的托盘(Link_11).
  void onExecute(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    PlaceTarget pt;
    if (!resolvePlaceTarget("/grasp/execute", pt, res->message)) {
      res->success = false; return;
    }
    const int tray = pt.tray;
    const double thickness = pt.thickness;
    const geometry_msgs::msg::Pose & target = pt.pose;
    const char * what = pt.what.c_str();

    if (dry_run_) {
      // 安全测试(第①②③步): 不抓真盒, 挂个虚拟盒让规划考虑几何, 只走放置轨迹规划/慢跑,
      // 不吸不放不计堆叠. 走完清掉虚拟盒.
      attachBox(true, thickness);
      bool ok = placeAtPose(target, what, tray);
      detachBox();
      res->success = ok;
      res->message = ok ? std::string("[dry_run] 放置轨迹规划通过: ") + what
                        : std::string("[dry_run] 放置轨迹规划失败: ") + what;
      RCLCPP_WARN(logger_, "==== [dry_run] 一轮结束 (%s), 不计堆叠 ====", what);
      return;
    }

    // 取本轮开工前(仍在上一轮收尾的 look 位)已知的候选数: 这一帧是本轮 pickCycle 挑
    // 目标盒时相机看到的现状, 已经包含"即将被吸走的这一个". 若当时就只看到 1 个,
    // 吸走后剩 0 个是必然结论, 不需要再摆回 look 拍一帧去确认同一件事.
    int pre_pick_left = 0;
    {
      std::lock_guard<std::mutex> lk(poses_mtx_);
      pre_pick_left = last_pick_count_;
    }

    std::string err;
    // 盒 attach 后豁免与托盘接触: 放到位时盒底本就落在托盘面(标定 tray_z 比 Link_11 网格
    // 顶面低几 mm), 不豁免则直下段被判碰撞截断. 侧蹭边框的顾虑已由"正上方->垂直直下"
    // 的入位方式消除, 不再靠碰撞检测拦.
    if (!pickCycle(err, true)) { res->success = false; res->message = err; return; }

    if (!placeAtPose(target, what, tray)) {
      res->success = false; res->message = std::string("放置失败(已吸取): ") + what; return;
    }
    // 放置成功才计入堆叠: 层数+1, 累计厚度 += 本层厚度, 并留持久碰撞体.
    pushLayer(tray, thickness, target);

    // 货架取前就只看到这一个 -> 吸走后必然剩 0, 直接报完工, 不摆 look (2026-08-13 改):
    // 状态机只等 pickable=0 就会转去下一步(离开货架前它自己会调 /grasp/ready), 回 look
    // 只是白走一趟空行程, 且不产生任何新信息.
    if (pre_pick_left <= 1) {
      const int free_slots = trayFreeTotal();
      RCLCPP_INFO(logger_,
                  "==== 抓放一轮完成 (%s, 累计高 %.1fmm), 取前候选数 %d -> 货架已空, 不回 look ====",
                  what, trayStackH(tray) * 1000, pre_pick_left);
      res->success = true;
      res->message = std::string("grasp cycle done: ") + what +
                     ", shelf empty, skip look, pickable=0, tray_free=" + std::to_string(free_slots);
      return;
    }

    // 收尾摆到 look 而非 ready (2026-07-31 改): 下一轮开工需要的姿态是 look 不是 ready ——
    // pickCycle 第一步就取 object_pose, 前提是相机已经对着货物. 原先回 ready 使得同一货架
    // 连抓多个盒时每轮都要多走一趟 ready->look 的空行程, 而那一趟是整段里最长的.
    // 回 ready 只在"底盘要走了"时才必需 (臂收身前不拖着走), 那是状态机知道的事, 由它在
    // 离开货架前显式调 /grasp/ready.
    // ⚠️ 别把这里改回 ready 来"图省事": 状态机侧的同货架循环依赖收尾在 look.
    if (!moveToLook()) {
      res->success = false; res->message = "放置后摆看货姿势失败"; return;
    }

    int left = 0;
    waitPickCount(left, look_count_wait_sec_);
    const int free_slots = trayFreeTotal();
    RCLCPP_INFO(logger_,
                "==== 抓放一轮完成 (%s, 累计高 %.1fmm), 已在 look 位, 还剩 %d 个可抓, 托盘余位 %d ====",
                what, trayStackH(tray) * 1000, left, free_slots);
    res->success = true;
    res->message = std::string("grasp cycle done: ") + what +
                   ", at look pose, pickable=" + std::to_string(left) +
                   ", tray_free=" + std::to_string(free_slots);
  }

  // ---- 堆叠状态 / 类别映射 工具 ----
  int currentCategory()
  {
    std::lock_guard<std::mutex> lk(cls_mtx_);
    return have_category_ ? last_category_ : default_category_;
  }

  // 放置该用哪个类别: 优先"吸盘上这个盒"在下插时锁存的值, 没有才回退到当前缓存.
  // 为什么必须锁存: /perception/object_class 与 object_pose 是两个独立话题, grasp_node
  // 各存最新值。抓完盒还在吸盘上时 yolo 继续跑, 缓存会被视野里剩下的盒刷掉 ——
  // 放置时读它就放错盘 (2026-08-03 实机复现, 详见 stageInsert 里的锁存点).
  // 回退分支留给 /grasp/place_only 单独调用的场景 (真机分段验证: 盒是人手放上吸盘的,
  // 没走过 stageInsert, 此时当前缓存就是最好的信息).
  int placeCategory()
  {
    std::lock_guard<std::mutex> lk(cls_mtx_);
    if (have_held_category_) return held_category_;
    return have_category_ ? last_category_ : default_category_;
  }

  // 类别 -> category_ids 下标 (映射表/厚度表都按此下标索引). 找不到返回 -1.
  int categoryIndex(int category) const
  {
    for (size_t i = 0; i < category_ids_.size(); ++i) {
      if (static_cast<int>(category_ids_[i]) == category) return static_cast<int>(i);
    }
    return -1;
  }

  // 本层厚度(米): 只采信 place.yaml 的实测标定值; 视觉厚度仅打印对照, 不参与 release_z.
  // 视觉侧 table 深度取自 OBB 向内收缩的 ROI, 窗口内全是盒顶像素(实测深度散布仅 6~10mm),
  // table-d 恒落在 3mm 量级. 采信它会让 release_z 低 20mm+, 把盒压进托盘或顶死臂.
  double currentThickness(int ci)
  {
    double vision = -1.0;
    {
      std::lock_guard<std::mutex> lk(cls_mtx_);
      if (have_thickness_) vision = last_thickness_;
    }
    double th = 0.025;
    if (ci >= 0 && ci < static_cast<int>(fallback_thickness_.size())) {
      th = fallback_thickness_[ci];
    }
    if (vision > 0.0) {
      RCLCPP_INFO(logger_, "厚度: 采用标定值 %.1fmm (视觉报 %.1fmm, 仅对照)",
                  th * 1000, vision * 1000);
    }
    return th;
  }

  geometry_msgs::msg::Pose trayContactPose(int t) const
  {
    geometry_msgs::msg::Pose p;
    p.position.x = tray_x_[t];
    p.position.y = tray_y_[t];
    p.position.z = tray_z_[t];
    p.orientation.x = tray_qx_[t];
    p.orientation.y = tray_qy_[t];
    p.orientation.z = tray_qz_[t];
    p.orientation.w = tray_qw_[t];
    return p;
  }

  int trayLayers(int t)
  {
    std::lock_guard<std::mutex> lk(stack_mtx_);
    return tray_layers_[t];
  }

  double trayStackH(int t)
  {
    std::lock_guard<std::mutex> lk(stack_mtx_);
    return tray_stack_h_[t];
  }

  // 全部托盘还能放几个盒. 报给状态机作"还能不能接着抓"的参考量.
  // 只是参考不是判据: 真正拦下一轮的是 resolvePlaceTarget 的 TRAY_FULL —— 盒能不能放取决于
  // 它自己的类别映射到的那个盘满没满, 而下一个盒的类别在抓之前不知道. 总余量 >0 而映射盘
  // 已满是可能的, 那种情况由 TRAY_FULL 兜.
  int trayFreeTotal()
  {
    std::lock_guard<std::mutex> lk(stack_mtx_);
    int free_slots = 0;
    for (int t = 0; t < num_trays_; ++t) {
      free_slots += std::max(0, static_cast<int>(tray_capacity_[t]) - tray_layers_[t]);
    }
    return free_slots;
  }

  // 放置成功后调: 层数/累计厚度 +1, 并在规划场景留一个持久世界碰撞体代表这个已落盒.
  // release_pose = 释放时吸盘位姿 (盒顶贴吸盘末端), 盒心在其 -Z 方向半个盒高处.
  // 该碰撞体不 detach, 生命周期到 reset_stack —— 后续放别盘时 MoveIt 自动避让, 不再横扫.
  void pushLayer(int t, double thickness, const geometry_msgs::msg::Pose & release_pose)
  {
    std::lock_guard<std::mutex> lk(stack_mtx_);
    tray_layers_[t] += 1;
    // 累计高度含本层间隙: 下一层的释放高度由它打底, 故间隙必须进累计量, 否则第 2 层只会
    // 抬自己那 5mm 而不叠上第 1 层的 5mm (用户要的是逐层累加).
    tray_stack_h_[t] += thickness + layer_gap_;

    geometry_msgs::msg::Pose box = release_pose;
    box.position.z -= box_size_z_ / 2.0;   // 盒心在吸盘末端下方半个盒高 (工具朝下)
    const std::string id = "placed_t" + std::to_string(t) + "_l" +
                           std::to_string(tray_layers_[t]);
    placed_ids_[t].push_back(id);
    placed_poses_[t].push_back(box);
    // 存吸盘释放位姿与本层厚度, 供卸货(popLayer)原路取回. placed_poses_ 存的是**盒心**
    // (碰撞体要盒心), 而卸货要回到**吸盘**位姿, 二者差半个盒高, 故释放位姿单独存一份.
    placed_release_[t].push_back(release_pose);
    placed_th_[t].push_back(thickness);
    addPlacedBox(id, box);
  }

  // pushLayer 的逆操作 (卸货用): 摘掉栈顶那一层. 后进先出 —— 摞的时候后放的在上面,
  // 取的时候必须先取上面那个, 否则从下面抽会带翻上层.
  // 返回该层的释放位姿 (= 当初放它时吸盘的位姿, 卸货取盒就回到这个位姿去吸) 与厚度.
  // 场景碰撞体一并删掉: 盒已经被取走, 留着会挡住后续规划 (幽灵碰撞体教训).
  bool popLayer(int t, geometry_msgs::msg::Pose & release_pose, double & thickness)
  {
    std::string id;
    {
      std::lock_guard<std::mutex> lk(stack_mtx_);
      if (t < 0 || t >= static_cast<int>(placed_ids_.size()) || placed_ids_[t].empty()) {
        return false;
      }
      release_pose = placed_release_[t].back();   // 吸盘位姿 (非盒心), 原路取回
      thickness = placed_th_[t].back();
      id = placed_ids_[t].back();
      placed_poses_[t].pop_back();
      placed_release_[t].pop_back();
      placed_th_[t].pop_back();
      placed_ids_[t].pop_back();

      tray_layers_[t] -= 1;
      tray_stack_h_[t] -= thickness + layer_gap_;   // 与 pushLayer 对称, 含本层间隙
      if (tray_layers_[t] < 0) tray_layers_[t] = 0;
      if (tray_stack_h_[t] < 0.0) tray_stack_h_[t] = 0.0;

      RCLCPP_INFO(logger_, "取走 %s (厚 %.1fmm), %d 号托盘余 %d 层, 累计高 %.1fmm",
                  id.c_str(), thickness * 1000, t, tray_layers_[t], tray_stack_h_[t] * 1000);
    }
    // 同步删 (锁外, 服务调用会阻塞几十 ms): 盒已在吸盘上, 场景里那个碰撞体必须在下一次
    // 规划前真的消失, 否则它与 attach 的 carried_box 重叠, 起点即判碰撞 (同 hideTrayBoxes
    // 那条竞态, 2026-07-31).
    if (!removeObjectsSync({id})) {
      psi_->removeCollisionObjects({id});
      RCLCPP_WARN(logger_, "取走 %s: 同步 REMOVE 失败, 已退回异步", id.c_str());
    }
    return true;
  }

  // 把一个已落盒作为独立世界碰撞体加入规划场景 (base_link 系).
  void addPlacedBox(const std::string & id, const geometry_msgs::msg::Pose & pose)
  {
    moveit_msgs::msg::CollisionObject co;
    co.id = id;
    co.header.frame_id = base_frame_;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions = {box_size_x_, box_size_y_, box_size_z_};
    co.primitives.push_back(prim);
    co.primitive_poses.push_back(pose);
    co.operation = co.ADD;
    psi_->applyCollisionObjects({co});
    RCLCPP_INFO(logger_, "已落盒 %s 留作规划场景碰撞体 (%.3f,%.3f,%.3f)",
                id.c_str(), pose.position.x, pose.position.y, pose.position.z);
  }

  // 问 move_group 当前场景里都有哪些世界碰撞体 (同步服务, 返回即是权威现状).
  // 用途: reset_stack 据此只删真实存在的 id, 并在删后复查是否真的清掉了.
  bool listSceneObjects(std::vector<std::string> & out)
  {
    if (!get_scene_cli_->wait_for_service(2s)) return false;
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components = req->components.WORLD_OBJECT_NAMES;
    auto fut = get_scene_cli_->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready) return false;
    out.clear();
    for (const auto & co : fut.get()->scene.world.collision_objects) out.push_back(co.id);
    return true;
  }

  // 用 move_group 的 apply_planning_scene 服务同步删掉场景里所有 placed_* 碰撞体.
  // 为什么需要这条第三路: PSI removeCollisionObjects 与 /collision_object 直发都是异步的,
  // 2026-07-29 实测重启 grasp_node 后场景里残留的 placed_t* 两条路都删不掉, 只有这个同步
  // 服务删得动 —— 残留盒挡住放置的 transit 规划与直下段, 表现为"覆盖不足 44%"/"plan 失败".
  // 删哪些由**现场**决定而不是按命名规则盲扫 0..63: 盲扫会把上百个不存在的 id 塞进 diff,
  // move_group 对不存在的 id 整体返回 success=false, 于是返回值分不出"真失败"与"删了但
  // 顺带扫了空 id"(2026-07-29 实测: 128 条 REMOVE 报失败, 场景其实已空). 改成先查现状、
  // 只删存在的、删完复查, 返回值才是可信的判据.
  // 空 primitives 的 REMOVE 条目按 id 删是合法的, 但 header.frame_id 必须给, 否则
  // move_group 报 "Unknown frame" 整个 diff 被拒.
  bool removePlacedViaService()
  {
    std::vector<std::string> ids;
    if (!listSceneObjects(ids)) {
      RCLCPP_WARN(logger_, "get_planning_scene 不可用, 跳过同步 REMOVE");
      return false;
    }
    std::vector<std::string> targets;
    for (const auto & id : ids) {
      if (id.rfind("placed_", 0) == 0 || id == kCarriedBoxId) targets.push_back(id);
    }
    if (targets.empty()) return true;
    if (!apply_scene_cli_->wait_for_service(2s)) {
      RCLCPP_WARN(logger_, "apply_planning_scene 服务不可用, 跳过同步 REMOVE");
      return false;
    }
    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    req->scene.is_diff = true;
    req->scene.robot_state.is_diff = true;
    for (const auto & id : targets) {
      moveit_msgs::msg::CollisionObject co;
      co.id = id;
      co.header.frame_id = base_frame_;
      co.operation = co.REMOVE;
      req->scene.world.collision_objects.push_back(co);
    }
    auto fut = apply_scene_cli_->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready) {
      RCLCPP_WARN(logger_, "apply_planning_scene 无响应");
      return false;
    }
    // 复查: 服务 success 只说 diff 被接受, 以场景里 placed_* 是否真没了为准.
    std::vector<std::string> after;
    if (!listSceneObjects(after)) return false;
    for (const auto & id : after) {
      if (id.rfind("placed_", 0) == 0) {
        RCLCPP_ERROR(logger_, "同步 REMOVE 后场景仍残留 %s", id.c_str());
        return false;
      }
    }
    RCLCPP_INFO(logger_, "同步 REMOVE 清掉 %zu 个已落盒碰撞体", targets.size());
    return true;
  }

  // 清掉 ground_placed_ids_ 里记录的地面卸货碰撞体 (unloadTray 每次进入时调, 清上一趟残留).
  // ⚠️ 不走 listSceneObjects() 现查现场再筛: 那条查询用的是 get_planning_scene 服务,
  // 2026-08-13 实测在 move_group 负载高时会卡住/超时, 而 if(查询失败) 分支当时被写成
  // "静默跳过、不打日志" —— 结果就是清理代码确实跑了、但因服务没响应啥也没删,
  // RViz 上上一趟卸的盒子还挂在原地, 且日志里完全看不出发生过这次失败(2026-08-13 实跑,
  // 卸 1 号盘的两个盒残留到卸 0 号盘还在场景里, 挡直下段).
  // 改成直接删"我们自己记的 id" —— 加盒时 push 进 ground_placed_ids_(见 addPlacedBox
  // 调用处), 这里直接对着这份自知清单发 REMOVE, 不需要先问 move_group "现在有哪些".
  // REMOVE 的通道用 apply_planning_scene 同步服务(与 removePlacedViaService 同一套):
  // PSI/话题直发都是异步、生效时机不定, 只有这个服务的返回才是"删没删掉"的权威判据.
  bool removeGroundBoxesViaService()
  {
    if (ground_placed_ids_.empty()) return true;
    if (!apply_scene_cli_->wait_for_service(2s)) {
      RCLCPP_WARN(logger_, "清上一趟卸货点残留: apply_planning_scene 服务不可用, 留着下次再试");
      return false;
    }
    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    req->scene.is_diff = true;
    req->scene.robot_state.is_diff = true;
    for (const auto & id : ground_placed_ids_) {
      moveit_msgs::msg::CollisionObject co;
      co.id = id;
      co.header.frame_id = base_frame_;
      co.operation = co.REMOVE;
      req->scene.world.collision_objects.push_back(co);
    }
    auto fut = apply_scene_cli_->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready) {
      RCLCPP_WARN(logger_, "清上一趟卸货点残留: apply_planning_scene 无响应, 留着下次再试");
      return false;
    }
    RCLCPP_INFO(logger_, "清掉上一趟卸货点残留 %zu 个碰撞体", ground_placed_ids_.size());
    ground_placed_ids_.clear();
    return true;
  }

  // 临时移出某托盘的所有已落盒 (放同盘下一层时它们正是落点, 不能当障碍挡住直下段).
  //
  // ⚠️ 必须走 apply_planning_scene 同步等回执, 不能用 psi_->removeCollisionObjects():
  // 后者只往话题发一条 diff 就返回, 场景何时真的更新不确定. 调用方紧接着就
  // computeCartesianPath, 盒可能还在场景里 -> 起点判在碰撞中 -> 覆盖 0%.
  // 2026-07-31 实跑证据 (卸货, 手上盒底低于同盘下层盒顶, 两次几何相同):
  //   0 号盘: 吸取->爬升 隔 1.77s, 异步移出来得及生效, 覆盖 100% (侥幸赢了竞态)
  //   1 号盘: 只隔 54ms, 移出还没到, 覆盖 0% -> 臂没爬, 贴着托盘规划 transit -> 擦到别盘
  // 时序决定成败正是竞态的特征, 所以"上一轮过了"不代表安全.
  void hideTrayBoxes(int t)
  {
    std::vector<std::string> ids;
    {
      std::lock_guard<std::mutex> lk(stack_mtx_);
      if (t < 0 || t >= static_cast<int>(placed_ids_.size()) || placed_ids_[t].empty()) return;
      ids = placed_ids_[t];
    }
    if (!removeObjectsSync(ids)) {
      // 退回异步 (服务不可用时总比什么都不做好), 并明确告警: 此时覆盖 0% 就是这个原因.
      psi_->removeCollisionObjects(ids);
      RCLCPP_WARN(logger_, "移出 %d 号托盘已落盒: 同步 REMOVE 失败, 已退回异步 (后续笛卡尔"
                  "覆盖可能因场景未更新而偏低)", t);
      return;
    }
    RCLCPP_INFO(logger_, "临时移出 %d 号托盘 %zu 个已落盒 (本层落点, 同步已确认)",
                t, ids.size());
  }

  // 同步删一批碰撞体: 发 apply_planning_scene 并等回执, 返回时场景已更新.
  bool removeObjectsSync(const std::vector<std::string> & ids)
  {
    if (ids.empty()) return true;
    if (!apply_scene_cli_->wait_for_service(2s)) return false;
    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    req->scene.is_diff = true;
    req->scene.robot_state.is_diff = true;
    for (const auto & id : ids) {
      moveit_msgs::msg::CollisionObject co;
      co.id = id;
      co.header.frame_id = base_frame_;
      co.operation = co.REMOVE;
      req->scene.world.collision_objects.push_back(co);
    }
    auto fut = apply_scene_cli_->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready) return false;
    return fut.get()->success;
  }

  // 把之前 hide 的托盘已落盒原样加回 (放置完直下段结束后调).
  void showTrayBoxes(int t)
  {
    std::lock_guard<std::mutex> lk(stack_mtx_);
    if (t < 0 || t >= static_cast<int>(placed_ids_.size())) return;
    for (size_t i = 0; i < placed_ids_[t].size(); ++i) {
      moveit_msgs::msg::CollisionObject co;
      co.id = placed_ids_[t][i];
      co.header.frame_id = base_frame_;
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = prim.BOX;
      prim.dimensions = {box_size_x_, box_size_y_, box_size_z_};
      co.primitives.push_back(prim);
      co.primitive_poses.push_back(placed_poses_[t][i]);
      co.operation = co.ADD;
      psi_->applyCollisionObjects({co});
    }
  }

  // /grasp/unload: 从托盘取盒(此刻 object_pose 报的就是托盘上的盒) -> 放到目的地(参数).
  void onUnload(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    RCLCPP_INFO(logger_, "==== /grasp/unload: 从托盘取盒 -> 放目的地(%.2f,%.2f,%.2f) ====",
                place_x_, place_y_, place_z_);

    std::string err;
    // 卸货取盒: 盒本在托盘里, attach 后与 Link_11 重叠, 必须允许盒碰托盘才能抬出
    if (!pickCycle(err, true)) { res->success = false; res->message = err; return; }

    if (!placeAt(place_x_, place_y_, place_z_, place_clearance_)) {
      res->success = false; res->message = "放目的地失败(已吸取)"; return;
    }
    if (!moveToReady()) {
      res->success = false; res->message = "卸货后回 ready 失败"; return;
    }

    RCLCPP_INFO(logger_, "==== 卸货一轮完成 ====");
    res->success = true; res->message = "unload cycle done";
  }

  // /grasp/unload_tray: 把一个托盘上的盒全部卸到地面, 两个卸货点左右平铺.
  //
  // 全程不用视觉: 取盒目标就是当初放它时的吸盘位姿 (placed_release_ 里存着), 放盒是它的
  // 逆操作. 放得进去就取得出来, 取放走完全对称的同一套几何, 不引入新的可达性风险.
  // 这也绕开了感知侧 pick_z_max 的限制 —— 托盘上的盒高于该阈值会被检测端整批滤掉,
  // 走视觉重新识别根本拿不到目标.
  //
  // 顺序: 后进先出, 先取栈顶. 摞的时候后放的在上面, 从下面抽会带翻上层.
  // 卸货点分配: 用**全局**的 unload_seq_ 计数器取点 (0 -> base x 负向, 1 -> 正向),
  // 每卸一个推进一格模 num_unload_points_. 两点由 unload_base_x/y ± unload_x_offset 定,
  // 释放高度 = unload_z + 本盒厚度.
  // ⚠️ 用持久计数器而不是循环下标 k: single=true 每次调用只卸一个, 若还用 k 则两次单卸
  // 的 k 都是 0, 第二个盒会落到第一个盒身上。计数器跨调用保持, 到 reset_stack 才归零。
  // ⚠️ 计数器全局共用、不按托盘分: 卸货点是地面上的物理位置, 与盒来自哪个托盘无关 ——
  // per-tray 时 L1 卸一个和 L2 卸一个会双双落到同一点 (2026-08-03 改).
  // 整盘模式在计数器为 0 时的落点序列与原先按 k 完全一致。
  //
  // single=true: 只卸栈顶一个就返回, 且不回 ready (交给调用方决定去哪). 遥控 L1/L2 走这条,
  // 一按一个便于人跟着取盒; mm_task 状态机走 single=false 整盘卸, 语义不变。
  bool unloadTray(int tray, std::string & err, bool single = false)
  {
    if (tray < 0 || tray >= num_trays_) { err = "托盘号越界"; return false; }
    const int n = trayLayers(tray);
    if (n <= 0) { err = std::to_string(tray) + " 号托盘是空的, 无盒可卸"; return false; }
    const int rounds = single ? 1 : n;

    RCLCPP_INFO(logger_,
                "==== 卸货(%s): %d 号托盘 %d 个盒, 本次卸 %d 个 -> 卸货基准 (%.4f,%.3f) 沿 base x ±%.1fmm, 地面 z=%.3f ====",
                single ? "单个" : "整盘", tray, n, rounds,
                unload_base_x_, unload_base_y_, unload_x_offset_ * 1000, unload_z_);

    // 卸货点碰撞体用 base_link 系记录 (机身系), 车挪到下一个卸货位后上一趟登记的
    // placed_unloaded_* 相当于"跟着机身瞬移"到了新位置附近, 变成幽灵碰撞体挡住这里的
    // 直下段 (2026-08-13 实跑: RViz 里 place1 卸的两个盒一直没消失, 疑似此前卡在覆盖
    // 93-96% 的放置失败与其有关, 未 100% 确认因果, 但清掉总没有坏处). 每次进 unloadTray
    // 先清一遍上一趟的残留 —— 此刻本盘还没放任何新盒, ground_placed_ids_ 里的必然都是
    // "上一趟"的, 清了不会误删本趟的. single 模式本来就不登记(见下), 不需要清.
    if (!single) removeGroundBoxesViaService();

    for (int i = 0; i < rounds; ++i) {
      // seq 单调递增(到 reset_stack 归零), 落点 k = seq % 点数. 碰撞体 id 用 seq 而不是 k:
      // k 会绕回, 复用 id 等于把上一个盒的碰撞体挪走 —— 地上那个盒就此脱管, 下次规划不避它。
      const int seq = unload_seq_;
      const int k = seq % num_unload_points_;
      // ---- 取盒: 回到当初放它的吸盘位姿 ----
      geometry_msgs::msg::Pose release;
      double thickness = 0.0;
      {   // 只读栈顶, 先不 pop —— 取失败时状态不能丢 (盒还在托盘上)
        std::lock_guard<std::mutex> lk(stack_mtx_);
        if (placed_release_[tray].empty()) { err = "堆叠状态与层数不一致"; return false; }
        release = placed_release_[tray].back();
        thickness = placed_th_[tray].back();
      }
      // 下压量按类别取 (负值多压): 释放位姿不等于盒落定后的高度, 差多少是这个类别盒子
      // 自己的性质 (盒面软硬/落进托盘沉多少), 与它摞在第几层无关.
      const double shortfall = unloadShortfallFor(thickness);
      release.position.z += shortfall;

      const std::string what = std::to_string(tray) + "号托盘第" +
                               std::to_string(trayLayers(tray)) + "层";
      RCLCPP_INFO(logger_, "取盒(%s): 厚 %.0fmm, 下压量 %+.0fmm -> 目标 z=%.3f",
                  what.c_str(), thickness * 1000, shortfall * 1000, release.position.z);
      if (!pickFromTray(release, thickness, what.c_str(), tray, true)) {
        err = "取盒失败: " + what; return false;
      }

      // ---- 取成功才更新堆叠状态 (盒已在吸盘上, 不在托盘上了) ----
      geometry_msgs::msg::Pose popped;
      double popped_th = 0.0;
      if (!popLayer(tray, popped, popped_th)) { err = "popLayer 失败"; return false; }

      // ---- 放到卸货点: 按 slot 计数器取点, 沿 base x 轴左右分开 ----
      const double sign = (k % 2 == 0) ? -1.0 : 1.0;
      geometry_msgs::msg::Pose dest;
      dest.position.x = unload_base_x_ + sign * unload_x_offset_;
      dest.position.y = unload_base_y_;
      // 释放高度按本盒厚度现算 (吸盘吸着盒顶面, 盒底落地): 同盘两盒厚度可能不同.
      dest.position.z = unload_z_ + thickness;
      // 朝向用实标卸货点的朝向, **不**沿用取盒时的托盘标定朝向: 托盘朝向 yaw≈-175°
      // (tray_q*, 为对齐托盘围栏而左旋过), 卸货点实标朝向 yaw≈-90°, 差 85° —— 沿用托盘
      // 朝向盒子会歪着落在地上 (2026-07-30 实跑观察). yaw -90° 也正是 look 位的 TCP 朝向
      // (look = ready + J1 +90°, ready yaw -180°), 与实标读值 -90.02° 吻合.
      dest.orientation = unload_quat_;

      RCLCPP_INFO(logger_, "卸货: 第 %d 个盒 -> 卸货点 %d (%.3f,%.3f,%.3f) 厚 %.0fmm yaw %.1f°",
                  k + 1, k, dest.position.x, dest.position.y, dest.position.z,
                  thickness * 1000, quatYaw(dest.orientation) * 180.0 / M_PI);
      if (!placeAtPose(dest, ("卸货点" + std::to_string(k)).c_str(), tray, true)) {
        if (dry_run_) detachBox();   // 失败也要清掉虚拟盒, 否则残留在吸盘上
        err = "放到卸货点失败(已吸取): 卸货点" + std::to_string(k);
        return false;
      }
      // dry_run 下取盒段挂的虚拟盒到此清掉, 否则它会一直挂在吸盘上进下一轮.
      if (dry_run_) detachBox();
      // 先竖直退开再登记碰撞体: 释放那刻吸盘末端正贴在盒顶上 (释放 z = 盒顶 z), 原地登记
      // 出来的碰撞体与 suction_link 立刻接触 -> 后续每次 plan 都判"起点在碰撞中"直接作废
      // (2026-07-30 实跑: 取第二个盒时 transit 规划 "Invalid states at index [0 1]",
      //  接触对 placed_unloaded_1_0 <-> suction_link). 几何本就是贴着的, 停位误差几 mm
      // 就决定判不判碰撞, 所以早几轮侥幸过了不代表安全.
      if (!dry_run_ && !moveRelativeXYZ(0.0, 0.0, lift_height_)) {
        err = "卸货后竖直退开失败: 卸货点" + std::to_string(k);
        return false;
      }
      // 卸到地面的盒留个碰撞体防下一个盒放到它身上 (不进堆叠, 那是托盘的账).
      // id 必须带 placed_ 前缀: reset_stack 的同步 REMOVE 与其复查都只认这个前缀,
      // 叫别的名字就成了连重启都清不掉的幽灵碰撞体 (2026-07-29 事故), 而卸货点就在
      // look 位附近地面, 残留一个正好挡住下一轮直下段.
      //
      // single 模式不登记 (2026-08-03): 遥控 L1/L2 一按卸一个, 人当场把盒取走, 场景里
      // 那个碰撞体立刻变成地面上的幽灵 —— 位置恰在 look 位正前方, 下一次 ✕ 抓取的直下段
      // 从它头上过, 规划直接判碰撞. 整盘模式(mm_task 自动跑)没人取盒, 盒确实还在地上,
      // 那里必须登记.
      if (!single) {
        const std::string ground_id =
          "placed_unloaded_" + std::to_string(tray) + "_" + std::to_string(seq);
        addPlacedBox(ground_id, boxCenterFromRelease(dest, thickness));
        ground_placed_ids_.push_back(ground_id);
      }
      // 这一个确实落地了才推进 slot: 上面任一步失败都 return, 计数器不动, 重试仍用同一点.
      unload_seq_ += 1;
    }

    // single 下不回 ready: 调用方(遥控 L1/L2)紧接着自己去 look, 回 ready 只是白走一趟.
    if (!single && !moveToReady()) { err = "卸货后回 ready 失败"; return false; }
    RCLCPP_INFO(logger_, "==== %d 号托盘卸了 %d 个, 余 %d 层 ====",
                tray, rounds, trayLayers(tray));
    return true;
  }

  // transit 高度候选, 从低到高试, 取第一个"transit 规划 + 直下段"两段都通的.
  // 装货与卸货各一组 (目的地不同: 托盘里 vs 地面), 但机制相同.
  // 2026-07-31 装货也改成多档 (原先只有 transit_z_ 一档 0.26): 当初写死高值是因为起点贴地、
  // 只有终点在高空, 规划器有自由先横移再爬, 低档会低空掠过别盘那摞盒. 现在爬升先把臂拉到
  // transit 高度、两端同高, 那个理由不再成立. 试档全是纯规划不动臂, 通不过自动退高档.
  // 空列表兜底回 transit_z_, 免得参数没配时候选为空直接失败.
  std::vector<double> transitCandidates(bool unload_mode) const
  {
    const auto & c = unload_mode ? unload_transit_candidates_ : transit_candidates_;
    if (c.empty()) return {transit_z_};
    return c;
  }

  // 取盒下压量: 按类别取. 类别由厚度反查 fallback_thickness_ —— 四个类别的标定厚度互不
  // 相同 (22/24/12/15mm), 反查是唯一的, 于是不必在堆叠里再存一份类别字段 (存了还要管
  // seed_placed 那条路径怎么填). 查不到就退回第一个值.
  double unloadShortfallFor(double thickness) const
  {
    for (size_t i = 0; i < fallback_thickness_.size(); ++i) {
      if (std::abs(fallback_thickness_[i] - thickness) < 1e-6 &&
          i < unload_pick_shortfall_.size()) {
        return unload_pick_shortfall_[i];
      }
    }
    return unload_pick_shortfall_.empty() ? 0.0 : unload_pick_shortfall_.front();
  }

  // 从吸盘释放位姿推盒心位姿 (工具朝下, 盒心在吸盘末端下方半个盒高).
  geometry_msgs::msg::Pose boxCenterFromRelease(const geometry_msgs::msg::Pose & release,
                                                double thickness) const
  {
    geometry_msgs::msg::Pose box = release;
    box.position.z -= thickness / 2.0;
    return box;
  }

  // 从托盘取一个盒: 规划到该盒释放位姿正上方 transit_z_ -> 笛卡尔垂直直下到释放位姿 ->
  // 吸 -> attach. 与 placeAtPose 严格对称 (它是: 抬起 -> transit -> 直下 -> 释放),
  // 故放得进去就取得出来.
  bool pickFromTray(const geometry_msgs::msg::Pose & release, double thickness,
                    const char * what, int tray, bool unload_mode = false)
  {
    const double v = plan_velocity_scaling_ * place_velocity_scaling_;
    move_group_->setMaxVelocityScalingFactor(v);
    move_group_->setMaxAccelerationScalingFactor(v);

    // 本盘已落盒全程移出: 它们正是要取的目标 (以及同盘下层), 当障碍会挡住 transit 与直下.
    // 别盘的留着当障碍, 防规划器横扫 (2026-07-28 蹭飞事故).
    hideTrayBoxes(tray);

    // 规划到正上方高空 transit, 高度逐级试 (同 placeAtPose, 理由见那边注释): 两段都验过
    // 才执行, 免得执行完 transit 才发现直下段不行. 取盒这头执行前臂上还没盒子, 卡住的
    // 后果比放置那头轻, 但同一套逻辑更好维护.
    // 重算失败退下一档重来 (同 placeAtPose, 理由见那边): 试算起点是 transit 规划终点的理论
    // 关节值, 执行后有停位误差, 重算可能掉档 —— 旧代码此时直接失败, 而更高档次根本没试过.
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::msg::RobotTrajectory dtraj;
    geometry_msgs::msg::Pose transit = release;
    std::vector<geometry_msgs::msg::Pose> dwps{release};
    const auto cands = transitCandidates(unload_mode);
    bool descended = false;
    double used_z = 0.0;

    for (size_t ci = 0; ci < cands.size() && !descended; ++ci) {
      const double z = cands[ci];
      transit.position.z = z;
      move_group_->setStartStateToCurrentState();
      move_group_->setPoseTarget(transit);
      if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(logger_, "取盒(%s): transit z=%.3f 规划失败, 试下一档", what, z);
        continue;
      }
      const auto cur_state = move_group_->getCurrentState();
      if (!cur_state) { showTrayBoxes(tray); restorePlanScaling(); return false; }
      moveit::core::RobotState start(*cur_state);
      const auto & jt = plan.trajectory_.joint_trajectory;
      start.setVariablePositions(jt.joint_names, jt.points.back().positions);
      move_group_->setStartState(start);
      const double frac = move_group_->computeCartesianPath(dwps, 0.005, 0.0, dtraj);
      // 阈值 0.99 而非 0.9: 卡在 0.9 上的档次实测必破, 早点让给下一档.
      if (frac < 0.99) {
        RCLCPP_WARN(logger_, "取盒(%s): transit z=%.3f 的直下段试算只覆盖 %.0f%% (<99%%), 试下一档",
                    what, z, frac * 100);
        continue;
      }
      RCLCPP_INFO(logger_,
                  "取盒(%s): transit z=%.3f 两段试算全通 (transit %zu 点, 直下 %.0f%%)",
                  what, z, plan.trajectory_.joint_trajectory.points.size(), frac * 100);

      if (dry_run_) {
        RCLCPP_WARN(logger_, "取盒(%s): [dry_run] 目标 z=%.3f, 到此为止", what,
                    release.position.z);
        // 挂虚拟盒 (同 onExecute/place_only 的 dry_run): 没有它后面放到卸货点那段是按
        // "吸盘上空着"规划的, 验不到盒子几何在目的地会不会撞. 由 unloadTray 走完一轮 detach.
        attachBox(true, thickness);
        showTrayBoxes(tray); restorePlanScaling(); return true;
      }
      if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(logger_, "取盒(%s): transit z=%.3f 执行失败, 试下一档", what, z);
        continue;
      }
      settle();
      // 按实测起点重算 (理论轨迹会被 start-state 校验拒掉).
      move_group_->setStartStateToCurrentState();
      const double frac2 = move_group_->computeCartesianPath(dwps, 0.005, 0.0, dtraj);
      if (frac2 < 0.9) {
        RCLCPP_WARN(logger_,
                    "取盒(%s): transit z=%.3f 直下段按实测起点重算只覆盖 %.0f%% (试算 %.0f%%), 试下一档",
                    what, z, frac2 * 100, frac * 100);
        continue;   // 臂停在本档 transit 点, 吸盘上还没盒子, 退档安全
      }
      if (move_group_->execute(dtraj) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(logger_, "取盒(%s): transit z=%.3f 直下段执行失败, 试下一档", what, z);
        continue;
      }
      settle();
      descended = true; used_z = z;
    }
    if (!descended) {
      RCLCPP_ERROR(logger_, "取盒(%s): 所有 transit 高度都没能走完两段 (%.3f,%.3f)", what,
                   release.position.x, release.position.y);
      showTrayBoxes(tray); restorePlanScaling(); return false;
    }
    RCLCPP_INFO(logger_, "取盒(%s): 已用 transit z=%.3f 走完两段", what, used_z);

    // 吸取 + 转保压 (与 stageInsert 同一套: 抽真空建立负压, 再 STOP 关泵关阀维持).
    publishPump(PUMP_SUCK);
    RCLCPP_INFO(logger_, "取盒(%s): 已到位, 发 /pump_cmd 1 抽真空 %.1fs", what, suck_duration_);
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(suck_duration_ * 1000)));
    publishPump(PUMP_STOP);
    // 盒在托盘里与 Link_11 几何重叠, 必须豁免与托盘接触, 否则连抬起都规划不了.
    attachBox(true, thickness);
    showTrayBoxes(tray);   // 本盘其余盒加回当障碍 (这个盒已 popLayer 出栈, 不会重复)
    restorePlanScaling();
    RCLCPP_INFO(logger_, "取盒(%s): 已吸取并转保压", what);
    return true;
  }

  // 抓取周期(三段): 取 object_pose -> ①粗定位 ②精修 ③末段直插吸取. 抓取与卸货共用.
  // 分工: ①粗定位开环把相机开到盒上方; ②精修闭环只对水平(xy+yaw), 高度保持不动;
  //       ③开环从当前高度直插到盒顶.
  // ②只管水平, 是因为两件事各有各的可信来源:
  //   - 水平必须闭环. 手眼标定误差(机械安装误差 + 增量式零位与 URDF 零位不一致)让检测
  //     坐标整体偏 2~3cm, 且是开环系统偏差, 补不掉; 闭环实时看盒与吸盘的相对关系才能约掉.
  //   - 高度不需要视觉. 下插行程由 TF 实测 TCP 高度算, 是机械臂自己的量, 不过手眼标定.
  // 附带好处: 精修全程停在 pregrasp 高度, 相机不贴近盒子, ROI 里始终有台面像素 ->
  //   避开近距离深度基准崩掉 (实测近距离同一位置两次报 -0.035 / +0.135, 差 170mm).
  bool pickCycle(std::string & err, bool allow_tray_touch)
  {
    geometry_msgs::msg::PoseStamped obj;
    if (!waitObject(obj, 3.0)) { err = "无 object_pose"; return false; }
    if (!stageCoarse(obj)) { err = "① 粗定位失败"; return false; }
    if (!stageRefine())    { err = "② 精修失败"; return false; }
    // ②之后再转 yaw (吸取前): 吸盘轴先与盒轴对齐, 吸起来盒在吸盘上的相对朝向就是 0,
    // 放置时只需把吸盘转到托盘标定朝向, 不用再算"盒相对吸盘"那一层.
    if (!stageAlignYaw()) { err = "转 yaw 对齐失败"; return false; }
    // 调试开关: 跑完 ②+对齐 yaw 后停住, 不下插. 用 ros2 param set /grasp_node
    // pick_skip_insert true 触发, 配合 /grasp/pick_only 验精修对准效果.
    const bool skip_insert = getOrDeclare<bool>("pick_skip_insert", false);
    if (skip_insert) {
      RCLCPP_WARN(logger_, "pick_skip_insert=true: ②+对齐 yaw 后停住, 跳过 ③下插");
      return true;
    }
    if (!stageInsert(allow_tray_touch)) { err = "③ 末段直插失败"; return false; }
    return true;
  }

  // ① 粗定位: 规划到盒上方 pregrasp_height, 姿态固定为 coarse_yaw_ (吸盘朝下), 不跟盒子转.
  //
  // 为什么①不对齐盒子 yaw: 吸盘是圆的气吸, 绕自身轴转任何角度吸力一样, 对齐 yaw 对"吸"
  // 零收益(对齐是夹爪的需求). 而相机装在腕上, ①一转 yaw 相机跟着转 —— 实测①转到 -177°
  // 时相机中心比吸盘偏 55mm, 盒子被推到画面右下角裁掉一半, 模型认不出, object_pose 直接
  // 断流. 更根本的是 cam_target 是在某个腕姿态下标的, ①一换姿态它就不成立.
  // 固定成 coarse_yaw_ 后: 相机朝向从 look 到①到② 全程不变, cam_target 天然成立,
  // ②的实测雅可比也不随构型漂.
  //
  // 放置需要盒子摆正, 那个 yaw 挪到②之后单独转 (见 stageRotateYaw): 先对准再转, 吸取瞬间
  // 吸盘与盒子朝向已一致, 盒子在吸盘上的相对朝向是确定的; 若改成吸起来再转, 盒子是斜贴在
  // 吸盘上的, 放置时还要算"盒相对吸盘"的朝向, 多一层易错换算.
  bool stageCoarse(const geometry_msgs::msg::PoseStamped & obj)
  {
    geometry_msgs::msg::Pose target;
    target.position.x = obj.pose.position.x;
    target.position.y = obj.pose.position.y;
    target.position.z = obj.pose.position.z + pregrasp_height_;
    target.orientation = yawToQuat(coarse_yaw_);
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "① 规划失败 (吸盘 yaw 固定 %+.0f°)", coarse_yaw_ * 180.0 / M_PI);
      return false;
    }
    RCLCPP_INFO(logger_, "① 规划成功 (吸盘 yaw 固定 %+.0f°)", coarse_yaw_ * 180.0 / M_PI);
    RCLCPP_INFO(logger_, "① 粗定位执行到盒上方 %.0fcm", pregrasp_height_ * 100);
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
    settle();
    return true;
  }

  // ②之后: 原地转腕, 闭环把吸盘长轴转到与盒长轴平行, 为放置做准备
  // (放置要求盒子摆正; 吸本身不需要 yaw).
  //
  // 判据是"图像角落到 θ_img*", 不是"腕转到解算出的 ψ". 两者的区别是本段的全部要点:
  // 相机固连腕, 所以吸盘长轴在图像里的角度是个与构型无关的常数 θ_img*; 盒长轴与吸盘
  // 长轴平行 <=> θ_img == θ_img*. 于是对齐是个可以在机上直接看到的终点条件, 腕该转
  // 多少不需要事先知道.
  //
  // 为什么不再解算 ψ (2026-07-28 推翻了前一版): 前一版写 ψ = ψ_ref + s·k·(θ_ref − θ_img),
  // 一次开环解算就下发, 对不对全押在常斜率 k 上, 而实测斜率随角区在 0.5~0.9 间变
  // (腕绕 base z 转而光轴不与 base z 平行, 投影把关系压成随 ψ 变化的畸变). 单次开环解算
  // 偏 40% 就是几十度, 直接把腕甩过 (−90,90] 折叠边界. 改成闭环后 k 降级成"步长估计":
  // 每步转完重新采 θ_img, 偏了下一步自己修回来, 终点精度只由 θ_img* 与容差决定.
  // 斜率取 yaw_slope_prior_ 作先验直接下发第一步, 被单调性检查证伪才回退 measureYawSlope
  // 试探实测. 原先无条件先试探(与 ② 精修实测雅可比同一思路), 但那要多付"小动一下 + 两次
  // 排空采样"约 2s; 而闭环本就用来吸收先验偏差, 先验只需量级对.
  // 顺带消掉旧链最恶心的性质 —— 偏差随"离标定点角距离"放大, 使得在标定点附近试必然误判
  // 为已修好 (2026-07-27 因此踩了四轮).
  //
  // ⚠️ 2026-07-28 撤回两个错误诊断 (别再照它们改代码):
  //   ① "OBB 退化/吸盘遮挡致 θ_img 假值" —— 错. 走到 ±88° 时 conf 0.83、框位置深度全稳,
  //      那是真值, 只是腕真的被转到了那里.
  //   ② "投影斜率连符号都会翻" —— 也错. 实测 θ_img +41.5° 处斜率 +0.86、−39.8° 处 +0.48,
  //      同号. 真身是 rotateToolYawBy 的 90° 等价折叠把下发步长换成了反号的那个分支
  //      (请求 +60° 实走 −30°, 请求 −46.2° 实走 +43.8°), 见 yawEquivalents 注释. 该折叠
  //      已删 —— θ_img 判据是 180° 周期的, 与 90° 折叠根本不兼容.
  //
  // 位置目标沿用当前实测 TCP xyz, 只换朝向 —— 绕吸盘自身轴转, 名义上盒心位置不变.
  // 每步打印 TCP xy 漂移: 腕关节轴与吸盘轴未必严格共线, 漂多少用实测说话.
  bool stageAlignYaw()
  {
    geometry_msgs::msg::PoseStamped before;
    if (!currentTcp(before)) { RCLCPP_ERROR(logger_, "转 yaw: 取当前位姿失败"); return false; }

    move_group_->setMaxVelocityScalingFactor(rotate_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(rotate_velocity_scaling_);

    double turned = 0.0;        // 本段累计转动量(度), 防跑飞
    double theta = 0.0, err = 0.0;
    if (!sampleYawErr(theta, err)) { restorePlanScaling(); return false; }
    if (std::fabs(err) <= yaw_align_tol_deg_) {
      RCLCPP_INFO(logger_, "转 yaw: θ_img=%+.1f° 已在容差内(差 %+.1f°), 无需转",
                  theta, err);
      restorePlanScaling();
      return true;
    }

    // ---- 先按先验斜率直接下发, 只有它被证伪才回退去试探 ----
    // 原先每轮无条件先转 yaw_probe_deg_ 试探一步实测斜率, 代价是"小动一下 + 两次排空采样"
    // 约 2s 的顿挫. 而斜率实测四次(θ_img +41.5/+40.8/+25.9 处 +0.86/+0.84/+0.83,
    // -39.8° 处 +0.48)符号从未翻过, 量级也只在 0.5~0.9 间 —— 这个精度对闭环足够: 步长打
    // yaw_step_gain_ 折, 偏 20% 只是多迭代一步, 终点精度只由 θ_img* 与容差决定.
    // 先验偏太多(含符号猜反)由单调性检查兜住: 一步后误差没降就回退试探实测, 不再直接中止.
    double slope = yaw_slope_prior_;
    bool slope_measured = false;
    RCLCPP_INFO(logger_, "转 yaw: θ_img=%+.1f° (判据 %+.1f°, 差 %+.1f°), 先按先验斜率 %+.2f 走",
                theta, yaw_target_theta_img_, err, slope);

    bool aligned = false;
    double prev_err = std::fabs(err);
    for (int step = 0; step < yaw_align_max_steps_; ++step) {
      if (std::fabs(err) <= yaw_align_tol_deg_) {
        RCLCPP_INFO(logger_, "转 yaw: θ_img=%+.1f° 距判据 %+.1f° <= 容差 %.1f°, 对齐 (第%d步)",
                    theta, err, yaw_align_tol_deg_, step);
        aligned = true;
        break;
      }
      if (step > 0 && std::fabs(err) > prev_err * 0.8) {
        // 转完误差没降. 两种可能: 先验斜率不准(含符号猜反), 或斜率在这个角区真变了.
        // 先验还没被实测替换过 -> 花一次试探把它量准再继续; 已经是实测值还不收敛 -> 中止,
        // 再迭代只会越跑越远.
        if (slope_measured) {
          RCLCPP_ERROR(logger_, "转 yaw 第%d步: 误差 %.1f° -> %.1f° 未收敛(实测斜率已变), 中止",
                       step + 1, prev_err, std::fabs(err));
          restorePlanScaling(); return false;
        }
        RCLCPP_WARN(logger_, "转 yaw: 先验斜率 %+.2f 下误差 %.1f° -> %.1f° 没降, 转试探步实测",
                    slope, prev_err, std::fabs(err));
        if (!measureYawSlope(theta, err, turned, slope)) { restorePlanScaling(); return false; }
        slope_measured = true;
      }
      prev_err = std::fabs(err);
      double d_deg = clampAbs(-err / slope * yaw_step_gain_, yaw_max_step_deg_);
      if (std::fabs(turned) + std::fabs(d_deg) > yaw_align_max_turn_deg_) {
        RCLCPP_ERROR(logger_, "转 yaw: 累计转动将超上限 %.0f° (已转 %+.1f°, 还要 %+.1f°), 中止",
                     yaw_align_max_turn_deg_, turned, d_deg);
        restorePlanScaling(); return false;
      }
      RCLCPP_INFO(logger_, "转 yaw 第%d步: θ_img=%+.1f° (差 %+.1f°) / 斜率 %+.2f -> 腕转 %+.1f°",
                  step + 1, theta, err, slope, d_deg);
      if (!rotateToolYawBy(d_deg * M_PI / 180.0)) { restorePlanScaling(); return false; }
      turned += d_deg;
      if (!sampleYawErr(theta, err)) { restorePlanScaling(); return false; }
    }
    restorePlanScaling();
    if (!aligned) {
      RCLCPP_ERROR(logger_, "转 yaw: %d 步未对齐 (末次差 %+.1f°)", yaw_align_max_steps_, err);
      return false;
    }

    geometry_msgs::msg::PoseStamped after;
    if (!currentTcp(after)) return false;
    const double dx = after.pose.position.x - before.pose.position.x;
    const double dy = after.pose.position.y - before.pose.position.y;
    RCLCPP_INFO(logger_, "转 yaw 完成: 腕 %+.1f° -> %+.1f°, TCP xy 漂移 (%+.1f,%+.1f)mm |d|=%.1fmm",
                quatYaw(before.pose.orientation) * 180.0 / M_PI,
                quatYaw(after.pose.orientation) * 180.0 / M_PI,
                dx * 1000, dy * 1000, std::hypot(dx, dy) * 1000);
    return true;
  }

  // 转一小步 yaw_probe_deg_ 实测 dθ_img/dψ. 只在先验斜率被单调性检查证伪时才调 ——
  // 无条件先试探要多付"小动一下 + 两次排空采样"约 2s.
  // 就地更新 theta/err/turned 到试探后的实况, 调用方接着按 slope 继续迭代.
  bool measureYawSlope(double & theta, double & err, double & turned, double & slope)
  {
    // 试探方向按误差符号取: 先验若大致对, 这一步本身也是有效位移, 不白转.
    const double probe = (err > 0.0) ? -yaw_probe_deg_ : yaw_probe_deg_;
    RCLCPP_INFO(logger_, "转 yaw 试探: θ_img=%+.1f° (差 %+.1f°) -> 试转 %+.1f° 量斜率",
                theta, err, probe);
    if (!rotateToolYawBy(probe * M_PI / 180.0)) return false;
    turned += probe;

    double theta2 = 0.0, err2 = 0.0;
    if (!sampleYawErr(theta2, err2)) return false;
    // 斜率也要折叠作差: 长轴无向, θ_img 在 ±90° 边界会翻符号, 直接相减会得到 ~180° 的假变化.
    const double dtheta = wrapAngle90((theta2 - theta) * M_PI / 180.0) * 180.0 / M_PI;
    slope = dtheta / probe;
    RCLCPP_INFO(logger_, "转 yaw 试探结果: 腕转 %+.1f° -> θ_img %+.1f°→%+.1f° (变化 %+.1f°), 斜率 %+.2f",
                probe, theta, theta2, dtheta, slope);
    // 斜率太小 = 检测没跟随腕转(视野甩飞/盒子被挤出画/长短轴分不清). 拿它解步长会得到
    // 巨大转动. 中止是安全的: 此刻还没吸盒子, 臂停在 pregrasp 高度.
    if (std::fabs(slope) < yaw_slope_min_) {
      RCLCPP_ERROR(logger_, "转 yaw: 实测斜率 %.2f 低于下限 %.2f, 判检测不跟随腕转, 中止",
                   slope, yaw_slope_min_);
      return false;
    }
    theta = theta2; err = err2;
    return true;
  }

  // 采一次 θ_img 与它到判据的折叠差(度). 折到 (-90,90] 作差: 长轴无向, 差 180° 同朝向.
  // 坏观测门在此: |θ_img| 贴 ±90° 边界时**不一定**是坏值(2026-07-28 实测 -88° 是真值,
  // 检测健康), 但那是折叠边界, 后续做差/求斜率都可能翻符号, 单帧判不出真假 —— 故只在
  // 这里拦掉一次, 让上层中止而不是拿它算步长. 中止是安全的: 还没吸盒子, 臂在 pregrasp 高度.
  bool sampleYawErr(double & out_theta, double & out_err)
  {
    bool got = false;
    for (int i = 0; i < axis_resample_tries_ && !got; ++i) {
      rclcpp::sleep_for(std::chrono::milliseconds(
        static_cast<int>(axis_flush_sec_ * 1000)));   // 排空运动中拍的积压帧
      got = avgAxisAngle(out_theta);
    }
    if (!got) {
      RCLCPP_ERROR(logger_, "转 yaw: 取不到一致的 θ_img (重采 %d 次)", axis_resample_tries_);
      return false;
    }
    if (std::fabs(out_theta) > yaw_bad_theta_deg_) {
      RCLCPP_ERROR(logger_, "转 yaw: θ_img=%+.1f° 贴 ±90° 折叠边界, 符号不可判, 中止",
                   out_theta);
      return false;
    }
    out_err = wrapAngle90((out_theta - yaw_target_theta_img_) * M_PI / 180.0) * 180.0 / M_PI;
    return true;
  }

  // 在当前朝向上再绕 base z 补转 d 弧度, 位置不变. 等价分支(180° 折)就近优先:
  // 就近那个可能落在腕限位外或规划失败, 另一个留作兜底.
  bool rotateToolYawBy(double d)
  {
    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) return false;
    geometry_msgs::msg::Pose target = cur.pose;
    const auto want = rotateAboutBaseZ(cur.pose.orientation, d);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for (const auto & q : yawEquivalents(want)) {
      target.orientation = q;
      move_group_->setStartStateToCurrentState();
      move_group_->setPoseTarget(target);
      if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
        settle();
        return true;
      }
      RCLCPP_WARN(logger_, "转 yaw 到 %+.1f° 规划失败, 试等价分支", quatYaw(q) * 180.0 / M_PI);
    }
    RCLCPP_ERROR(logger_, "转 yaw: 所有等价分支都规划失败");
    return false;
  }

  // jog 专用直转: 绕 base 竖直轴转 d 弧度, 位置不变, 只试目标朝向本身(不走 yawEquivalents
  // 的 90°/180° 等价折叠) —— 标定时你要转多少就转多少, 不被折叠换成别的等价姿态.
  bool jogRotateYaw(double d)
  {
    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) return false;
    geometry_msgs::msg::Pose target = cur.pose;
    target.orientation = rotateAboutBaseZ(cur.pose.orientation, d);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target);
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "jog 转腕 %+.1f° 规划失败", d * 180.0 / M_PI);
      return false;
    }
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
    settle();
    return true;
  }

  geometry_msgs::msg::Quaternion yawToQuat(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
  }

  // 绕 base_link 竖直轴再补转 a: q = q_z(a) * q_target.
  geometry_msgs::msg::Quaternion rotateAboutBaseZ(
    const geometry_msgs::msg::Quaternion & t, double a)
  {
    const double cw = std::cos(a / 2.0), cz = std::sin(a / 2.0);
    geometry_msgs::msg::Quaternion q;
    q.w = cw * t.w - cz * t.z;
    q.x = cw * t.x - cz * t.y;
    q.y = cw * t.y + cz * t.x;
    q.z = cw * t.z + cz * t.w;
    return q;
  }

  // 目标姿态的等价分支(只按 180° 折), 按"离当前吸盘朝向近"排序返回.
  // 排序的意义是省掉无意义的转动 —— θ_img 折到 (-90°,90°], 盒摆在边界附近时检测值在
  // +88°/-88° 间反复翻, 不排序会为 176° 的名义差白转半圈.
  //
  // 2026-07-28 删掉近正方形盒的 90° 折叠分支: 它与 θ_img 判据根本不兼容. θ_img 折到
  // (-90,90], 即判据本身是 180° 周期的 —— 姿态转 90° 后 θ_img 读出的是 90° 误差而不是 0.
  // 于是 90° 折叠会把闭环解出的步长静默换成"最近的那个", 只要 |步长| > 45° 就换成一个
  // 反号的步长: 实测请求 +60° 实走 -30°(60-90), 请求 -46.2° 实走 +43.8°. 这正是早前
  // 误判成"投影斜率连符号都会翻"的真身 —— 斜率一直是 +0.8 上下, 是下发的步长被折反了.
  std::vector<geometry_msgs::msg::Quaternion> yawEquivalents(
    const geometry_msgs::msg::Quaternion & target)
  {
    std::vector<geometry_msgs::msg::Quaternion> out;
    out.reserve(2);
    for (int k = 0; k < 2; ++k) out.push_back(rotateAboutBaseZ(target, M_PI * k));

    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) return out;
    const double ycur = quatYaw(cur.pose.orientation);
    std::stable_sort(out.begin(), out.end(),
                     [&](const geometry_msgs::msg::Quaternion & a,
                         const geometry_msgs::msg::Quaternion & b) {
                       return std::fabs(wrapAngle(quatYaw(a) - ycur)) <
                              std::fabs(wrapAngle(quatYaw(b) - ycur));
                     });
    return out;
  }


  // ② 精修: 相机系闭环, 把盒心在相机系的横向 (y,z) 推到标定目标值 (cam_target_y/z).
  // 误差在相机系算, 不经手眼标定/TF, 所以臂移动不会让目标跳 (见 cam_point_topic_ 注释).
  // 高度不控, 臂全程停在①到达的 pregrasp 高度.
  //
  // 实现是"笛卡尔步进"而非连续 twist 伺服. 换掉 twist 的两个原因:
  //   1) 抖: 检测 ~11Hz 而伺服 30Hz, 中间夹一个 P 控制器, 三个节奏叠起来关节侧就是一顿一顿.
  //      低通只能把台阶磨圆, 治不了根. 步进把闭环频率降到检测频率, 每步是 MoveIt 时间参数化
  //      过的轨迹, 自带加减速.
  //   2) 盒子是静止的, 本来就不需要连续跟随; 步进还顺带绕开 servo 那一整套(奇异点阈值、
  //      平滑插件、use_gazebo 输出路径).
  //
  // 映射靠实测而非推导: 先试探性在 base x/y 各走 probe_step_, 量 p_cam 横向怎么变, 得到
  // 2x2 雅可比 J (d[cam_y,cam_z] / d[base_x,base_y]), 再解 dxy = -J^-1 * e 一步到位.
  // 之所以不用 TF 旋转算: 检测端 p_cam 走的是 R_mech_optical @ Rz(optical_roll_deg),
  // 该 roll(-90°) 把横向两轴对调了, 于是 p_cam 的横向轴与 Link_30 的 TF 轴差 ~90° ——
  // 实测指令方向与实际位移方向夹角 105° 就是这么来的. 那个 roll 是队友手眼标定的一部分,
  // 改它会同时破坏 base_link 检测坐标, 所以在本节点侧用实测把整条链路一次量掉:
  // 无论中间差多少次旋转/反射, 实测 J 都直接包含.
  bool stageRefine()
  {
    // 步进段整体降速, 收尾恢复.
    move_group_->setMaxVelocityScalingFactor(refine_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(refine_velocity_scaling_);
    const bool ok = refineStepwise();
    restorePlanScaling();
    if (ok) RCLCPP_INFO(logger_, "② 精修收敛 (相机系横向达标, 高度与朝向未动)");
    return ok;
  }

  bool refineStepwise()
  {
    const rclcpp::Time t0 = node_->now();

    double cy = 0.0, cz = 0.0;
    if (!avgCamPoint(cy, cz)) { RCLCPP_ERROR(logger_, "② 无新鲜 p_cam"); return false; }
    double e_start = std::hypot(cy - cam_target_y_, cz - cam_target_z_);
    RCLCPP_INFO(logger_, "② 起始相机系误差 %.1fmm", e_start * 1000);
    if (e_start < cam_tol_) {
      RCLCPP_INFO(logger_, "② 已在容差内, 无需精修"); return true;
    }

    // ---- 试探量雅可比 ----
    // 沿 base +x 走一步, 看 p_cam 横向变化 -> J 第一列; 再沿 +y 走一步 -> J 第二列.
    // 每次试探后不回退: 试探本身也是有效位移, 回退纯属浪费行程(且多一次起停).
    double j[2][2];
    for (int axis = 0; axis < 2; ++axis) {
      const double dx = (axis == 0) ? probe_step_ : 0.0;
      const double dy = (axis == 0) ? 0.0 : probe_step_;
      if (!moveRelativeXY(dx, dy)) {
        RCLCPP_ERROR(logger_, "② 试探移动失败 (axis=%d)", axis); return false;
      }
      double ny = 0.0, nz = 0.0;
      if (!avgCamPoint(ny, nz)) { RCLCPP_ERROR(logger_, "② 试探后无 p_cam"); return false; }
      j[0][axis] = (ny - cy) / probe_step_;
      j[1][axis] = (nz - cz) / probe_step_;
      RCLCPP_INFO(logger_, "② 试探 base %s +%.0fmm -> cam (y,z) 变化 (%+.1f,%+.1f)mm",
                  axis == 0 ? "x" : "y", probe_step_ * 1000,
                  (ny - cy) * 1000, (nz - cz) * 1000);
      cy = ny; cz = nz;
    }

    const double det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
    // det 接近 0 = 两次试探引起的 cam 变化几乎共线, 解不出唯一位移(病态). 多半是某个方向
    // 臂没真动(限位/规划失败被忽略)或检测噪声盖过了试探量. 此时硬解会得到巨大位移, 危险.
    if (std::fabs(det) < 0.05) {
      RCLCPP_ERROR(logger_, "② 雅可比病态 det=%.4f, 停止 (试探位移可能未生效)", det);
      return false;
    }
    RCLCPP_INFO(logger_, "② 实测雅可比 [[%.2f,%.2f],[%.2f,%.2f]] det=%.3f",
                j[0][0], j[0][1], j[1][0], j[1][1], det);

    // ---- 迭代收敛 ----
    for (int step = 0; step < refine_max_steps_; ++step) {
      if ((node_->now() - t0).seconds() > refine_timeout_) {
        RCLCPP_ERROR(logger_, "② 精修超时"); return false;
      }
      const double ey = cy - cam_target_y_;
      const double ez = cz - cam_target_z_;
      const double err = std::hypot(ey, ez);
      if (err < cam_tol_) {
        RCLCPP_INFO(logger_, "② 第%d步后误差 %.1fmm < 容差 %.1fmm, 收敛",
                    step, err * 1000, cam_tol_ * 1000);
        return true;
      }
      // 解 J * dxy = -e, 消掉误差.
      double dx = (-ey * j[1][1] + ez * j[0][1]) / det;
      double dy = (-ez * j[0][0] + ey * j[1][0]) / det;
      dx = clampAbs(dx * refine_step_gain_, refine_max_step_);
      dy = clampAbs(dy * refine_step_gain_, refine_max_step_);
      RCLCPP_INFO(logger_, "② 第%d步: 误差 (%+.1f,%+.1f)mm |e|=%.1fmm -> 走 base (%+.1f,%+.1f)mm",
                  step + 1, ey * 1000, ez * 1000, err * 1000, dx * 1000, dy * 1000);
      if (!moveRelativeXY(dx, dy)) { RCLCPP_ERROR(logger_, "② 步进移动失败"); return false; }
      if (!avgCamPoint(cy, cz)) { RCLCPP_ERROR(logger_, "② 步进后无 p_cam"); return false; }
      const double now_err = std::hypot(cy - cam_target_y_, cz - cam_target_z_);
      if (now_err > e_start * refine_diverge_ratio_ && now_err > cam_tol_ * 3.0) {
        RCLCPP_ERROR(logger_, "② 误差发散: 起始 %.1fmm -> 当前 %.1fmm, 停止",
                     e_start * 1000, now_err * 1000);
        return false;
      }
    }
    RCLCPP_ERROR(logger_, "② %d 步未收敛", refine_max_steps_);
    return false;
  }

  // 相对当前 TCP 在 base_link 水平面移动 (dx,dy), 姿态与高度不变. 笛卡尔路径, 自带加减速.
  bool moveRelativeXY(double dx, double dy) { return moveRelativeXYZ(dx, dy, 0.0); }

  // 相对当前 TCP 走一步纯笛卡尔直线 (base_link 系位移 dx,dy,dz). 朝向不变.
  bool moveRelativeXYZ(double dx, double dy, double dz)
  {
    if (std::sqrt(dx * dx + dy * dy + dz * dz) < 1e-4) return true;
    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) return false;
    geometry_msgs::msg::Pose wp = cur.pose;
    wp.position.x += dx;
    wp.position.y += dy;
    wp.position.z += dz;
    std::vector<geometry_msgs::msg::Pose> wps{wp};
    moveit_msgs::msg::RobotTrajectory traj;
    move_group_->setStartStateToCurrentState();
    const double frac = move_group_->computeCartesianPath(wps, 0.002, 0.0, traj);
    if (frac < 0.9) {
      RCLCPP_ERROR(logger_, "笛卡尔步进路径覆盖不足 %.0f%%", frac * 100);
      return false;
    }
    if (move_group_->execute(traj) != moveit::core::MoveItErrorCode::SUCCESS) return false;
    settle();
    return true;
  }

  // 取 cam_avg_frames_ 帧新鲜 p_cam 的横向均值, 压掉单帧 1~2mm 噪声.
  // 只收 stamp 比上一帧新的, 避免同一帧被重复计入(检测 ~11Hz, 轮询快得多).
  bool avgCamPoint(double & out_y, double & out_z)
  {
    double sy = 0.0, sz = 0.0;
    int n = 0;
    rclcpp::Time last(0, 0, RCL_ROS_TIME);
    const rclcpp::Time t0 = node_->now();
    rclcpp::WallRate r(60.0);
    while (rclcpp::ok() && n < cam_avg_frames_) {
      if ((node_->now() - t0).seconds() > cam_wait_timeout_) return false;
      geometry_msgs::msg::PointStamped cp;
      if (latestCamPoint(cp)) {
        const rclcpp::Time st(cp.header.stamp);
        if (st > last) {
          last = st;
          sy += cp.point.y;
          sz += cp.point.z;
          ++n;
        }
      }
      r.sleep();
    }
    out_y = sy / n;
    out_z = sz / n;
    return true;
  }

  // ③ 末段: 取当前 suction_tip 位姿, 沿其 -Z 相对下插, 再吸.
  //    严禁用盒子绝对坐标 setPoseTarget —— waypoint 只由当前实测位姿 + -Z 偏置得来.
  //    行程不用固定值, 改由实测算: TCP 实测高度 - 锁定盒顶高度. 手眼标定有 2~3cm 误差,
  //    盲插固定行程会连带把这份误差插进台面; 用 TF 实测的 TCP 高度做基准,
  //    锁定值偏几 mm 也只影响落点几 mm. 夹在 [min,max] 内兜住锁定值本身出错的情况.
  bool stageInsert(bool allow_tray_touch)
  {
    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) { RCLCPP_ERROR(logger_, "③ 取当前位姿失败"); return false; }

    // 盒顶高度纯几何算: 地面(URDF 常量) + 该类别卡尺厚度. 不用视觉测高.
    const int cat = currentCategory();
    const int ci = categoryIndex(cat);
    const double thickness = currentThickness(ci);
    const double box_top_z = ground_z_ + thickness;

    // 把类别锁存成"手上这个盒的类别": 下插是确定抓哪个盒的最后时刻, 此刻的 last_category_
    // 必与目标同盒 (yolo 侧用 p_cam 帧间最近邻锁目标, 四路话题指向同一个盒).
    // 2026-08-03 实机: 桌上摆一个左盘盒 + 一个右盘盒, 概率性放错盘。根因是遥控流程
    // ✕抓取 -> level -> (人按)■放置 中, resolvePlaceTarget 读的是**按 ■ 那一刻**的
    // last_category_ —— 盒已在吸盘上, yolo 仍在跑, 视野里只剩另一个盒, 缓存早被刷成它的
    // 类别了。自动流程 /grasp/execute 没这问题: 它进门就 resolve, 抓之前类别已定。
    {
      std::lock_guard<std::mutex> lk(cls_mtx_);
      held_category_ = cat;
      have_held_category_ = true;
    }

    double stroke = cur.pose.position.z - box_top_z - insert_shortfall_;
    const double raw = stroke;
    stroke = std::max(insert_stroke_min_, std::min(insert_stroke_max_, stroke));
    RCLCPP_INFO(logger_,
                "③ 行程 %.1fmm (TCP z=%.3f - 盒顶 z=%.3f = 地面%.3f+厚%.0fmm - 少插%.0fmm), 用 %.1fmm",
                raw * 1000, cur.pose.position.z, box_top_z, ground_z_,
                thickness * 1000, insert_shortfall_ * 1000, stroke * 1000);

    // 当前 tool 系 -Z 在 base_link 中的方向 = R(cur)*(0,0,-1)
    const auto & q = cur.pose.orientation;
    // R*(0,0,1) 第三列:
    const double zx = 2.0 * (q.x * q.z + q.w * q.y);
    const double zy = 2.0 * (q.y * q.z - q.w * q.x);
    const double zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);

    geometry_msgs::msg::Pose wp = cur.pose;   // 姿态保持当前
    wp.position.x -= zx * stroke;              // 沿 -Z 下插
    wp.position.y -= zy * stroke;
    wp.position.z -= zz * stroke;

    std::vector<geometry_msgs::msg::Pose> waypoints{wp};
    moveit_msgs::msg::RobotTrajectory traj;
    move_group_->setStartStateToCurrentState();
    const double fraction = move_group_->computeCartesianPath(waypoints, 0.005, 0.0, traj);
    RCLCPP_INFO(logger_, "③ 直插笛卡尔路径覆盖 %.0f%%", fraction * 100);
    if (fraction < 0.9) { RCLCPP_ERROR(logger_, "③ 直插路径覆盖不足"); return false; }
    // 下插降速: 压 J6 间隙窜动 (见 insert_velocity_scaling_ 声明处注释).
    // ⚠️ 这两个 setMax*ScalingFactor 在 computeCartesianPath 之后调用, 对 cartesian path 的
    // 时间参数化无影响 (MoveIt 的 cartesian path 不读这俩因子, 只有 plan() 读). 留着是历史
    // 遗物, 实际全速跑. 真要降速得在 computeCartesianPath 前调 + 用 plan() 而非 cartesian path.
    // 之前这里曾加手动重参数化 (time_from_start * 1/scaling + 清空 v/a) 想真把 30% 落下去,
    // 结果直插变 5s 太慢, 用户实测反馈"之前好好的", 已撤回. 偶发 TIMED_OUT 用放宽
    // move_group 的 allowed_execution_duration_multiplier 解决 (见 move_group.launch.py).
    move_group_->setMaxVelocityScalingFactor(insert_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(insert_velocity_scaling_);
    const bool moved = move_group_->execute(traj) == moveit::core::MoveItErrorCode::SUCCESS;
    restorePlanScaling();
    if (!moved) return false;

    // 抽真空 suck_duration_ 秒建立负压, 再发 STOP 转入保压 —— 固件的 PUMP_STOP 是
    // "关泵+关阀", 气路封住靠密封维持负压, 盒子仍吸着 (真放开是 PUMP_RELEASE 开阀).
    // 不能一直发 SUCK: 泵持续空转发热, 且吸住后继续抽没有收益.
    publishPump(PUMP_SUCK);
    RCLCPP_INFO(logger_, "③ 已下插 %.1fcm, 发 /pump_cmd 1 抽真空 %.1fs",
                stroke * 100, suck_duration_);
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(suck_duration_ * 1000)));
    publishPump(PUMP_STOP);
    RCLCPP_INFO(logger_, "③ 发 /pump_cmd 0 转入保压 (关泵关阀, 盒仍吸着)");
    attachBox(allow_tray_touch, thickness);  // 盒进规划场景: 后续放置绕开托盘边框
    return true;
  }

  // 把盒子作为 attached collision object 挂到吸盘: 吸取后吸盘末端在盒顶, 盒心在吸盘
  // -Z(工具系向下)方向 0.0125 处. 此后 MoveIt 规划放置路径会考虑这块几何, 从上方入托盘
  // 而非侧向蹭过边框. touch_links 默认只放吸盘 link (吸取处接触不误报);
  // allow_tray_touch=true 时再把托盘 link 加进 touch_links —— 盒与 Link_11 的接触是预期的:
  // 卸货取盒时盒本在托盘里几何重叠, 放盒到位时盒底落在托盘面. 不豁免则规划器判碰撞,
  // 取盒连抬起都规划不了, 放盒直下段中途被截断.
  // box_z: 当前类别真实盒高 (用 currentThickness(ci) 传进来), 替代 box_size_z_ 默认值 25mm.
  // 2026-07-29: 25mm 对 category 1 (22mm) 多 3mm, 碰撞体偏低撞托盘围栏顶, 直下覆盖 84%.
  // 改成按类别传真实厚度, 碰撞体贴近真实, 围栏 clearance 多 3mm.
  void attachBox(bool allow_tray_touch, double box_z)
  {
    moveit_msgs::msg::CollisionObject co;
    co.id = kCarriedBoxId;
    co.header.frame_id = ee_link_;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions = {box_size_x_, box_size_y_, box_z};
    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;
    p.position.z = -box_z / 2.0;   // 盒心在吸盘末端下方半个盒高
    co.primitives.push_back(prim);
    co.primitive_poses.push_back(p);
    co.operation = co.ADD;

    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = ee_link_;
    aco.object = co;
    // ee_link_ (suction_tip) 本身没有几何体, 它只是 suction_link 的一个子坐标系; 真正带
    // 网格的是父链接 suction_link. 光豁免 suction_tip 等于没豁免 —— 盒顶面就贴在吸盘本体
    // 上, 盒 vs suction_link 判碰撞是必然的, 只看 OMPL 采样到哪个点才暴露.
    // 2026-07-30 实跑: 卸货第二个盒 transit 规划 "Invalid states at index [21] out of 61",
    // 接触对 carried_box <-> suction_link, 而同一段前几轮都过 —— 典型的边缘几何偶发.
    aco.touch_links = {ee_link_, suction_body_link_};
    if (allow_tray_touch) aco.touch_links.push_back(tray_frame_);
    psi_->applyAttachedCollisionObject(aco);
    RCLCPP_INFO(logger_, "盒子已 attach 到 %s 作碰撞体 (%.0fx%.0fx%.0fmm)%s", ee_link_.c_str(),
                box_size_x_ * 1000, box_size_y_ * 1000, box_z * 1000,
                allow_tray_touch ? " [豁免与托盘接触]" : "");
  }

  // 释放盒子后从规划场景摘除: detach + 移除, 以免残留碰撞体挡住回 ready 的规划.
  void detachBox()
  {
    move_group_->detachObject(kCarriedBoxId);
    psi_->removeCollisionObjects({kCarriedBoxId});
    // 盒离手, 锁存的类别随之失效: 不清则下一轮 placeCategory() 会拿上一个盒的类别去
    // 定托盘, 而下一个盒未必同类。detachBox 是盒离手的唯一出口(放置/卸货/dry_run 都过它),
    // 清在这里不会漏。
    {
      std::lock_guard<std::mutex> lk(cls_mtx_);
      have_held_category_ = false;
    }
    RCLCPP_INFO(logger_, "盒子已 detach 并移出规划场景");
  }

  // 放置完回 ready(烘焙初始姿态 P): 臂收回身前, 底盘再走不拖着伸出的臂.
  bool moveToReady()
  {
    move_group_->setStartStateToCurrentState();
    move_group_->setNamedTarget("ready");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "回 ready 规划失败"); return false;
    }
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
    // 必须等停稳才返回, 同 moveToLook/moveToHome: 控制器报完成时关节尚未停到位, 调用方
    // 若背靠背接下一段, setStartStateToCurrentState() 会抓到未停稳的中间态, 从这个假起点
    // 规划出的轨迹起点与真实位置有偏差, 执行时先纠偏再走正路 —— 表现为某关节"卡一下又
    // 突然跟上"。2026-08-01 实测: 遥控停机键连调 ready->home, ready 轨迹 8ms 报成功
    // (本就在 ready 位, 近零长度), 80ms 后 home 即开始规划, J2 出现前后来回。
    // 手动一步步调时中间有人为间隔, 所以这个缺陷一直被遮着。
    settle();
    RCLCPP_INFO(logger_, "已回 ready 位");
    return true;
  }

  bool moveToHome()
  {
    move_group_->setStartStateToCurrentState();
    move_group_->setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "回 home 规划失败"); return false;
    }
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
    settle();
    RCLCPP_INFO(logger_, "已回 home 收拢位");
    return true;
  }

  // 腕姿态回正到 coarse_yaw_ (吸盘朝下) + TCP 抬到离地 level_height_, x/y 原地不动.
  // 一次 plan 同时改姿态和 z: 分两步做的话中间态可能是"歪着还没抬起", 更容易撞.
  bool levelAndLift(std::string & msg)
  {
    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) { msg = "取当前位姿失败"; return false; }
    geometry_msgs::msg::Pose target;
    target.position.x = cur.pose.position.x;
    target.position.y = cur.pose.position.y;
    target.position.z = ground_z_ + level_height_;
    target.orientation = yawToQuat(coarse_yaw_);
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      msg = "姿态回正规划失败"; return false;
    }
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      msg = "姿态回正执行失败"; return false;
    }
    settle();
    char buf[96];
    std::snprintf(buf, sizeof(buf), "已回正 (吸盘朝下, TCP z=%.3f 离地%.0fcm)",
                  target.position.z, level_height_ * 100);
    msg = buf;
    RCLCPP_INFO(logger_, "%s", buf);
    return true;
  }

  // 看货姿势: 取 ready 关节值, J1(Joint_11) 加 look_j1_offset_(默认 +90°), 手眼相机转向
  // 货物侧再做闭环抓取(为了让视觉看见). 纯关节目标, 不算笛卡尔.
  bool moveToLook()
  {
    std::map<std::string, double> joints = move_group_->getNamedTargetValues("ready");
    if (joints.find(j1_name_) == joints.end()) {
      RCLCPP_ERROR(logger_, "看货姿势: ready 状态里找不到关节 %s", j1_name_.c_str());
      return false;
    }
    joints[j1_name_] += look_j1_offset_;
    move_group_->setStartStateToCurrentState();
    move_group_->setJointValueTarget(joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "看货姿势规划失败"); return false;
    }
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
    // 必须等停稳才返回: 感知端每帧用**当前** TF 把相机系点转到 base_link, 臂没停稳时
    // 同一个静止盒子的 base_link 坐标会随 FK 一起滑. 2026-07-30 实测 (run_cycle.sh 背靠背
    // 调 look->execute): 类别1 盒的像素(523,193)/深度 0.368m/相机系[0.368,0.036,0.081] 三者
    // 帧帧不动, 而 base_link 在 0.8s 内从 (-0.333,-0.021) 滑到 (-0.285,-0.165), 共 145mm,
    // yaw -72°->-43°. waitObject 的排空+4 次重采(~1.4s)全落在这个窗口里, 一致性门 4 次
    // 全拦, execute 报 "无 object_pose" 中止.
    // 其它每一段(①/笛卡尔步进/转腕/放置各段)执行后都调 settle(), look 原先是唯一漏的.
    // 手动一步步调时中间有人为间隔, 所以一直没暴露.
    settle();
    RCLCPP_INFO(logger_, "已到看货姿势 (ready %s%+.0f°)", j1_name_.c_str(),
                look_j1_offset_ * 180.0 / M_PI);
    return true;
  }

  // 放置到地面: 相对抬起(携盒) -> 到目的地上方(top-down) -> 笛卡尔直下到吸盘 z_tip
  // (盒底离地 ~5mm) -> 释放 + detach. 不在半空释放, 盒子落稳. 卸货用.
  bool placeAt(double x, double y, double z_tip, double clearance)
  {
    // 抬起 (相对当前 +Z, 携盒). 只在起点低于下面那个"目的地上方"航点时才抬 ——
    // 2026-08-03: ○ 放地面前遥控先调 /grasp/level (TCP 抬到离地 20cm, z≈0.152), 而航点
    // z = place_z_ + place_clearance_ = 0.150, 于是这一抬把臂送到 0.182 再降回 0.150,
    // 纯多余的一上一下. 抬起的用途是"脱离接触面, 别让规划器把盒沿地面拖过去", 起点已经
    // 悬空高于航点时它不成立.
    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) return false;
    if (cur.pose.position.z < z_tip + clearance - 0.005) {
      geometry_msgs::msg::Pose up = cur.pose;
      up.position.z += lift_height_;
      std::vector<geometry_msgs::msg::Pose> wps{up};
      moveit_msgs::msg::RobotTrajectory traj;
      move_group_->setStartStateToCurrentState();
      if (move_group_->computeCartesianPath(wps, 0.005, 0.0, traj) > 0.5) {
        move_group_->execute(traj);
        settle();
        RCLCPP_INFO(logger_, "放置: 已抬起 %.0fcm", lift_height_ * 100);
      }
    } else {
      RCLCPP_INFO(logger_, "放置: 起点 z=%.3f 已高于目的地上方航点 %.3f, 跳过抬起",
                  cur.pose.position.z, z_tip + clearance);
    }

    // 到目的地上方 (维持吸盘朝下姿态)
    geometry_msgs::msg::Pose above;
    above.position.x = x;
    above.position.y = y;
    above.position.z = z_tip + clearance;
    above.orientation = cur.pose.orientation;
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(above);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "放置: 规划到 (%.2f,%.2f) 上方失败", x, y); return false;
    }
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
    settle();

    // 笛卡尔直下到 z_tip: 盒子落到近地面才释放, 不半空抛
    geometry_msgs::msg::Pose down = above;
    down.position.z = z_tip;
    std::vector<geometry_msgs::msg::Pose> dwps{down};
    moveit_msgs::msg::RobotTrajectory dtraj;
    move_group_->setStartStateToCurrentState();
    const double frac = move_group_->computeCartesianPath(dwps, 0.005, 0.0, dtraj);
    if (frac < 0.9) {
      RCLCPP_ERROR(logger_, "放置: 直下贴地路径覆盖不足 %.0f%%", frac * 100); return false;
    }
    if (move_group_->execute(dtraj) != moveit::core::MoveItErrorCode::SUCCESS) return false;

    releasePulse();
    detachBox();
    RCLCPP_INFO(logger_, "放置: 已直下到吸盘 z=%.3f (盒底离地~5mm), 已释放并关阀", z_tip);
    return true;
  }

  // 放到"标定好的绝对目标位姿"(含朝向), 顶向下入位: 竖直爬升到 transit 高度(携盒) -> 规划到高空 transit
  // (标定 xy+朝向, z = transit_z_ 绝对) -> 笛卡尔垂直直下到标定位姿 -> 释放 + detach.
  // 放托盘用: 托盘在肩后死区, 唯一可达是工具 yaw≈180°朝下的特定位姿(RViz 实测标定).
  // 早前直接 plan() 到标定位姿: 盒 attach 后规划器为让盒全程避开边框会绕大圈甩臂; 改成
  // 顶上空旷处入、再垂直直下, 盒竖直进托盘不蹭边框, 规划器无需绕路, 路径直不甩.
  // 2026-07-29 再改: 原 plan 到 tray_clearance_ 低空 (target.z+0.06), 左盘有盒时规划被
  // placed_t1 挡到 abort; 改成 plan 到 transit_z_ 高空 (0.30), 两端都在自由空间, 失败率≈0.
  //
  // tray: 本次落盘的托盘号. 直下段前临时移出该托盘已落盒 (它们正是本层落点, 当障碍会挡住
  // 直下), 段后加回; 别的托盘已落盒全程留在场景当障碍, 防规划器横扫 (2026-07-28 蹭飞事故).
  //
  // (原先这里还有一条 placeCartesian "斜切进位" 抄近道: 抬到低空 0.14 再一条纯笛卡尔直线
  //  横切入位, 想治 OMPL 随机采样绕高包. 2026-07-30 删: 8/8 轮笛卡尔覆盖只有 9~13%, 即
  //  刚离开起点就断 —— 保持工具朝向横切那条直线上大半位姿没有 IK 解, 是可达性问题, 不是
  //  被碰撞体挡的, 抬高到 0.20 也已逼近 transit_z_ 没有意义. 且盒 attach 时 touch_links
  //  含 tray_frame_, 盒与托盘碰撞全程豁免, 低空横切蹭围栏侧壁规划器不报、执行不停, 只能
  //  靠人看. 零收益 + 无声风险, 故放置统一走下面的高空 transit_z_ 一条路.)
  // unload_mode: 走卸货那组 transit 高度候选 (从低到高试) 而非抓取放置的单一 transit_z_.
  // 卸货目的地在地面而非托盘里, 不需要 transit_z_ 那么高 (它是为"携盒越过别盘那摞 2 层盒"
  // 定的), 白飞 10cm.
  bool placeAtPose(const geometry_msgs::msg::Pose & target, const char * what, int tray,
                   bool unload_mode = false)
  {
    // 四步渐进安全测试: 放置段整体降速(默认 0.2, place_velocity_scaling 再乘一档),
    // 收尾恢复. dry_run 时只 plan+打印, 不 execute, 也不动气泵/detach (盒仍吸着不释放).
    const double v = plan_velocity_scaling_ * place_velocity_scaling_;
    move_group_->setMaxVelocityScalingFactor(v);
    move_group_->setMaxAccelerationScalingFactor(v);
    if (dry_run_) {
      RCLCPP_WARN(logger_, "放置(%s): [dry_run] 只规划打印, 不执行/不释放", what);
    }

    // 本托盘已落盒必须在**爬升之前**就移出, 而非只在 transit 规划前移出.
    // 2026-07-31 实跑: 卸货取完第 2 层后吸盘停在 z≈0.104, 手上盒底 0.089 比同盘第 1 层盒顶
    // 0.101 还低 12mm —— 而 pickFromTray 收尾已把第 1 层加回场景, 于是笛卡尔爬升的**起点
    // 状态就判在碰撞里**, 第一步即断, 覆盖 0%. 臂没爬, 贴着托盘去规划 transit, 又退化成
    // "只有一端在高空", 横移时擦到别盘那摞盒 (用户实机观察到撞左托盘).
    // 四次卸货完全对上: 取 l2(下面还有 l1) 两次都是 0%, 取 l1(下面空) 两次都是 100%.
    // 本盘已放盒是落点/来源, 不是障碍, 整个放置段都不该当障碍; 别盘的留着当障碍.
    // ⚠️ 此行之后任何失败返回前都必须 showTrayBoxes 加回.
    hideTrayBoxes(tray);

    // 携盒纯竖直爬升到 transit 高度, 之后才去规划 transit (原先只相对抬 lift_height_=3cm).
    // 2026-07-30: 抬 3cm 后 TCP 仍在 z≈-0.008 基本贴地, 而 transit 航点在 0.26 —— 只有一端
    // 在高空. RRTConnect 在哪一段爬升是随机的, 它可以先横移再爬, 于是低空掠过**别盘**那摞
    // 2 层盒: plan 在 95ms 后报 "Invalid states at index [26] out of 48", 接触对
    // placed_t1_l2 <-> carried_box (0 号托盘第 1 层实跑). 两端同高后, 路径没有理由往下钻。
    // 爬升是笛卡尔纯直线, 截断安全: 覆盖不足时执行到的那段仍然只是竖直向上, 不会横移蹭东西,
    // 故拿到多少覆盖就走多少, 没到位只告警 (transit 规划失败会在下面正常报错返回).
    // 取 transitCandidates 的第一档(最低)作爬升目标: 更高的档次起点已在高空, 无需再爬。
    const auto climb_cands = transitCandidates(unload_mode);
    const double climb_z = climb_cands.empty() ? 0.0 : climb_cands.front();
    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) { showTrayBoxes(tray); restorePlanScaling(); return false; }
    // 爬升目标就是 transit 高度本身, 不再叠 lift_height_.
    // 2026-08-03: 原式 max(cur.z + lift_height_, climb_z) 在"起点已高过 transit"时会凭空
    // 多抬 3cm 再下降 —— ■ 放托盘先经 /grasp/ready(TCP z≈0.294), max(0.324, 0.18)=0.324,
    // 实机看到的就是"到 ready 后先抬一下才下去"(日志: 已竖直爬升到 z=0.324 目标 0.324).
    // 爬升的用途只有一个: 从贴地的吸取位把臂拉到与 transit 航点同高, 免得规划器先横移再爬、
    // 低空掠过别盘那摞盒. 起点本就够高时这一段没有任何作用, 整段跳过.
    if (cur.pose.position.z < climb_z - 0.005) {
      geometry_msgs::msg::Pose up = cur.pose;
      up.position.z = climb_z;
      std::vector<geometry_msgs::msg::Pose> wps{up};
      moveit_msgs::msg::RobotTrajectory traj;
      move_group_->setStartStateToCurrentState();
      const double up_frac = move_group_->computeCartesianPath(wps, 0.005, 0.0, traj);
      if (up_frac > 0.5 && !dry_run_) {
        move_group_->execute(traj);
        settle();
        geometry_msgs::msg::PoseStamped now;
        const double reached = currentTcp(now) ? now.pose.position.z : up.position.z;
        RCLCPP_INFO(logger_, "放置(%s): 已竖直爬升到 z=%.3f (目标 %.3f, 覆盖 %.0f%%)",
                    what, reached, up.position.z, up_frac * 100);
        if (reached < up.position.z - 0.02) {
          RCLCPP_WARN(logger_, "放置(%s): 爬升未到位 (差 %.0fmm), transit 规划可能被别盘已放盒挡到",
                      what, (up.position.z - reached) * 1000);
        }
      } else if (dry_run_) {
        RCLCPP_WARN(logger_, "放置(%s): [dry_run] 爬升到 z=%.3f 路径覆盖 %.0f%%, 不执行",
                    what, up.position.z, up_frac * 100);
      } else {
        RCLCPP_WARN(logger_, "放置(%s): 爬升路径只覆盖 %.0f%%, 不执行爬升, 直接试 transit",
                    what, up_frac * 100);
      }
    } else {
      RCLCPP_INFO(logger_, "放置(%s): 起点 z=%.3f 已在 transit 首档 %.3f 之上, 跳过爬升",
                  what, cur.pose.position.z, climb_z);
    }

    // 规划到高空 transit 航点 (xy+朝向用标定值, z 绝对高度): 两端都在自由空间, 规划器不用
    // 钻障碍缝. 之后纯垂直直下到 target.
    //
    // 高度逐级试 (2026-07-30 改): 原先写死一个高度, 实跑发现卸货点这一带的 IK 可达区间很窄
    // 且两段各有各的约束 —— 0.16 时 transit 规划成功(68点)但直下只覆盖 87%; 抬到 0.20 反而
    // transit 本身规划不出来; 0.26 两段都通. 中间高度落在两个可行构型之间的空隙里, 挑不出
    // 一个"够低又两段都过"的定值, 故改成从低到高试, 取第一个两段都过的.
    // ⚠️ 必须两段都验过才执行 transit: 先执行再发现直下段不行, 盒子就吸着卡在半空了
    // (0.16 那轮的实际后果). 直下段试算的起点接 transit 规划的终点, 不动真臂.
    // ⚠️ 重算失败必须退下一档重来, 不能直接返回失败 (2026-07-31 实跑教训):
    // 试算用 transit 规划终点的**理论**关节值, 执行后有停位误差, 按实测起点重算可能掉档 ——
    // 0.18 档实测 试算 100% -> 重算 88%. 收紧试算阈值治不了: 差异不是裕度不够, 而是停位误差
    // 把起点挪到了另一个 IK 分支附近. 旧代码此时直接 return false, 盒吸着卡半空, 而 0.20/
    // 0.26 两档根本没试过. 现在整个"transit 规划 + 试算 + 执行 + 重算"都在循环里, 重算破了
    // 就换下一档重新爬升+规划, 只有全部档次都败才算失败.
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::msg::RobotTrajectory dtraj;
    geometry_msgs::msg::Pose transit = target;
    std::vector<geometry_msgs::msg::Pose> dwps{target};
    const auto cands = transitCandidates(unload_mode);
    bool descended = false;
    double used_z = 0.0;

    for (size_t ci = 0; ci < cands.size() && !descended; ++ci) {
      const double z = cands[ci];
      // 退档后先补爬到本档高度: 上面那次爬升只爬到首档(最低档), 而更高档的 transit 航点在
      // 更高处. 不补爬就又变成"起点低、终点高"那个让规划器低空横移的老毛病.
      // 首档(ci==0)不用补, 循环前已经爬过.
      if (ci > 0) {
        geometry_msgs::msg::PoseStamped c2;
        if (currentTcp(c2) && c2.pose.position.z < z - 0.005) {
          geometry_msgs::msg::Pose u2 = c2.pose;
          u2.position.z = z;
          std::vector<geometry_msgs::msg::Pose> w2{u2};
          moveit_msgs::msg::RobotTrajectory t2;
          move_group_->setStartStateToCurrentState();
          const double f2 = move_group_->computeCartesianPath(w2, 0.005, 0.0, t2);
          if (f2 > 0.5) {
            move_group_->execute(t2);
            settle();
          }
          RCLCPP_INFO(logger_, "放置(%s): 退档补爬到 z=%.3f (覆盖 %.0f%%)", what, z, f2 * 100);
        }
      }
      transit.position.z = z;
      move_group_->setStartStateToCurrentState();
      move_group_->setPoseTarget(transit);
      if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(logger_, "放置(%s): transit z=%.3f 规划失败, 试下一档", what, z);
        continue;
      }
      // 直下段试算: 起点接上面 transit 规划的终点 (真机也这么接, 因为此刻还没执行 transit).
      // 试算阈值 0.99 而非执行时的 0.9: 卡在 0.9 上的档次实测必破, 早点让给下一档.
      const auto cur_state = move_group_->getCurrentState();
      if (!cur_state) {
        RCLCPP_ERROR(logger_, "放置: 取当前状态失败");
        showTrayBoxes(tray); restorePlanScaling(); return false;
      }
      moveit::core::RobotState start(*cur_state);
      const auto & jt = plan.trajectory_.joint_trajectory;
      start.setVariablePositions(jt.joint_names, jt.points.back().positions);
      move_group_->setStartState(start);
      const double frac = move_group_->computeCartesianPath(dwps, 0.005, 0.0, dtraj);
      if (frac < 0.99) {
        RCLCPP_WARN(logger_, "放置(%s): transit z=%.3f 的直下段试算只覆盖 %.0f%% (<99%%), 试下一档",
                    what, z, frac * 100);
        continue;
      }
      RCLCPP_INFO(logger_,
                  "放置(%s): transit z=%.3f 两段试算全通 (transit %zu 点, 直下 %.0f%%)",
                  what, z, plan.trajectory_.joint_trajectory.points.size(), frac * 100);

      if (dry_run_) {
        RCLCPP_WARN(logger_, "放置(%s): [dry_run] 目标 z=%.3f, 到此为止不执行",
                    what, target.position.z);
        showTrayBoxes(tray); restorePlanScaling();
        return true;   // dry_run 视为通过(规划全成功), 但不真放/不计堆叠由调用侧另判
      }
      if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(logger_, "放置(%s): transit z=%.3f 执行失败, 试下一档", what, z);
        continue;
      }
      settle();
      // 按实测起点重算 (理论轨迹会被 MoveIt 的 start-state 校验拒掉).
      move_group_->setStartStateToCurrentState();
      const double frac2 = move_group_->computeCartesianPath(dwps, 0.005, 0.0, dtraj);
      if (frac2 < 0.9) {
        RCLCPP_WARN(logger_,
                    "放置(%s): transit z=%.3f 直下段按实测起点重算只覆盖 %.0f%% (试算 %.0f%%), 试下一档",
                    what, z, frac2 * 100, frac * 100);
        continue;   // 臂此刻停在 z 高度的 transit 点上, 下一档从这里重新规划, 安全
      }
      if (move_group_->execute(dtraj) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(logger_, "放置(%s): transit z=%.3f 直下段执行失败, 试下一档", what, z);
        continue;
      }
      settle();
      descended = true; used_z = z;
    }
    if (!descended) {
      RCLCPP_ERROR(logger_, "放置(%s): 所有 transit 高度都没能走完两段 (%.3f,%.3f)", what,
                   target.position.x, target.position.y);
      showTrayBoxes(tray); restorePlanScaling(); return false;
    }
    RCLCPP_INFO(logger_, "放置(%s): 已用 transit z=%.3f 走完两段", what, used_z);

    releasePulse();
    detachBox();
    showTrayBoxes(tray);   // 落盘后本托盘旧盒加回; pushLayer 再把这个新盒加进去
    restorePlanScaling();
    RCLCPP_INFO(logger_, "放置(%s): 垂直入位到 (%.3f,%.3f,%.3f), 已释放并关阀", what,
                target.position.x, target.position.y, target.position.z);
    return true;
  }

  // 执行完一段后静置再规划下一段. 控制器报 "successfully finished" 只表示轨迹点发完,
  // 实测关节还没停到位: 携盒抬起后 Joint_12 实测欠到指令 0.039rad (2.2°, 重力下垂+减速器
  // 间隙), 而 MoveIt 执行前校验 start 与实测的偏差上限是 0.01rad. 紧接着规划会拿到尚未
  // 回落的状态当起点, 提交时实测已落下去 -> "start point deviates from current robot
  // state" 直接 ABORTED. 放宽那个容差只是把陈旧起点放进去, 所以改成等实测追上来.
  // 实测 0.5s 定时等不够(放置直下后 Joint_15 还在爬), 所以改成等"关节真的停了": 每 100ms
  // 采一次实测关节, 连续两次最大变化 < settle_eps_ 才算停稳, 最多等 settle_sec_.
  void settle()
  {
    std::vector<double> prev;
    const rclcpp::Time t0 = node_->now();
    int quiet = 0;
    while (rclcpp::ok() && (node_->now() - t0).seconds() < settle_sec_) {
      rclcpp::sleep_for(
        std::chrono::milliseconds(static_cast<int>(settle_poll_sec_ * 1000)));
      std::vector<double> cur;
      // getCurrentState 拿不到 100ms 内的新状态时返回空指针, 直接 -> 解引用会段错误
      // (2026-07-29 实机: 精修第2步后 joint_states 晚到 42ms, 整个 grasp_node SIGSEGV 挂掉,
      // 服务调用永不返回). 空了当"这拍没采到"跳过继续轮询, 不进停稳判据.
      const auto state = move_group_->getCurrentState(0.1);
      if (!state) {
        RCLCPP_WARN(logger_, "静置采样: joint_states 未在 100ms 内更新, 跳过本拍");
        quiet = 0;
        continue;
      }
      state->copyJointGroupPositions(planning_group_, cur);
      if (prev.size() == cur.size()) {
        double dmax = 0.0;
        for (size_t i = 0; i < cur.size(); ++i) {
          dmax = std::max(dmax, std::fabs(cur[i] - prev[i]));
        }
        if (dmax < settle_eps_) {
          if (++quiet >= 2) return;
        } else {
          quiet = 0;
        }
      }
      prev = cur;
    }
    RCLCPP_WARN(logger_, "静置等满 %.1fs 关节仍在动, 继续 (下一段可能被起点容差拒)",
                settle_sec_);
  }

  // 某段用完自己的降速倍率后, 恢复全局规划缩放 (plan_velocity_scaling).
  void restorePlanScaling()
  {
    move_group_->setMaxVelocityScalingFactor(plan_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(plan_velocity_scaling_);
  }

  // ---- 工具 ----
  // 等一个**可信**的 object_pose: 要求连采几帧彼此一致才采信 (可选先排空视觉管线).
  // 原先必须排空: 摆臂刚停那刻管线里积压的是运动中拍的画面, 而感知端用**当前** TF 把它转到
  // base_link -> 图像姿态与 TF 姿态错配, 坐标整体偏掉.
  // 2026-07-30 起感知端按**图像自己的 stamp** 查 TF, 陈旧帧配同时刻 TF, 坐标照样正确 ->
  // 错配从根上没了, object_flush_sec 已置 0 (仍留参数, 需要时可再打开).
  // 帧间一致性门保留: 它是真判据, 不依赖排空.
  // 全败则返回 false 让上层中止 —— 此刻还没吸盒子, 臂停在 look 位, 中止是安全的.
  bool waitObject(geometry_msgs::msg::PoseStamped & out, double timeout_s)
  {
    for (int t = 0; t < object_resample_tries_; ++t) {
      rclcpp::sleep_for(std::chrono::milliseconds(
        static_cast<int>(object_flush_sec_ * 1000)));   // 排空运动中拍的积压帧
      if (sampleObjectStable(out, timeout_s)) return true;
    }
    RCLCPP_ERROR(logger_, "取不到一致的 object_pose (重采 %d 次)", object_resample_tries_);
    return false;
  }

  // 连采 object_consist_frames_ 帧新鲜 object_pose, 要求彼此散布 < object_consist_tol_.
  // 只收 stamp 比上一帧新的, 避免同一帧被重复计入(检测 ~11Hz, 轮询快得多).
  // 输出取末帧(而非均值): 静止时各帧本就重合, 取均值无收益; 末帧最新, 姿态字段也无需插值.
  bool sampleObjectStable(geometry_msgs::msg::PoseStamped & out, double timeout_s)
  {
    std::vector<geometry_msgs::msg::PoseStamped> buf;
    rclcpp::Time last(0, 0, RCL_ROS_TIME);
    const rclcpp::Time t0 = node_->now();
    rclcpp::WallRate r(60.0);
    while (rclcpp::ok() && static_cast<int>(buf.size()) < object_consist_frames_) {
      if ((node_->now() - t0).seconds() > timeout_s) {
        RCLCPP_WARN(logger_, "object_pose 采样超时 (%.1fs 内只收到 %zu/%d 帧)",
                    timeout_s, buf.size(), object_consist_frames_);
        return false;
      }
      geometry_msgs::msg::PoseStamped p;
      if (latestObject(p)) {
        const rclcpp::Time st(p.header.stamp);
        if (st > last) { last = st; buf.push_back(p); }
      }
      r.sleep();
    }
    double spread = 0.0;
    for (const auto & a : buf) {
      spread = std::max(spread,
                        std::hypot(a.pose.position.x - buf.back().pose.position.x,
                                   a.pose.position.y - buf.back().pose.position.y));
    }
    if (spread > object_consist_tol_) {
      RCLCPP_WARN(logger_,
                  "object_pose %d 帧不一致 (散布 %.1fmm > %.1fmm, 疑摆臂中拍的帧), 重采",
                  object_consist_frames_, spread * 1000, object_consist_tol_ * 1000);
      return false;
    }
    out = buf.back();
    RCLCPP_INFO(logger_, "object_pose 已稳定: (%.3f,%.3f,%.3f) 散布 %.1fmm",
                out.pose.position.x, out.pose.position.y, out.pose.position.z,
                spread * 1000);
    return true;
  }

  bool latestObject(geometry_msgs::msg::PoseStamped & out)
  {
    std::lock_guard<std::mutex> lk(obj_mtx_);
    if (!have_object_) return false;
    const double age = (node_->now() - rclcpp::Time(last_object_.header.stamp)).seconds();
    if (age > object_stale_sec_) return false;
    out = last_object_;
    return true;
  }

  // 等一个到达时刻晚于"进入本函数"的候选数 —— 即摆到 look 位之后才拍的那一帧.
  // 不看消息 stamp 而看接收时刻: 判据是"这一帧是不是臂停稳后才进来的", 接收时刻正是这个
  // 量; 消息 stamp 是曝光/发布时刻, 与"管线里积压了多久"无关 (轮询期间会连收好几帧,
  // 取最后一个即最新).
  // 超时则返回最后已知值 + false, 让调用方在 message 里注明过期而不是假装看见 0 个 ——
  // 报 0 会让状态机以为抓完了直接收工, 而真相可能是感知节点没起.
  bool waitPickCount(int & out, double timeout_s)
  {
    const rclcpp::Time t0 = node_->now();
    rclcpp::WallRate r(60.0);
    while (rclcpp::ok() && (node_->now() - t0).seconds() < timeout_s) {
      {
        std::lock_guard<std::mutex> lk(poses_mtx_);
        if (last_poses_stamp_ > t0) { out = last_pick_count_; return true; }
      }
      r.sleep();
    }
    std::lock_guard<std::mutex> lk(poses_mtx_);
    out = last_pick_count_;
    RCLCPP_WARN(logger_, "候选数: %.1fs 内没收到新的 %s (报最后已知值 %d)",
                timeout_s, poses_topic_.c_str(), out);
    return false;
  }

  bool latestCamPoint(geometry_msgs::msg::PointStamped & out)
  {
    std::lock_guard<std::mutex> lk(cam_mtx_);
    if (!have_cam_point_) return false;
    const double age = (node_->now() - rclcpp::Time(last_cam_point_.header.stamp)).seconds();
    if (age > object_stale_sec_) return false;
    out = last_cam_point_;
    return true;
  }

  // 最新 θ_img (度). 过期(object_stale_sec_)则失败 —— 宁可不抓也不用陈旧朝向去转腕.
  bool latestAxisAngle(double & out)
  {
    std::lock_guard<std::mutex> lk(axis_mtx_);
    if (!have_axis_angle_) return false;
    if ((node_->now() - last_axis_stamp_).seconds() > object_stale_sec_) return false;
    out = last_axis_angle_;
    return true;
  }

  // 取 axis_avg_frames_ 帧新鲜 θ_img 的均值(度), 压掉单帧噪声.
  // 只收 stamp 比上一帧新的, 避免同一帧被重复计入(检测 ~11Hz, 轮询快得多).
  // 折到 (-90,90] 的角度不能直接求算术均值 —— 盒摆在边界附近时检测值在 +88/−88 间反复翻,
  // 算术均值会得到 0. 故走单位向量的 2θ 表示 (长轴无向, 差 180° 同朝向) 再折回.
  bool avgAxisAngle(double & out_deg)
  {
    double sx = 0.0, sy = 0.0;
    int n = 0;
    rclcpp::Time last(0, 0, RCL_ROS_TIME);
    const rclcpp::Time t0 = node_->now();
    rclcpp::WallRate r(60.0);
    std::vector<double> raw;
    raw.reserve(axis_avg_frames_);
    while (rclcpp::ok() && n < axis_avg_frames_) {
      if ((node_->now() - t0).seconds() > cam_wait_timeout_) return false;
      {
        std::lock_guard<std::mutex> lk(axis_mtx_);
        if (have_axis_angle_ &&
            (node_->now() - last_axis_stamp_).seconds() <= object_stale_sec_ &&
            last_axis_stamp_ > last)
        {
          last = last_axis_stamp_;
          raw.push_back(last_axis_angle_);
          const double a2 = 2.0 * last_axis_angle_ * M_PI / 180.0;
          sx += std::cos(a2);
          sy += std::sin(a2);
          ++n;
        }
      }
      r.sleep();
    }
    // 一致性门: 合成向量长度 |R|/n 是 2θ 空间的集中度, 帧间一致时 ≈1. 长短边互换(近正方形
    // 盒长宽比仅 1.11~1.14)在 2θ 空间是 180° 反向, 几帧 -45° 混几帧 +45° 求和会互相抵消,
    // atan2(0,0) 吐出 0 —— 一个**任何单帧里都不存在**的值, 却看着像"已对齐".
    // 2026-07-28 实测: 原始帧全是 -44°, 本段首次采样报 +2.5°(前一轮 +10.9°), 正是这个抵消.
    // 抵消出的假值比噪声危险得多: 它把斜率算成 5.4~6.2(真值 ~0.9), 一步只转几度, 直接
    // 让整段对齐失败. 故不一致就拒掉让上层中止/重采, 绝不把它当真值往下传.
    const double concentration = std::hypot(sx, sy) / n;
    out_deg = 0.5 * std::atan2(sy, sx) * 180.0 / M_PI;
    if (concentration < axis_min_concentration_) {
      std::string dump;
      for (const auto & v : raw) dump += string_format("%+.1f ", v);
      RCLCPP_ERROR(logger_, "θ_img %d 帧不一致 (集中度 %.2f < %.2f, 疑长短轴互换), 原始帧: %s",
                   n, concentration, axis_min_concentration_, dump.c_str());
      return false;
    }
    return true;
  }

  static std::string string_format(const char * fmt, double v)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), fmt, v);
    return std::string(buf);
  }

  bool currentTcp(geometry_msgs::msg::PoseStamped & out)
  {
    // 直接查 TF base_link->suction_tip, 锁定 base_link 系 (getCurrentPose 返回的是
    // planning_frame 系, 可能是 world, 会与 object_pose / computeCartesianPath pose_ref 混系).
    geometry_msgs::msg::TransformStamped tf;
    if (!lookup(ee_link_, tf)) return false;
    out.header = tf.header;
    out.pose.position.x = tf.transform.translation.x;
    out.pose.position.y = tf.transform.translation.y;
    out.pose.position.z = tf.transform.translation.z;
    out.pose.orientation = tf.transform.rotation;
    return true;
  }

  bool lookup(const std::string & child, geometry_msgs::msg::TransformStamped & tf)
  {
    try {
      tf = tf_buffer_->lookupTransform(base_frame_, child, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(logger_, "TF %s->%s: %s", base_frame_.c_str(), child.c_str(), e.what());
      return false;
    }
  }

  void publishPump(int8_t v)
  {
    std_msgs::msg::Int8 m; m.data = v; pump_pub_->publish(m);
  }

  // 释放 = 开阀破真空一小下, 立刻关阀. 不能发完 RELEASE 就不管: PUMP_RELEASE 是持续通电
  // 开阀, 实测放置后阀一直开着十几分钟烫手 (放置返回后没人再动气泵, ready 若失败更是直接
  // 卡在开阀状态). 破负压只需要几百 ms, 之后 STOP 关泵关阀才是常态.
  void releasePulse()
  {
    publishPump(PUMP_RELEASE);
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(release_duration_ * 1000)));
    publishPump(PUMP_STOP);
    RCLCPP_INFO(logger_, "已开阀释放 %.1fs 后关阀 (电磁阀不留电)", release_duration_);
  }

  bool callTrigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli, const std::string & name)
  {
    if (!cli->wait_for_service(3s)) {
      RCLCPP_ERROR(logger_, "%s 服务不可用", name.c_str());
      return false;
    }
    // 20s 而非 5s: servo_node 首次 start_servo 要加载碰撞几何 (本机网格顶点多, 实测
    // 手工调服务瞬间返回, 但节点忙时 5s 会超时误判失败).
    auto fut = cli->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (fut.wait_for(20s) != std::future_status::ready) {
      RCLCPP_ERROR(logger_, "%s 无响应", name.c_str());
      return false;
    }
    return fut.get()->success;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::string planning_group_, ee_link_, base_frame_, tray_frame_, object_topic_, pump_topic_;
  std::string suction_body_link_;   // 吸盘本体(带网格), attach 时须与 ee_link_ 一同豁免
  double pregrasp_height_, lift_height_, tray_clearance_, transit_z_;
  double jog_dx_, jog_dy_, jog_dz_, jog_dyaw_deg_;
  double insert_stroke_min_, insert_stroke_max_, ground_z_, insert_shortfall_;
  double suck_duration_, release_duration_, insert_velocity_scaling_;
  double settle_sec_, settle_eps_, settle_poll_sec_;
  double place_x_, place_y_, place_z_, place_clearance_, level_height_;
  double unload_base_x_, unload_base_y_, unload_x_offset_, unload_z_;
  std::vector<double> unload_transit_candidates_;  // 卸货 transit 高度候选, 从低到高试
  std::vector<double> transit_candidates_;         // 装货(放托盘) transit 高度候选, 同上
  std::vector<double> unload_pick_shortfall_;    // 按 category_ids 顺序的取盒下压量
  // 卸到地面的盒 id (placed_unloaded_*), 只在真机整盘模式登记. 不进 placed_ids_(那是
  // 托盘的账), 单独一份供下一次 unloadTray 开头精确清理 —— 直接删自知清单, 不问
  // move_group "现在有哪些"(那条查询用的服务不稳定, 见 removeGroundBoxesViaService).
  std::vector<std::string> ground_placed_ids_;
  geometry_msgs::msg::Quaternion unload_quat_;   // 卸货点释放朝向 (实标, 非托盘朝向)
  double plan_velocity_scaling_, rotate_velocity_scaling_;

  // 双托盘放置 + 按类别堆叠
  int num_trays_{2};
  std::vector<double> tray_x_, tray_y_, tray_z_, tray_qx_, tray_qy_, tray_qz_, tray_qw_;
  std::vector<int64_t> tray_capacity_, category_ids_, category_tray_;
  double layer_gap_{0.005};   // 每层额外抬高量, 逐层累加 (两盘共用)
  std::vector<double> fallback_thickness_;
  bool dry_run_{false};
  double place_velocity_scaling_{1.0};
  std::string class_topic_, thickness_topic_, cam_point_topic_, cam_frame_;
  // 可抓候选数 (poses_mtx_ 保护): 只记长度与接收时刻, 供 /grasp/look 报给状态机.
  std::string poses_topic_;
  double look_count_wait_sec_{1.5};
  std::mutex poses_mtx_;
  int last_pick_count_{0};
  rclcpp::Time last_poses_stamp_{0, 0, RCL_ROS_TIME};
  // 吸盘朝向对齐 (走图像角, 不过手眼外参)
  std::string axis_angle_topic_;
  double yaw_target_theta_img_, yaw_align_tol_deg_;
  double yaw_probe_deg_, yaw_slope_prior_, yaw_slope_min_, yaw_step_gain_, yaw_max_step_deg_;
  double yaw_bad_theta_deg_, yaw_align_max_turn_deg_;
  double axis_flush_sec_, axis_min_concentration_;
  int axis_resample_tries_;
  int yaw_align_max_steps_, axis_avg_frames_;
  std::mutex axis_mtx_;
  double last_axis_angle_{0.0};
  rclcpp::Time last_axis_stamp_{0, 0, RCL_ROS_TIME};
  bool have_axis_angle_{false};
  double cam_target_y_, cam_target_z_, cam_tol_, refine_diverge_ratio_, coarse_yaw_;
  std::mutex cam_mtx_;
  geometry_msgs::msg::PointStamped last_cam_point_;
  bool have_cam_point_{false};
  int default_category_{1};
  // 运行时堆叠状态 (stack_mtx_ 保护)
  std::mutex stack_mtx_;
  std::vector<int> tray_layers_;
  std::vector<double> tray_stack_h_;
  // 下一个卸下的盒该去哪个卸货点: 落点 = unload_seq_ % num_unload_points_.
  // 跨服务调用保持, 使单个卸货(一按一个)不会把第二个盒摞到第一个身上; reset_stack 归零.
  //
  // **全局一个计数器, 不按托盘分** (2026-08-03 改): 两个卸货点是地面上的物理位置, 与盒
  // 从哪个托盘来无关. 原先 per-tray 时, L1 卸 1 号盘第一个和 L2 卸 0 号盘第一个各自 seq=0,
  // 会双双落到同一个点 —— 人若没及时取走第一个, 第二个直接摞上去, 而单卸模式已不登记
  // 碰撞体(见 unloadTray 里那条), 规划器也不会帮着避开。
  int unload_seq_{0};
  int num_unload_points_{2};
  // 已放盒的持久碰撞体 id, 按托盘分组 (stack_mtx_ 保护). 放置成功后留在规划场景, 使后续
  // 放另一盘时 MoveIt 自动避让, 不再横扫已放盒 (2026-07-28 蹭飞事故). detach 的是携带盒
  // (carried_box), 这些是"已落盘"的独立世界碰撞体, 生命周期到 reset_stack 才清.
  std::vector<std::vector<std::string>> placed_ids_;
  // 每个已放盒的 base_link 系**盒心**位姿 (与 id 一一对应), 供临时移出后原样加回.
  std::vector<std::vector<geometry_msgs::msg::Pose>> placed_poses_;
  // 每个已放盒放置时的**吸盘**释放位姿 + 厚度 (与 id 一一对应). 卸货(popLayer)原路取回用:
  // 取盒要把吸盘开回当初放它的那个位姿, 而 placed_poses_ 存的是盒心, 差半个盒高.
  std::vector<std::vector<geometry_msgs::msg::Pose>> placed_release_;
  std::vector<std::vector<double>> placed_th_;
  // 类别/厚度 B 路线缓存 (cls_mtx_ 保护)
  std::mutex cls_mtx_;
  int last_category_{1};
  double last_thickness_{0.0};
  bool have_category_{false}, have_thickness_{false};
  // "吸盘上这个盒"的类别: ③下插时锁存, detachBox 时清. 见 placeCategory().
  int held_category_{1};
  bool have_held_category_{false};
  double box_size_x_, box_size_y_, box_size_z_;
  std::string j1_name_;
  double look_j1_offset_;
  double probe_step_, refine_step_gain_, refine_max_step_, refine_velocity_scaling_;
  int refine_max_steps_, cam_avg_frames_;
  double cam_wait_timeout_, refine_timeout_, object_stale_sec_;
  double object_flush_sec_, object_consist_tol_;
  int object_consist_frames_, object_resample_tries_;

  static constexpr const char * kCarriedBoxId = "carried_box";

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poses_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr class_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thickness_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr cam_point_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr axis_angle_sub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pump_pub_;
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr co_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_, unload_srv_, ready_srv_, look_srv_,
    pick_srv_, place_srv_, coarse_srv_, cam_cal_srv_, yaw_cal_srv_, reset_srv_, jog_srv_,
    seed_srv_, unload_tray_srv_, unload_one_srv_, home_srv_, level_srv_, place_ground_srv_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_cli_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_cli_;
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_, scene_cb_group_, perc_cb_group_;

  std::mutex obj_mtx_;
  geometry_msgs::msg::PoseStamped last_object_;
  bool have_object_{false};
};

}  // namespace mm_grasp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "grasp_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto grasp = std::make_shared<mm_grasp::GraspNode>(node);
  grasp->initMoveGroup();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
