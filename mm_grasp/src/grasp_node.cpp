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
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.hpp>
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
    // 高度一起算错. 吸盘有海绵/波纹, 差 2mm 靠密封唇自己贴上, 少压一点减轻压盒和顶臂.
    insert_shortfall_ = node_->declare_parameter<double>("insert_shortfall", 0.002);
    // 吸取抽真空时长(秒), 到时转 PUMP_STOP 保压. 1s 足够: 实测 3s 里负压早已建立,
    // 吸住后继续抽没有收益, 只是让泵空转发热.
    suck_duration_ = node_->declare_parameter<double>("suck_duration", 1.0);
    // 释放开阀时长(秒), 到时立刻 PUMP_STOP 关阀. 见 releasePulse(): 阀持续通电会发烫.
    // 3s: 实测 0.5/1/1.5s 泄气都不净, 盒子仍粘在吸盘上. 吸盘腔体加气管容积不小, 破真空
    // 比想象的慢. 3s 离"通电几分钟发烫"仍差两个数量级, 拿时长换可靠脱开.
    release_duration_ = node_->declare_parameter<double>("release_duration", 3.0);
    // 末段下插降速倍率: 实测下降段抖动. 降速让每个轨迹点的关节增量变小, 抖动幅度随之变小.
    // 这是压制不是根治(根因待查: 疑在笛卡尔路径的时间参数化/该构型雅可比条件数).
    insert_velocity_scaling_ = node_->declare_parameter<double>("insert_velocity_scaling", 0.3);
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

    // ---- 吸盘朝向对齐: 走图像角 θ_img, 不过手眼外参 ----
    // 视觉发的 base_link 系盒 yaw = R @ 相机系长轴, R(base_link<-Link_30) 含手眼外参旋转
    // 与 FK, 随构型漂: 2026-07-27 实测检测报盒 +36.2°(腕在 coarse_yaw -90° 时), 按它转到
    // 腕 -143.7° 后人工再修 6° 到 -149.7° 才与盒边贴合. 近正方形盒斜 6° 进带围栏托盘会蹭卡.
    // 相机固连腕(Link_30 挂 Link_29), 所以"吸盘长轴在图像里占多少度"是与构型无关的常数,
    // 实测标定一对 (θ_ref, ψ_ref) 即可: ψ = ψ_ref + s·(θ_ref − θ_img). 整条链只过检测.
    axis_angle_topic_ = getOrDeclare<std::string>(
      "axis_angle_topic", "/perception/object_axis_angle");
    // 标定常数(度): 盒轴与吸盘轴贴合时, 图像角 θ_ref 与当时的腕 yaw ψ_ref.
    // 用 /grasp/calib_yaw_ref 读取填入. 缺省值来自 2026-07-27 首次标定.
    yaw_ref_theta_img_ = getOrDeclare<double>("yaw_ref_theta_img", -37.6);
    yaw_ref_tool_ = getOrDeclare<double>("yaw_ref_tool", -149.7);
    // 符号: 腕绕竖直轴转 Δ, 图像里长轴角转 −Δ (J6 轴 (0,0,-1) 且近竖直, 实测腕每 +2°
    // 吸盘 yaw 精确 −2°). 装反/换相机安装朝向则改 +1.
    yaw_axis_sign_ = getOrDeclare<double>("yaw_axis_sign", -1.0);
    // 近正方形盒的类别: 这些类别转 90° 后占位几乎不变, 故按 90° 等价折叠就近取腕转角
    // (省掉无谓大角度转动). 其余类别是长方形, 转 90° 长短边互换会顶到围栏, 只能按 180° 折叠.
    square_categories_ = getOrDeclare<std::vector<int64_t>>("square_categories", {1});

    class_topic_ = getOrDeclare<std::string>("class_topic", "/perception/object_class");
    thickness_topic_ = getOrDeclare<std::string>("thickness_topic", "/perception/object_thickness");
    default_category_ = static_cast<int>(getOrDeclare<int64_t>("default_category", 1));

    // 运行时堆叠状态: 每托盘已放盒数 + 累计厚度. reset 服务清零(新一轮).
    tray_layers_.assign(num_trays_, 0);
    tray_stack_h_.assign(num_trays_, 0.0);

    // 卸货目的地 (base_link 系, top-down, 写死车右侧地面): 从托盘取盒后, 先到上方,
    // 再笛卡尔直下到吸盘末端 z=place_z_ (盒底离地 ~5mm) 才释放, 盒子落稳而非半空抛.
    // place_x/y 写死车右侧 (base_link -y = 车右); place_z_ = 释放时吸盘末端高度:
    // 盒高 0.025, 盒底离地 5mm -> 末端 = 0.005 + 0.025 = 0.030.
    place_x_ = node_->declare_parameter<double>("place_x", 0.0);
    place_y_ = node_->declare_parameter<double>("place_y", -0.38);
    place_z_ = node_->declare_parameter<double>("place_z", 0.030);
    place_clearance_ = node_->declare_parameter<double>("place_clearance", 0.12);

    // 被抓盒子尺寸 (world grasp_box: 0.09x0.055x0.025), 吸取后 attach 到吸盘作碰撞体,
    // 让放置规划知道吸盘下挂着盒 -> 绕开托盘边框, 不再侧向蹭入。
    box_size_x_ = node_->declare_parameter<double>("box_size_x", 0.09);
    box_size_y_ = node_->declare_parameter<double>("box_size_y", 0.055);
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
    refine_max_step_ = node_->declare_parameter<double>("refine_max_step", 0.04);
    // 步进段规划降速. 0.15 是最初怕抖留的余量; 实测步进本身自带加减速, 不抖, 提到 0.4
    // 与全局同速, 精修那几步的起停停顿明显变短.
    refine_velocity_scaling_ = node_->declare_parameter<double>("refine_velocity_scaling", 0.4);
    // 每次取位置前平均几帧检测: 单帧 p_cam 横向有 1~2mm 噪声, 不平均则 6mm 阈值会被噪声
    // 反复触发又取消.
    cam_avg_frames_ = node_->declare_parameter<int>("cam_avg_frames", 3);
    cam_wait_timeout_ = node_->declare_parameter<double>("cam_wait_timeout", 3.0);
    refine_timeout_ = node_->declare_parameter<double>("refine_timeout", 60.0);
    object_stale_sec_ = node_->declare_parameter<double>("object_stale_sec", 0.5);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    object_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      object_topic_, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(obj_mtx_);
        last_object_ = *msg;
        have_object_ = true;
      });

    // B 路线: 队友视觉侧额外发的"当前目标类别 + 厚度(米)". 回调只缓存, 放置时取最新.
    // 收不到则 execute 用 default_category_ + fallback_thickness_ 兜底, 不阻塞抓取.
    class_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      class_topic_, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Int32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(cls_mtx_);
        last_category_ = msg->data;
        have_category_ = true;
      });
    cam_point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      cam_point_topic_, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(cam_mtx_);
        last_cam_point_ = *msg;
        have_cam_point_ = true;
      });
    // θ_img: OBB 长轴在彩色图像里的角度(度). 只在 ① 之前(腕仍在 coarse_yaw_、相机没被
    // 吸盘遮挡)采样一次; ② 之后臂已贴近, 吸盘悬在盒上方挡住视野, 此时的检测不可信.
    axis_angle_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      axis_angle_topic_, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(axis_mtx_);
        last_axis_angle_ = msg->data;
        last_axis_stamp_ = node_->now();
        have_axis_angle_ = true;
      });
    thickness_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      thickness_topic_, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(cls_mtx_);
        last_thickness_ = msg->data;
        have_thickness_ = true;
      });

    pump_pub_ = node_->create_publisher<std_msgs::msg::Int8>(pump_topic_, 10);

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
    // 看货姿势 (mm_task 抓取前 S3 调): ready + J1+90°, 相机转向货物再做闭环抓取.
    look_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/look",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = moveToLook();
        res->message = res->success ? "arm at look pose" : "move to look failed";
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
        attachBox(true);
        if (!placeAtPose(pt.pose, pt.what.c_str())) {
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
        pushLayer(pt.tray, pt.thickness);
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
    // 标定吸盘朝向基准: 人工把吸盘长轴转到与盒长轴贴合后调一次, 读回当时的
    // (θ_img, 腕 yaw) 就是 (yaw_ref_theta_img, yaw_ref_tool). 只读不动臂.
    // 标定时腕必须在 coarse_yaw_ 附近且相机能看清盒(吸盘别压在盒上), 否则 θ_img 不可信.
    yaw_cal_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/grasp/calib_yaw_ref",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        double theta = 0.0;
        if (!latestAxisAngle(theta)) {
          res->success = false; res->message = "无新鲜 θ_img"; return;
        }
        geometry_msgs::msg::PoseStamped tcp;
        if (!currentTcp(tcp)) {
          res->success = false; res->message = "取 TCP 位姿失败"; return;
        }
        char buf[192];
        std::snprintf(buf, sizeof(buf),
                      "yaw_ref_theta_img: %.1f  yaw_ref_tool: %.1f",
                      theta, quatYaw(tcp.pose.orientation) * 180.0 / M_PI);
        RCLCPP_WARN(logger_, "标定吸盘朝向基准 -> 填进 place.yaml: %s", buf);
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
        RCLCPP_INFO(logger_, "堆叠计数已清零 (%d 托盘)", num_trays_);
        res->success = true; res->message = "stack counters reset";
      },
      rmw_qos_profile_services_default, srv_cb_group_);

    RCLCPP_INFO(logger_,
                "grasp_node 就绪: group=%s ee=%s base=%s 订 %s 发 %s, "
                "服务 /grasp/execute /grasp/unload /grasp/ready /grasp/look",
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
    const int category = currentCategory();
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
    if (trayLayers(tray) >= static_cast<int>(tray_capacity_[tray])) {
      err = std::to_string(tray) + "号托盘已满(" +
            std::to_string(tray_capacity_[tray]) + ")";
      RCLCPP_ERROR(logger_, "%s", err.c_str());
      return false;
    }

    out.tray = tray;
    out.thickness = thickness;
    // release z = 托盘空载接触 z + 累计下层厚度 + 本层厚度 (吸盘吸盒顶, 盒底落下层顶).
    out.pose = trayContactPose(tray);
    out.pose.position.z = tray_z_[tray] + trayStackH(tray) + thickness;
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
      attachBox(true);
      bool ok = placeAtPose(target, what);
      detachBox();
      res->success = ok;
      res->message = ok ? std::string("[dry_run] 放置轨迹规划通过: ") + what
                        : std::string("[dry_run] 放置轨迹规划失败: ") + what;
      RCLCPP_WARN(logger_, "==== [dry_run] 一轮结束 (%s), 不计堆叠 ====", what);
      return;
    }

    std::string err;
    // 盒 attach 后豁免与托盘接触: 放到位时盒底本就落在托盘面(标定 tray_z 比 Link_11 网格
    // 顶面低几 mm), 不豁免则直下段被判碰撞截断. 侧蹭边框的顾虑已由"正上方->垂直直下"
    // 的入位方式消除, 不再靠碰撞检测拦.
    if (!pickCycle(err, true)) { res->success = false; res->message = err; return; }

    if (!placeAtPose(target, what)) {
      res->success = false; res->message = std::string("放置失败(已吸取): ") + what; return;
    }
    // 放置成功才计入堆叠: 层数+1, 累计厚度 += 本层厚度.
    pushLayer(tray, thickness);

    if (!moveToReady()) {
      res->success = false; res->message = "放置后回 ready 失败"; return;
    }

    RCLCPP_INFO(logger_, "==== 抓放一轮完成 (%s, 累计高 %.1fmm) ====",
                what, trayStackH(tray) * 1000);
    res->success = true; res->message = std::string("grasp cycle done: ") + what;
  }

  // ---- 堆叠状态 / 类别映射 工具 ----
  int currentCategory()
  {
    std::lock_guard<std::mutex> lk(cls_mtx_);
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

  void pushLayer(int t, double thickness)
  {
    std::lock_guard<std::mutex> lk(stack_mtx_);
    tray_layers_[t] += 1;
    tray_stack_h_[t] += thickness;
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
    // θ_img 必须在这里锁: 此刻腕仍在 look 姿态, 相机俯视盒子无遮挡, 是唯一干净的观测时机.
    // ①/②走完后吸盘悬在盒正上方 12cm, 盒被吸盘遮挡, 检测退化 (转 yaw 后重测曾报出
    // 偏 50° 的假数). 用图像角而非 obj 里的 base_link 盒 yaw, 是为绕开手眼外参旋转.
    double theta_img = 0.0;
    if (!latestAxisAngle(theta_img)) { err = "无新鲜 θ_img (长轴图像角)"; return false; }
    const double tool_yaw = toolYawFromImageAngle(theta_img);
    RCLCPP_INFO(logger_,
                "锁定朝向: θ_img=%+.1f° -> 吸盘目标 yaw=%+.1f° "
                "(标定 θ_ref=%+.1f°/ψ_ref=%+.1f°, s=%+.0f)",
                theta_img, tool_yaw * 180.0 / M_PI,
                yaw_ref_theta_img_, yaw_ref_tool_, yaw_axis_sign_);
    if (!stageCoarse(obj)) { err = "① 粗定位失败"; return false; }
    if (!stageRefine())    { err = "② 精修失败"; return false; }
    // ②之后再转 yaw (吸取前): 吸盘轴先与盒轴对齐, 吸起来盒在吸盘上的相对朝向就是 0,
    // 放置时只需把吸盘转到托盘标定朝向, 不用再算"盒相对吸盘"那一层.
    if (!stageRotateYaw(tool_yaw, theta_img)) { err = "转 yaw 失败"; return false; }
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

  // ②之后: 原地把吸盘转到"与盒长轴对齐"的朝向 tool_yaw, 为放置做准备
  // (放置要求盒子摆正; 吸本身不需要 yaw). tool_yaw 由 θ_img 经标定常数解出, 见
  // toolYawFromImageAngle —— 不过手眼外参, 所以不随构型漂.
  // 位置目标沿用当前实测 TCP xyz, 只换朝向 —— 绕吸盘自身轴转, 名义上盒心位置不变.
  // 转完打印 TCP xy 漂移: 腕关节轴与吸盘轴未必严格共线, 漂多少要用实测说话, 不靠假设.
  bool stageRotateYaw(double tool_yaw, double theta_img)
  {
    geometry_msgs::msg::PoseStamped before;
    if (!currentTcp(before)) { RCLCPP_ERROR(logger_, "转 yaw: 取当前位姿失败"); return false; }

    move_group_->setMaxVelocityScalingFactor(rotate_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(rotate_velocity_scaling_);

    geometry_msgs::msg::Pose target = before.pose;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = false;
    // 等价分支就近优先: 近正方形盒(square_categories)按 90° 折, 长方形盒只能按 180° 折.
    // 就近那个可能落在腕限位外或规划失败, 所以其余分支留作兜底.
    for (const auto & q : yawEquivalents(yawToQuat(tool_yaw))) {
      target.orientation = q;
      move_group_->setStartStateToCurrentState();
      move_group_->setPoseTarget(target);
      if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger_, "转 yaw 规划成功 (解出 %+.1f° -> 就近等价 %+.1f°)",
                    tool_yaw * 180.0 / M_PI, quatYaw(q) * 180.0 / M_PI);
        planned = true;
        break;
      }
      RCLCPP_WARN(logger_, "转 yaw %+.1f° 规划失败, 试等价分支", quatYaw(q) * 180.0 / M_PI);
    }
    if (!planned) {
      RCLCPP_ERROR(logger_, "转 yaw: 所有等价分支都规划失败");
      restorePlanScaling(); return false;
    }
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      restorePlanScaling(); return false;
    }
    settle();
    restorePlanScaling();

    geometry_msgs::msg::PoseStamped after;
    if (!currentTcp(after)) return false;
    const double dx = after.pose.position.x - before.pose.position.x;
    const double dy = after.pose.position.y - before.pose.position.y;
    RCLCPP_INFO(logger_, "转 yaw 后 TCP xy 漂移 (%+.1f,%+.1f)mm |d|=%.1fmm",
                dx * 1000, dy * 1000, std::hypot(dx, dy) * 1000);

    reportYawResidual(theta_img, tool_yaw, quatYaw(after.pose.orientation));
    return true;
  }

  // 转完 yaw 后的自检: 只比"指令 vs 实到", 不重测盒子.
  // 为什么不重测: 此刻吸盘悬在盒正上方 12cm, 盒被吸盘遮挡、常被挤到画面边缘, 这时的检测
  // 是坏视角 —— 2026-07-27 实测据此算出 +50.6° 的假残差, 而人工量的真实偏差只有 6°.
  // 真正的对齐残差无法在机上闭环量到(要量就得再看一眼被遮住的盒), 它由标定常数
  // (yaw_ref_theta_img/yaw_ref_tool) 的准确度决定, 靠"盒能否干净落进围栏"离线校.
  // 折到 (-90,90] 比较: 长轴无向, 差 180° 是同一朝向.
  void reportYawResidual(double theta_img, double tool_yaw_cmd, double tool_yaw_now)
  {
    const double track = wrapAngle90(tool_yaw_now - tool_yaw_cmd);
    RCLCPP_INFO(logger_,
                "转 yaw 自检: θ_img=%+.1f° -> 指令 %+.1f°, 实到 %+.1f° (跟随误差 %+.1f°). "
                "对齐残差不在此测(吸盘遮挡盒子), 由标定常数决定",
                theta_img, tool_yaw_cmd * 180.0 / M_PI,
                tool_yaw_now * 180.0 / M_PI, track * 180.0 / M_PI);
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

  // 目标姿态的等价分支, 按"离当前吸盘朝向近"排序返回.
  // 折叠步长按盒形状分: 近正方形盒(square_categories, 现只有类别1)转 90° 后占位几乎不变,
  // 所以 4 个分支都等价, 就近取能省掉大角度转动; 其余类别是长方形, 转 90° 长短边互换,
  // 进带围栏托盘会顶到边框, 只有 180° 才是真等价.
  // 排序的意义是省掉无意义的转动 —— θ_img 折到 (-90°,90°], 盒摆在边界附近时检测值在
  // +88°/-88° 间反复翻, 不排序会为 176° 的名义差白转半圈.
  std::vector<geometry_msgs::msg::Quaternion> yawEquivalents(
    const geometry_msgs::msg::Quaternion & target)
  {
    const bool square = isSquareCategory(currentCategory());
    const double step = square ? M_PI / 2.0 : M_PI;
    const int n = square ? 4 : 2;

    std::vector<geometry_msgs::msg::Quaternion> out;
    out.reserve(n);
    for (int k = 0; k < n; ++k) out.push_back(rotateAboutBaseZ(target, step * k));

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

  bool isSquareCategory(int category) const
  {
    for (const auto & c : square_categories_) {
      if (static_cast<int>(c) == category) return true;
    }
    return false;
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
  bool moveRelativeXY(double dx, double dy)
  {
    if (std::hypot(dx, dy) < 1e-4) return true;
    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) return false;
    geometry_msgs::msg::Pose wp = cur.pose;
    wp.position.x += dx;
    wp.position.y += dy;
    std::vector<geometry_msgs::msg::Pose> wps{wp};
    moveit_msgs::msg::RobotTrajectory traj;
    move_group_->setStartStateToCurrentState();
    const double frac = move_group_->computeCartesianPath(wps, 0.002, 0.0, traj);
    if (frac < 0.9) {
      RCLCPP_ERROR(logger_, "② 水平步进路径覆盖不足 %.0f%%", frac * 100);
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
    const int ci = categoryIndex(currentCategory());
    const double thickness = currentThickness(ci);
    const double box_top_z = ground_z_ + thickness;

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
    attachBox(allow_tray_touch);  // 盒进规划场景: 后续放置绕开托盘边框
    return true;
  }

  // 把盒子作为 attached collision object 挂到吸盘: 吸取后吸盘末端在盒顶, 盒心在吸盘
  // -Z(工具系向下)方向 0.0125 处. 此后 MoveIt 规划放置路径会考虑这块几何, 从上方入托盘
  // 而非侧向蹭过边框. touch_links 默认只放吸盘 link (吸取处接触不误报);
  // allow_tray_touch=true 时再把托盘 link 加进 touch_links —— 盒与 Link_11 的接触是预期的:
  // 卸货取盒时盒本在托盘里几何重叠, 放盒到位时盒底落在托盘面. 不豁免则规划器判碰撞,
  // 取盒连抬起都规划不了, 放盒直下段中途被截断.
  void attachBox(bool allow_tray_touch)
  {
    moveit_msgs::msg::CollisionObject co;
    co.id = kCarriedBoxId;
    co.header.frame_id = ee_link_;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions = {box_size_x_, box_size_y_, box_size_z_};
    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;
    p.position.z = -box_size_z_ / 2.0;   // 盒心在吸盘末端下方半个盒高
    co.primitives.push_back(prim);
    co.primitive_poses.push_back(p);
    co.operation = co.ADD;

    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = ee_link_;
    aco.object = co;
    aco.touch_links = {ee_link_};
    if (allow_tray_touch) aco.touch_links.push_back(tray_frame_);
    psi_->applyAttachedCollisionObject(aco);
    RCLCPP_INFO(logger_, "盒子已 attach 到 %s 作碰撞体 (%.0fx%.0fx%.0fmm)%s", ee_link_.c_str(),
                box_size_x_ * 1000, box_size_y_ * 1000, box_size_z_ * 1000,
                allow_tray_touch ? " [豁免与托盘接触]" : "");
  }

  // 释放盒子后从规划场景摘除: detach + 移除, 以免残留碰撞体挡住回 ready 的规划.
  void detachBox()
  {
    move_group_->detachObject(kCarriedBoxId);
    psi_->removeCollisionObjects({kCarriedBoxId});
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
    RCLCPP_INFO(logger_, "已回 ready 位");
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
    RCLCPP_INFO(logger_, "已到看货姿势 (ready %s%+.0f°)", j1_name_.c_str(),
                look_j1_offset_ * 180.0 / M_PI);
    return true;
  }

  // 放置到地面: 相对抬起(携盒) -> 到目的地上方(top-down) -> 笛卡尔直下到吸盘 z_tip
  // (盒底离地 ~5mm) -> 释放 + detach. 不在半空释放, 盒子落稳. 卸货用.
  bool placeAt(double x, double y, double z_tip, double clearance)
  {
    // 抬起 (相对当前 +Z, 携盒)
    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) return false;
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

  // 放到"标定好的绝对目标位姿"(含朝向), 顶向下入位: 相对抬起(携盒) -> 规划到目标正上方
  // (标定 xy+朝向, z 抬 tray_clearance_) -> 笛卡尔垂直直下到标定位姿 -> 释放 + detach.
  // 放托盘用: 托盘在肩后死区, 唯一可达是工具 yaw≈180°朝下的特定位姿(RViz 实测标定).
  // 早前直接 plan() 到标定位姿: 盒 attach 后规划器为让盒全程避开边框会绕大圈甩臂; 改成
  // 顶上空旷处入、再垂直直下, 盒竖直进托盘不蹭边框, 规划器无需绕路, 路径直不甩.
  bool placeAtPose(const geometry_msgs::msg::Pose & target, const char * what)
  {
    // 四步渐进安全测试: 放置段整体降速(默认 0.2, place_velocity_scaling 再乘一档),
    // 收尾恢复. dry_run 时只 plan+打印, 不 execute, 也不动气泵/detach (盒仍吸着不释放).
    const double v = plan_velocity_scaling_ * place_velocity_scaling_;
    move_group_->setMaxVelocityScalingFactor(v);
    move_group_->setMaxAccelerationScalingFactor(v);
    if (dry_run_) {
      RCLCPP_WARN(logger_, "放置(%s): [dry_run] 只规划打印, 不执行/不释放", what);
    }

    // 携盒相对抬起, 先让盒离开当前接触面
    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) { restorePlanScaling(); return false; }
    geometry_msgs::msg::Pose up = cur.pose;
    up.position.z += lift_height_;
    std::vector<geometry_msgs::msg::Pose> wps{up};
    moveit_msgs::msg::RobotTrajectory traj;
    move_group_->setStartStateToCurrentState();
    const double up_frac = move_group_->computeCartesianPath(wps, 0.005, 0.0, traj);
    if (up_frac > 0.5 && !dry_run_) {
      move_group_->execute(traj);
      settle();
      RCLCPP_INFO(logger_, "放置(%s): 已抬起 %.0fcm", what, lift_height_ * 100);
    } else if (dry_run_) {
      RCLCPP_WARN(logger_, "放置(%s): [dry_run] 抬起路径覆盖 %.0f%%, 不执行",
                  what, up_frac * 100);
    }

    // 规划到目标正上方 (xy+朝向用标定值, z 抬 tray_clearance_)
    geometry_msgs::msg::Pose above = target;
    above.position.z = target.position.z + tray_clearance_;
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(above);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "放置(%s): 规划到正上方 (%.3f,%.3f,%.3f) 失败", what,
                   above.position.x, above.position.y, above.position.z);
      restorePlanScaling(); return false;
    }
    RCLCPP_INFO(logger_, "放置(%s): 规划到正上方 (%.3f,%.3f,%.3f) 成功, 轨迹 %zu 点", what,
                above.position.x, above.position.y, above.position.z,
                plan.trajectory_.joint_trajectory.points.size());
    if (!dry_run_) {
      if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        restorePlanScaling(); return false;
      }
      settle();
    }

    // 笛卡尔垂直直下到标定放置位姿: 盒竖直入托盘, 不侧向蹭边框
    std::vector<geometry_msgs::msg::Pose> dwps{target};
    moveit_msgs::msg::RobotTrajectory dtraj;
    // dry_run 下上面两段都没真执行, 臂还停在原处; 直下段起点若仍取实测当前状态, 等于要求
    // 从原处一步笛卡尔跨到托盘正上方, 覆盖必然 0%. 故接上一段规划的终点作起点.
    if (dry_run_) {
      moveit::core::RobotState start(*move_group_->getCurrentState());
      const auto & jt = plan.trajectory_.joint_trajectory;
      start.setVariablePositions(jt.joint_names, jt.points.back().positions);
      move_group_->setStartState(start);
    } else {
      move_group_->setStartStateToCurrentState();
    }
    const double frac = move_group_->computeCartesianPath(dwps, 0.005, 0.0, dtraj);
    if (frac < 0.9) {
      RCLCPP_ERROR(logger_, "放置(%s): 垂直直下路径覆盖不足 %.0f%%", what, frac * 100);
      restorePlanScaling(); return false;
    }
    if (dry_run_) {
      RCLCPP_WARN(logger_, "放置(%s): [dry_run] 直下路径覆盖 %.0f%%, 目标 z=%.3f, 到此为止不执行",
                  what, frac * 100, target.position.z);
      restorePlanScaling();
      return true;   // dry_run 视为通过(规划全成功), 但不真放/不计堆叠由调用侧另判
    }
    if (move_group_->execute(dtraj) != moveit::core::MoveItErrorCode::SUCCESS) {
      restorePlanScaling(); return false;
    }
    settle();

    releasePulse();
    detachBox();
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
      move_group_->getCurrentState(0.1)->copyJointGroupPositions(planning_group_, cur);
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
  bool waitObject(geometry_msgs::msg::PoseStamped & out, double timeout_s)
  {
    const rclcpp::Time t0 = node_->now();
    rclcpp::WallRate r(20.0);
    while (rclcpp::ok() && (node_->now() - t0).seconds() < timeout_s) {
      if (latestObject(out)) return true;
      r.sleep();
    }
    return false;
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

  // θ_img(度) -> 吸盘目标 yaw(弧度). ψ = ψ_ref + s·(θ_ref − θ_img), 三个常数实测标定,
  // 不含手眼外参与 FK, 故不随构型漂 (见 axis_angle_topic_ 声明处注释).
  double toolYawFromImageAngle(double theta_img_deg)
  {
    const double d = yaw_axis_sign_ * (yaw_ref_theta_img_ - theta_img_deg);
    return wrapAngle((yaw_ref_tool_ + d) * M_PI / 180.0);
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
  double pregrasp_height_, lift_height_, tray_clearance_;
  double insert_stroke_min_, insert_stroke_max_, ground_z_, insert_shortfall_;
  double suck_duration_, release_duration_, insert_velocity_scaling_;
  double settle_sec_, settle_eps_, settle_poll_sec_;
  double place_x_, place_y_, place_z_, place_clearance_;
  double plan_velocity_scaling_, rotate_velocity_scaling_;

  // 双托盘放置 + 按类别堆叠
  int num_trays_{2};
  std::vector<double> tray_x_, tray_y_, tray_z_, tray_qx_, tray_qy_, tray_qz_, tray_qw_;
  std::vector<int64_t> tray_capacity_, category_ids_, category_tray_;
  std::vector<double> fallback_thickness_;
  bool dry_run_{false};
  double place_velocity_scaling_{1.0};
  std::string class_topic_, thickness_topic_, cam_point_topic_, cam_frame_;
  // 吸盘朝向对齐 (走图像角, 不过手眼外参)
  std::string axis_angle_topic_;
  double yaw_ref_theta_img_, yaw_ref_tool_, yaw_axis_sign_;
  std::vector<int64_t> square_categories_;
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
  // 类别/厚度 B 路线缓存 (cls_mtx_ 保护)
  std::mutex cls_mtx_;
  int last_category_{1};
  double last_thickness_{0.0};
  bool have_category_{false}, have_thickness_{false};
  double box_size_x_, box_size_y_, box_size_z_;
  std::string j1_name_;
  double look_j1_offset_;
  double probe_step_, refine_step_gain_, refine_max_step_, refine_velocity_scaling_;
  int refine_max_steps_, cam_avg_frames_;
  double cam_wait_timeout_, refine_timeout_, object_stale_sec_;

  static constexpr const char * kCarriedBoxId = "carried_box";

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr class_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thickness_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr cam_point_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr axis_angle_sub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pump_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_, unload_srv_, ready_srv_, look_srv_,
    pick_srv_, place_srv_, coarse_srv_, cam_cal_srv_, yaw_cal_srv_, reset_srv_;
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;

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
