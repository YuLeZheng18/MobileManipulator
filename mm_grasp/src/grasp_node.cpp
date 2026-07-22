// mm_grasp / grasp_node
// S4 三段抓取执行器 (被 mm_task 状态机调用, 本身不是状态机):
//   ① 粗定位 (闭环, MoveIt 规划): setPoseTarget suction_tip 到盒上方预抓取位
//   ② 精修   (闭环, moveit_servo): 读新鲜 object_pose, TwistStamped 视觉伺服对准并逼近
//   ③ 末段   (开环, 相对直插): 沿当前 suction_tip -Z 相对下插固定行程 + 气泵吸
//   放置: 相对抬起 -> 规划到 tray(Link_11) 上方 -> 释放
// 纪律(§5): 末段严禁用 FK 重算盒子 base_link 绝对坐标再 setPoseTarget.
//           末段 waypoint 只由"当前实测 suction_tip 位姿 + 固定 -Z 偏置"得来.
//
// 所有 MoveGroup 位姿运算统一在 base_link 系 (setPoseReferenceFrame), 与 object_pose /
// servo twist 一致, 从而与 move_group 内部 planning_frame 归属解耦.
//
// M4: std_srvs/Trigger 服务 /grasp/execute 触发一整轮.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    refine_height_ = node_->declare_parameter<double>("refine_height", 0.06);
    insert_stroke_ = node_->declare_parameter<double>("insert_stroke", 0.06);
    lift_height_ = node_->declare_parameter<double>("lift_height", 0.10);
    // 放托盘"正上方"抬高量: 到此高度盒远离托盘边框, 规划器不绕圈, 再垂直直下入位.
    tray_clearance_ = node_->declare_parameter<double>("tray_clearance", 0.06);

    // 放托盘目标位姿 (base_link 系): 直接用 RViz 规划到"盒底贴托盘中心"时实测的
    // suction_tip 位姿, 而非 Link_11 原点(那只是 URDF 占位, 非实际放盒点). 此后方位置
    // 可达的关键是工具 yaw≈180°朝下(非单位姿态), 故位姿含朝向一并标定. 可 GUI 复标覆盖.
    // z 由实测"盒底贴托盘底"的 0.102 抬 4mm -> 0.106: 盒子进碰撞后, 贴合接触态目标会被
    // 判 goal-in-collision 规划失败; 抬 4mm 让盒底离托盘底一点点, 规划得过, 释放后轻落。
    tray_place_x_ = node_->declare_parameter<double>("tray_place_x", -0.234);
    tray_place_y_ = node_->declare_parameter<double>("tray_place_y", -0.054);
    tray_place_z_ = node_->declare_parameter<double>("tray_place_z", 0.106);
    tray_place_qx_ = node_->declare_parameter<double>("tray_place_qx", 0.009);
    tray_place_qy_ = node_->declare_parameter<double>("tray_place_qy", 0.0);
    tray_place_qz_ = node_->declare_parameter<double>("tray_place_qz", 1.0);
    tray_place_qw_ = node_->declare_parameter<double>("tray_place_qw", 0.0);

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

    // xy_tol 放宽到 2cm: 下探+前伸构型下水平雅可比病态, servo 的 xy twist 近乎无力,
    // 粗定位 ~1.3cm 落位残差精修削不掉. z(0.005)/yaw(0.02) 仍紧收敛(可控且决定成败),
    // 吸盘朝下姿态不动; mock_suction 5cm 内即吸附, 2cm 水平偏置物理上吸得住.
    xy_tol_ = node_->declare_parameter<double>("xy_tol", 0.02);
    z_tol_ = node_->declare_parameter<double>("z_tol", 0.005);
    yaw_tol_ = node_->declare_parameter<double>("yaw_tol", 0.02);
    kp_lin_ = node_->declare_parameter<double>("kp_lin", 1.5);
    kp_ang_ = node_->declare_parameter<double>("kp_ang", 1.5);
    max_lin_vel_ = node_->declare_parameter<double>("max_lin_vel", 0.05);
    max_ang_vel_ = node_->declare_parameter<double>("max_ang_vel", 0.3);
    servo_rate_ = node_->declare_parameter<double>("servo_rate", 30.0);
    refine_timeout_ = node_->declare_parameter<double>("refine_timeout", 20.0);
    object_stale_sec_ = node_->declare_parameter<double>("object_stale_sec", 0.5);
    converge_cycles_ = node_->declare_parameter<int>("converge_cycles", 5);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    object_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      object_topic_, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(obj_mtx_);
        last_object_ = *msg;
        have_object_ = true;
      });

    pump_pub_ = node_->create_publisher<std_msgs::msg::Int8>(pump_topic_, 10);
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);

    start_servo_cli_ = node_->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    stop_servo_cli_ = node_->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");

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
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);
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
  // ---- 主流程 ----
  // /grasp/execute: 三段抓取源盒 -> 放到自己的托盘(Link_11).
  void onExecute(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    RCLCPP_INFO(logger_, "==== /grasp/execute: 抓源盒 -> 放托盘 ====");

    std::string err;
    // 放托盘: 盒 attach 不允许碰托盘(否则又蹭边框)
    if (!pickCycle(err, false)) { res->success = false; res->message = err; return; }

    geometry_msgs::msg::Pose tray;
    tray.position.x = tray_place_x_;
    tray.position.y = tray_place_y_;
    tray.position.z = tray_place_z_;
    tray.orientation.x = tray_place_qx_;
    tray.orientation.y = tray_place_qy_;
    tray.orientation.z = tray_place_qz_;
    tray.orientation.w = tray_place_qw_;
    if (!placeAtPose(tray, "托盘")) {
      res->success = false; res->message = "放托盘失败(已吸取)"; return;
    }
    if (!moveToReady()) {
      res->success = false; res->message = "放托盘后回 ready 失败"; return;
    }

    RCLCPP_INFO(logger_, "==== 抓放一轮完成 ====");
    res->success = true; res->message = "grasp cycle done";
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
  bool pickCycle(std::string & err, bool allow_tray_touch)
  {
    geometry_msgs::msg::PoseStamped obj;
    if (!waitObject(obj, 3.0)) { err = "无 object_pose"; return false; }
    if (!stageCoarse(obj)) { err = "① 粗定位失败"; return false; }
    if (!stageRefine())    { err = "② 精修失败"; return false; }
    if (!stageInsert(allow_tray_touch)) { err = "③ 末段直插失败"; return false; }
    return true;
  }

  // ① 粗定位: 规划到盒上方 pregrasp_height, 姿态取盒姿态(纯 yaw, 吸盘朝下)
  bool stageCoarse(const geometry_msgs::msg::PoseStamped & obj)
  {
    geometry_msgs::msg::Pose target;
    target.position.x = obj.pose.position.x;
    target.position.y = obj.pose.position.y;
    target.position.z = obj.pose.position.z + pregrasp_height_;
    target.orientation = obj.pose.orientation;

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "① 规划失败"); return false;
    }
    RCLCPP_INFO(logger_, "① 粗定位规划成功, 执行到盒上方 %.0fcm", pregrasp_height_ * 100);
    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // ② 精修: moveit_servo 视觉伺服, 持续读新鲜 object_pose, 对准 xy+yaw 并降到 refine_height
  bool stageRefine()
  {
    if (!callTrigger(start_servo_cli_, "start_servo")) {
      RCLCPP_ERROR(logger_, "② start_servo 失败"); return false;
    }
    RCLCPP_INFO(logger_, "② 伺服启动, 精修对准中...");

    rclcpp::WallRate rate(servo_rate_);
    const rclcpp::Time t0 = node_->now();
    int in_tol = 0;
    bool ok = false;

    while (rclcpp::ok()) {
      if ((node_->now() - t0).seconds() > refine_timeout_) {
        RCLCPP_ERROR(logger_, "② 精修超时"); break;
      }
      geometry_msgs::msg::PoseStamped obj;
      if (!latestObject(obj)) { publishZeroTwist(); rate.sleep(); continue; }

      geometry_msgs::msg::PoseStamped cur;
      if (!currentTcp(cur)) { publishZeroTwist(); rate.sleep(); continue; }

      const double ex = obj.pose.position.x - cur.pose.position.x;
      const double ey = obj.pose.position.y - cur.pose.position.y;
      const double target_z = obj.pose.position.z + refine_height_;
      const double ez = target_z - cur.pose.position.z;
      const double eyaw = wrapAngle(quatYaw(obj.pose.orientation) - quatYaw(cur.pose.orientation));

      const double exy = std::hypot(ex, ey);
      if (exy < xy_tol_ && std::fabs(ez) < z_tol_ && std::fabs(eyaw) < yaw_tol_) {
        if (++in_tol >= converge_cycles_) { ok = true; break; }
      } else {
        in_tol = 0;
      }

      geometry_msgs::msg::TwistStamped tw;
      tw.header.stamp = node_->now();
      tw.header.frame_id = base_frame_;
      tw.twist.linear.x = clampAbs(kp_lin_ * ex, max_lin_vel_);
      tw.twist.linear.y = clampAbs(kp_lin_ * ey, max_lin_vel_);
      tw.twist.linear.z = clampAbs(kp_lin_ * ez, max_lin_vel_);
      tw.twist.angular.z = clampAbs(kp_ang_ * eyaw, max_ang_vel_);
      twist_pub_->publish(tw);
      rate.sleep();
    }

    publishZeroTwist();
    callTrigger(stop_servo_cli_, "stop_servo");
    if (ok) RCLCPP_INFO(logger_, "② 精修收敛(xy/yaw/z 均达标)");
    return ok;
  }

  // ③ 末段: 取当前 suction_tip 位姿, 沿其 -Z 相对下插 insert_stroke, 再吸.
  //    严禁用盒子绝对坐标 setPoseTarget —— waypoint 只由当前实测位姿 + 固定 -Z 偏置得来.
  bool stageInsert(bool allow_tray_touch)
  {
    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) { RCLCPP_ERROR(logger_, "③ 取当前位姿失败"); return false; }

    // 当前 tool 系 -Z 在 base_link 中的方向 = R(cur)*(0,0,-1)
    const auto & q = cur.pose.orientation;
    // R*(0,0,1) 第三列:
    const double zx = 2.0 * (q.x * q.z + q.w * q.y);
    const double zy = 2.0 * (q.y * q.z - q.w * q.x);
    const double zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);

    geometry_msgs::msg::Pose wp = cur.pose;   // 姿态保持当前
    wp.position.x -= zx * insert_stroke_;      // 沿 -Z 下插
    wp.position.y -= zy * insert_stroke_;
    wp.position.z -= zz * insert_stroke_;

    std::vector<geometry_msgs::msg::Pose> waypoints{wp};
    moveit_msgs::msg::RobotTrajectory traj;
    move_group_->setStartStateToCurrentState();
    const double fraction = move_group_->computeCartesianPath(waypoints, 0.005, 0.0, traj);
    RCLCPP_INFO(logger_, "③ 直插笛卡尔路径覆盖 %.0f%%", fraction * 100);
    if (fraction < 0.9) { RCLCPP_ERROR(logger_, "③ 直插路径覆盖不足"); return false; }
    if (move_group_->execute(traj) != moveit::core::MoveItErrorCode::SUCCESS) return false;

    publishPump(PUMP_SUCK);
    RCLCPP_INFO(logger_, "③ 已下插 %.0fcm 并发 /pump_cmd 1 吸取", insert_stroke_ * 100);
    rclcpp::sleep_for(500ms);   // 给 mock_suction 判定吸附
    attachBox(allow_tray_touch);  // 盒进规划场景: 后续放置绕开托盘边框
    return true;
  }

  // 把盒子作为 attached collision object 挂到吸盘: 吸取后吸盘末端在盒顶, 盒心在吸盘
  // -Z(工具系向下)方向 0.0125 处. 此后 MoveIt 规划放置路径会考虑这块几何, 从上方入托盘
  // 而非侧向蹭过边框. touch_links 默认只放吸盘 link (吸取处接触不误报);
  // allow_tray_touch=true 时再把托盘 link 加进 touch_links —— 卸货取盒时盒本在托盘里,
  // 几何与 Link_11 重叠, 不豁免这对接触则规划器判 start-in-collision, 连抬起都规划不了.
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

    publishPump(PUMP_RELEASE);
    detachBox();
    RCLCPP_INFO(logger_, "放置: 已直下到吸盘 z=%.3f (盒底离地~5mm), 发 /pump_cmd 2 释放", z_tip);
    return true;
  }

  // 放到"标定好的绝对目标位姿"(含朝向), 顶向下入位: 相对抬起(携盒) -> 规划到目标正上方
  // (标定 xy+朝向, z 抬 tray_clearance_) -> 笛卡尔垂直直下到标定位姿 -> 释放 + detach.
  // 放托盘用: 托盘在肩后死区, 唯一可达是工具 yaw≈180°朝下的特定位姿(RViz 实测标定).
  // 早前直接 plan() 到标定位姿: 盒 attach 后规划器为让盒全程避开边框会绕大圈甩臂; 改成
  // 顶上空旷处入、再垂直直下, 盒竖直进托盘不蹭边框, 规划器无需绕路, 路径直不甩.
  bool placeAtPose(const geometry_msgs::msg::Pose & target, const char * what)
  {
    // 携盒相对抬起, 先让盒离开当前接触面
    geometry_msgs::msg::PoseStamped cur;
    if (!currentTcp(cur)) return false;
    geometry_msgs::msg::Pose up = cur.pose;
    up.position.z += lift_height_;
    std::vector<geometry_msgs::msg::Pose> wps{up};
    moveit_msgs::msg::RobotTrajectory traj;
    move_group_->setStartStateToCurrentState();
    if (move_group_->computeCartesianPath(wps, 0.005, 0.0, traj) > 0.5) {
      move_group_->execute(traj);
      RCLCPP_INFO(logger_, "放置(%s): 已抬起 %.0fcm", what, lift_height_ * 100);
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
      return false;
    }
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;

    // 笛卡尔垂直直下到标定放置位姿: 盒竖直入托盘, 不侧向蹭边框
    std::vector<geometry_msgs::msg::Pose> dwps{target};
    moveit_msgs::msg::RobotTrajectory dtraj;
    move_group_->setStartStateToCurrentState();
    const double frac = move_group_->computeCartesianPath(dwps, 0.005, 0.0, dtraj);
    if (frac < 0.9) {
      RCLCPP_ERROR(logger_, "放置(%s): 垂直直下路径覆盖不足 %.0f%%", what, frac * 100);
      return false;
    }
    if (move_group_->execute(dtraj) != moveit::core::MoveItErrorCode::SUCCESS) return false;

    publishPump(PUMP_RELEASE);
    detachBox();
    RCLCPP_INFO(logger_, "放置(%s): 垂直入位到 (%.3f,%.3f,%.3f), 发 /pump_cmd 2 释放", what,
                target.position.x, target.position.y, target.position.z);
    return true;
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

  void publishZeroTwist()
  {
    geometry_msgs::msg::TwistStamped tw;
    tw.header.stamp = node_->now();
    tw.header.frame_id = base_frame_;
    twist_pub_->publish(tw);
  }

  void publishPump(int8_t v)
  {
    std_msgs::msg::Int8 m; m.data = v; pump_pub_->publish(m);
  }

  bool callTrigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli, const std::string & name)
  {
    if (!cli->wait_for_service(3s)) {
      RCLCPP_ERROR(logger_, "%s 服务不可用", name.c_str());
      return false;
    }
    auto fut = cli->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (fut.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(logger_, "%s 无响应", name.c_str());
      return false;
    }
    return fut.get()->success;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::string planning_group_, ee_link_, base_frame_, tray_frame_, object_topic_, pump_topic_;
  double pregrasp_height_, refine_height_, insert_stroke_, lift_height_, tray_clearance_;
  double tray_place_x_, tray_place_y_, tray_place_z_;
  double tray_place_qx_, tray_place_qy_, tray_place_qz_, tray_place_qw_;
  double place_x_, place_y_, place_z_, place_clearance_;
  double box_size_x_, box_size_y_, box_size_z_;
  std::string j1_name_;
  double look_j1_offset_;
  double xy_tol_, z_tol_, yaw_tol_, kp_lin_, kp_ang_, max_lin_vel_, max_ang_vel_;
  double servo_rate_, refine_timeout_, object_stale_sec_;
  int converge_cycles_;

  static constexpr const char * kCarriedBoxId = "carried_box";

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_sub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pump_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_cli_, stop_servo_cli_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_, unload_srv_, ready_srv_, look_srv_;
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
