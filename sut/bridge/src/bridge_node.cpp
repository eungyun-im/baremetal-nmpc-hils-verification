// bmpc_bridge: 원본 mpc_path_tracking 노드를 대신하는 PC 쪽 노드.
//
// 원본 modelPredictiveController() 를 둘로 나눴다.
//   PC (이 파일): 입력 수신, 입력 위치 필터, 구간 판정, 3차 polyfit, 속도 제한 체인, 경로 끝 감속,
//                 시간 측정(T, odom_age, CAN 전송), 실효 Lf 추정
//   MCU (bare_metal_mpc): 지연보상, 가중치 스케줄, LTV-MPC, 조향·종방향 후처리
// 아래 전처리 코드는 원본에서 ROS 쪽 부분만 그대로 옮겼다(파라미터 이름과 기본값도 같다).
// MCU 가 돌려준 최종 조향·액셀·브레이크를 /ctrl_cmd 로 발행한다(launch 에서 /ctrl_cmd_mpc 로 remap).
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <mission_supervisor/SectionState.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <new_lane/lidar_topic_msg.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <waypoint_maker/Lane.h>
#include <waypoint_maker/State.h>
#include <waypoint_maker/Waypoint.h>

#include <Eigen/Core>
#include <Eigen/QR>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <array>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "bmpc/bmpc_udp.h"
#include "bmpc_bridge/mcu_config.h"
#include "bmpc_bridge/mcu_link.h"
#include "bmpc_bridge/vehicle_can.h"

#define SOME_MIN 0.01
#define SOME_MAX 0.15

using Eigen::VectorXd;

namespace {
double clampValue(double value, double lower, double upper) { return std::max(lower, std::min(value, upper)); }
double wrapToPi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
// 원본 MPC::polyfit (Householder QR)
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order)
{
  Eigen::MatrixXd A(xvals.size(), order + 1);
  for (int i = 0; i < xvals.size(); i++) A(i, 0) = 1.0;
  for (int j = 0; j < xvals.size(); j++)
    for (int i = 0; i <= order; i++) A(j, i) = pow(xvals(j), i);
  auto Q = A.householderQr();
  return Q.solve(yvals);
}
}  // namespace

class BmpcBridge {
 public:
  BmpcBridge() : nh_(), pnh_("~") {}
  ~BmpcBridge()
  {
    {
      std::lock_guard<std::mutex> lk(pub_mu_);
      pub_stop_ = true;
    }
    pub_cv_.notify_all();
    if (pub_thread_.joinable()) pub_thread_.join();
  }
  bool init();
  void step();
  void spinLoop();

 private:
  // ---- 원본 MPC 기반 클래스의 파라미터 ----
  int N_ = 30;
  double dt = 0.05, ref_v = 20.0, k_v = 0.4, Lf = 1.5;
  float steer_upperBound = 0.6981f, steer_lowerBound = -0.6981f;
  double speed_upperBound = 20.0, speed_lowerBound = 0.0;
  bool is_highway_mode = false, is_avoidance_mode = false;

  // ---- 원본 ROSCONTROL 상태 (전처리에 쓰는 것만) ----
  ros::NodeHandle nh_, pnh_;
  double ex_x_ = 0, ex_y_ = 0, cur_course_ = 0, cur_speed_ = 0;
  ros::Time ex_stamp;
  geometry_msgs::PoseStamped cur_pose_;
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  std::vector<waypoint_maker::Waypoint> speed_waypoints_;
  int waypoints_size_ = 0;
  bool is_pose_ = false, is_lane_ = false, is_course_ = false;
  double dist_ = 100.0;
  int current_mission_state_ = -1, loader_number_ = 0;

  double ref_v_nominal_ = 0, speed_upper_nominal_ = 0, speed_lower_nominal_ = 0;
  double speed_limit_urban_max_mps_ = 15.0, speed_limit_highway_max_mps_ = 25.0;
  bool is_acc = false;
  int acc_index = 0;
  bool external_acc_enabled_ = true;
  double external_acc_target_speed_mps_ = 0, external_acc_timeout_sec_ = 0.5;
  bool have_external_acc_target_ = false;
  ros::Time external_acc_target_stamp_;

  bool in_exempt_section_ = false, have_active_section_ = false;
  std::string active_section_id_, avoidance_section_key_;
  ros::Time last_active_section_time_;
  double avoidance_speed_limit_mps_ = 9999.0;
  ros::Time last_avoidance_speed_time_;
  bool have_avoidance_speed_ = false;
  double static_speed_scale_ = 9999.0;
  ros::Time last_static_speed_scale_time_;
  bool have_static_speed_scale_ = false;
  bool avoidance_speed_limit_was_active_ = false, static_speed_scale_was_active_ = false;

  double steering_angle = 0, target_velocity = 0;

  // 지연보상 시간 측정 (계산은 MCU)
  double dc_fallback_period_s_ = 0.020;
  double dc_period_s_ = -1.0, dc_last_imu_stamp_ = 0.0;
  std::deque<double> dc_intervals_;
  ros::Time dc_last_imu_rx_;
  double dc_lf_est_ = -1.0, dc_lf_x_ = 0, dc_lf_y_ = 0, dc_lf_last_stamp_ = 0;
  ros::Time last_odom_stamp_;
  double odom_yaw_rate_ = 0.0;
  std::string state_source_ = "odom";
  bool ekf_twist_ok_ = false;
  double ekf_yaw_rate_ = 0, ekf_vx_ = 0;

  // 입력 위치 필터
  bool sf_enabled_ = false;
  double sf_pos_tau_s_ = 1.0, sf_yaw_tau_s_ = 1.0;
  double sf_kappa_full_ = 0.004, sf_kappa_zero_ = 0.012, sf_preview_s_ = 2.5;
  double sf_wz_full_ = 0.03, sf_wz_zero_ = 0.10;
  double sf_s_up_rate_ = 0.5, sf_s_last_t_ = 0.0;
  double sf_max_lat_m_ = 0.04, sf_max_yaw_rad_ = 0.3 * M_PI / 180.0;
  double sf_reset_pos_m_ = 0.3, sf_reset_yaw_rad_ = 3.0 * M_PI / 180.0;
  bool sf_init_ = false;
  double sf_x_ = 0, sf_y_ = 0, sf_yaw_ = 0, sf_last_stamp_ = 0, sf_straightness_ = 0;
  int sf_reset_count_ = 0;
  std::string gps_mode_ = "normal";

  bool have_competition_feedback_ = false;
  double competition_speed_mps_ = 0, competition_wheel_angle_deg_ = 0;
  // 안전 제어기 통합 (MCU 펌웨어 SAFETY=1): 차량 CAN 원천 값과 MCU 가 낸 SAFE_OUT
  bool safety_enabled_ = false, safety_reset_on_start_ = true, safety_reset_sent_ = false;
  double safety_accel_gain_ = 3.0, safety_brake_gain_ = 5.0, safety_wait_s_ = 0.025;
  double comp_long_acc_ = 0, comp_lat_acc_ = 0, comp_accel_ = 0, comp_brake_ = 0, comp_prev_stamp_ = 0, comp_prev_v_ = 0;
  double inject_driver_brake_ = 0.0;  // 결함 주입: 운전자 브레이크 페달 [0..1]
  ros::WallTime inject_driver_until_;
  double imu_yaw_rate_dps_ = 0, imu_long_acc_ = 0, imu_lat_acc_ = 0;
  bmpc_bridge::vcan::Encoder vcan_enc_;
  bmpc_bridge::vcan::SafeOut safe_out_;
  bmpc_bridge::vcan::SafeStat safe_stat_;
  ros::Time safe_out_rx_;
  bool have_safe_out_ = false;
  int safe_echo_ = -1;              // 마지막 SAFE_STAT 의 CtrlAliveEcho
  unsigned long safe_crc_err_ = 0, safe_echo_miss_ = 0;
  void onSideFrame(const bmpc_can_frame_t &f);
  void sendVehicleCan();

  // ---- 실험용 주입 (C 단계) ----
  // 지연: 차량 명령을 inject_cmd_delay_s_ 늦게 발행한다 (별도 스레드, 제어 주기는 그대로). 응답 경로가 길어진 것과 같으므로
  // MCU 에 보내는 CAN 전송 시간 추정에 더한다 (지연보상이 켜져 있으면 MCU 가 보상한다).
  double inject_cmd_delay_s_ = 0.0;
  std::thread pub_thread_;
  std::mutex pub_mu_;
  std::condition_variable pub_cv_;
  std::deque<std::pair<std::chrono::steady_clock::time_point, morai_msgs::CtrlCmd>> pub_q_;
  bool pub_stop_ = false;
  void publishCmd(const morai_msgs::CtrlCmd &cmd);
  void publisherThread();
  // 결함 (/bmpc/inject, std_msgs/String): "blackout <s>" 요청·차량 CAN 모두 끊기, "veh_loss <s>" 차량 CAN 만 끊기,
  // "req_crc <n>" 다음 n 개 요청 CRC 변조, "veh_crc <n>" 다음 n 개 ESC 프레임 CRC 변조,
  // "driver_brake <0..1> <s>" 운전자 브레이크 페달, "bus_off <s>" 잘못된 비트레이트로 버스 오류 (TSR-08), "safety_reset" BENCH_RESET
  ros::WallTime inject_blackout_until_, inject_veh_loss_until_, inject_busoff_until_;
  bool busoff_active_ = false;
  ros::Publisher mcu_diag_pub_;
  int inject_req_crc_n_ = 0, inject_veh_crc_n_ = 0;
  bool inject_ctrl_pending_ = false, inject_ctrl_active_ = false, inject_ctrl_hold_ = false;
  double inject_ctrl_offset_rad_ = 0.0, inject_ctrl_ms_ = 0.0, inject_ctrl_last_steer_ = 0.0;
  ros::WallTime inject_ctrl_until_;
  int last_safe_state_ = -1;
  int safe_flags_or_ = 0, safe_flags2_or_ = 0;  // 지난 발행 뒤 받은 모든 SAFE_OUT/STAT 플래그 (한 주기만 서는 플래그도 남긴다)
  // PC 쪽 SAFE_OUT 주기 확인: 지난 발행 뒤 받은 SAFE_OUT 수, OutAlive 가 건너뛴 수 (누적, 빠진 10 ms 주기)
  int safe_out_n_ = 0, safe_last_alive_ = -1;
  unsigned long safe_alive_skips_ = 0;
  ros::Publisher inject_log_pub_;
  void InjectCallback(const std_msgs::String::ConstPtr &msg);
  void injectLog(const std::string &s);

  bool mission_speed_enabled_ = true;
  double mission_speed_default_mps_ = 0;
  std::array<double, 13> mission_speed_state_mps_{};
  double mission_speed_rate_limit_up_mps2_ = 1.8, mission_speed_rate_limit_down_mps2_ = 2.2;
  double filtered_mission_speed_target_mps_ = 0;
  bool have_filtered_mission_speed_target_ = false;

  bool curvature_speed_enabled_ = true;
  double curvature_lateral_accel_limit_mps2_ = 2.3, urban_curvature_lateral_accel_limit_mps2_ = 2.3;
  double urban_curvature_speed_rate_up_mps2_ = 1.0;
  double avoidance_lateral_accel_limit_mps2_ = 3.0, avoidance_curvature_min_speed_mps_ = 3.0;
  std::vector<std::string> avoidance_sections_{"M2", "M8"};
  double curvature_min_speed_mps_ = 3.5, curvature_max_speed_mps_ = 0.0;
  double curvature_preview_distance_m_ = 28.0, preview_distance_urban_m_ = 28.0, preview_distance_highway_m_ = 50.0;
  double curvature_min_lookahead_m_ = 0.5, curvature_deadband_inv_m_ = 0.01;
  double curvature_speed_rate_up_mps2_ = 1.0, curvature_speed_rate_down_mps2_ = 3.0;
  int curvature_sample_count_ = 40;
  double curvature_percentile_ = 0.90;
  bool curvature_use_waypoint_rk_ = true, curvature_fallback_to_polyfit_ = true;
  bool have_filtered_curve_speed_limit_ = false;
  double filtered_curve_speed_limit_mps_ = 0, last_curve_speed_limit_mps_ = 0, last_path_curvature_for_speed_limit_ = 0;
  bool path_end_stop_enabled_ = true;
  int path_end_min_polyfit_points_ = 4;
  double path_end_approach_trigger_m_ = 40.0, path_end_approach_decel_mps2_ = 1.5, path_end_stop_margin_m_ = 1.0;

  // ---- 브리지 ----
  std::unique_ptr<bmpc_bridge::McuLink> link_;
  bmpc_config_t mcu_cfg_;
  bool configured_ = false;
  ros::WallTime last_config_try_;
  double response_timeout_s_ = 0.030, config_timeout_s_ = 0.5;
  int fallback_hold_cycles_ = 3;
  bool reset_next_ = true;
  int consecutive_fail_ = 0;
  uint64_t total_fail_ = 0;
  bool have_last_cmd_ = false;
  double last_steer_ = 0, last_accel_ = 0, last_brake_ = 0;

  // ---- 발행 ----
  ros::Publisher ackermann_pub_, mpc_target_speed_pub_, curvature_speed_limit_pub_, curvature_capacity_speed_pub_,
      path_curvature_for_speed_limit_pub_, mission_speed_target_pub_, is_highway_mode_pub_, steering_delay_pub_,
      dc_debug_pub_, sf_debug_pub_, controller_time_pub_, bmpc_debug_pub_, bmpc_safety_pub_;
  std::vector<ros::Subscriber> subs_;

  // ---- 콜백 (원본 그대로) ----
  void EkfTwistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg);
  void EkfPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void GpsModeCallback(const std_msgs::String::ConstPtr &msg) { gps_mode_ = msg->data; }
  void PathCallback(const nav_msgs::Path::ConstPtr &path_msg);
  void FinalWaypointCallback(const waypoint_maker::Lane::ConstPtr &lane_msg);
  void StateCallback(const waypoint_maker::State::ConstPtr &state_msg);
  void LidarLaneCallback(const new_lane::lidar_topic_msg::ConstPtr &msg);
  void ExternalAccSpeedCallback(const std_msgs::Float64::ConstPtr &msg);
  void DelayImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void updateEffectiveLf(const morai_msgs::EgoVehicleStatus &msg);
  void CompetitionCallback(const morai_msgs::EgoVehicleStatus::ConstPtr &msg);
  void AvoidanceSpeedCallback(const std_msgs::Float64::ConstPtr &msg);
  void StaticSpeedScaleCallback(const std_msgs::Float64::ConstPtr &msg);
  void ActiveSectionCallback(const mission_supervisor::SectionState::ConstPtr &msg);

  // ---- 원본 함수 그대로 ----
  static double applyACC_kmh(double spd_state_kmh, bool is_acc, int acc_index);
  bool determineIsHighway() const;
  double missionStateTargetSpeedMps() const;
  double polynomialCurvatureAbs(const VectorXd &coeffs, double x) const;
  double percentileCurvatureSample(std::vector<double> &curvature_samples) const;
  double previewWaypointRkForSpeedLimit() const;
  double previewPolyfitCurvatureForSpeedLimit(const VectorXd &coeffs, const VectorXd &vehicle_waypoints_x) const;
  double computeCurvatureSpeedLimit(const VectorXd &coeffs, const VectorXd &vehicle_waypoints_x);
  void publishFloat64(ros::Publisher &p, double v)
  {
    std_msgs::Float64 m;
    m.data = v;
    p.publish(m);
  }
  void publishTargetSpeed(double v) { publishFloat64(mpc_target_speed_pub_, v); }

  // ---- 브리지 ----
  bool ensureConfigured();
  void fillTiming(bmpc_request_t &req);
  void runMcu(bmpc_request_t &req, bool stop);
};

// =====================================================================
// 초기화 (원본 MPC() + initSetup() 의 파라미터 읽기)
// 파라미터는 모두 이 노드의 private 이름공간(~)에서 읽는다. 값은 config/bmpc_params.yaml 에 있고
// sils.launch / replay_check.launch 가 노드 안에서 읽어 둔다. 대회 스택의 전역 파라미터는 보지 않는다.
// =====================================================================
bool BmpcBridge::init()
{
  // yaml 을 안 읽고 띄우면 아래 기본값(원본 코드 값, N=50 Lf=1.5 등)으로 조용히 돌게 되므로 시작하지 않는다
  if (!pnh_.hasParam("N") || !pnh_.hasParam("Lf")) {
    ROS_FATAL("[BMPC] %s/N or Lf not set: load bmpc_bridge/config/bmpc_params.yaml inside the node (rosparam load)",
              pnh_.getNamespace().c_str());
    return false;
  }
  pnh_.param<int>("N", N_, 50);
  pnh_.param<double>("dt", dt, 0.1);
  pnh_.param<double>("k_v", k_v, 0.4);
  pnh_.param<double>("ref_v", ref_v, 20.0);
  pnh_.param<double>("Lf", Lf, 1.5);
  pnh_.param<float>("steer/upperBound", steer_upperBound, 0.6981f);
  pnh_.param<float>("steer/lowerBound", steer_lowerBound, -0.6981f);
  pnh_.param<double>("speed/upperBound", speed_upperBound, 20.0);
  pnh_.param<double>("speed/lowerBound", speed_lowerBound, 0);

  ref_v_nominal_ = ref_v;
  speed_upper_nominal_ = speed_upperBound;
  speed_lower_nominal_ = speed_lowerBound;

  pnh_.param<double>("speed_limit/urban_max_mps", speed_limit_urban_max_mps_, 15.0);
  pnh_.param<double>("urban_control/max_speed_mps", speed_limit_urban_max_mps_, speed_limit_urban_max_mps_);
  pnh_.param<double>("speed_limit/highway_max_mps", speed_limit_highway_max_mps_, 25.0);
  pnh_.param<double>("highway_control/max_speed_mps", speed_limit_highway_max_mps_, speed_limit_highway_max_mps_);
  pnh_.param<bool>("external_acc/enabled", external_acc_enabled_, true);
  pnh_.param<double>("external_acc/timeout_sec", external_acc_timeout_sec_, 0.5);
  pnh_.param<bool>("path_end_stop/enabled", path_end_stop_enabled_, true);
  pnh_.param<int>("path_end_stop/min_polyfit_points", path_end_min_polyfit_points_, 4);
  pnh_.param<double>("path_end_stop/approach_trigger_m", path_end_approach_trigger_m_, 40.0);
  pnh_.param<double>("path_end_stop/approach_decel_mps2", path_end_approach_decel_mps2_, 1.5);
  pnh_.param<double>("path_end_stop/stop_margin_m", path_end_stop_margin_m_, 1.0);
  if (path_end_min_polyfit_points_ < 4) path_end_min_polyfit_points_ = 4;

  pnh_.param<bool>("mission_speed/enabled", mission_speed_enabled_, true);
  pnh_.param<double>("mission_speed/default_mps", mission_speed_default_mps_, ref_v_nominal_);
  mission_speed_state_mps_.fill(mission_speed_default_mps_);
  for (size_t i = 0; i < mission_speed_state_mps_.size(); ++i)
    pnh_.param<double>("mission_speed/state_" + std::to_string(i) + "_mps", mission_speed_state_mps_[i],
                      mission_speed_default_mps_);
  pnh_.param<double>("mission_speed/rate_limit_up_mps2", mission_speed_rate_limit_up_mps2_, 1.8);
  pnh_.param<double>("mission_speed/rate_limit_down_mps2", mission_speed_rate_limit_down_mps2_, 2.2);
  pnh_.param<bool>("curvature_speed/enabled", curvature_speed_enabled_, true);
  pnh_.param<double>("curvature_speed/lateral_accel_limit_mps2", curvature_lateral_accel_limit_mps2_, 2.3);
  pnh_.param<double>("avoidance_control/lateral_accel_limit_mps2", avoidance_lateral_accel_limit_mps2_, 3.0);
  pnh_.param<double>("avoidance_control/curvature_min_speed_mps", avoidance_curvature_min_speed_mps_, 3.0);
  pnh_.param("avoidance_control/sections", avoidance_sections_, std::vector<std::string>{"M2", "M8"});
  pnh_.param<double>("curvature_speed/min_speed_mps", curvature_min_speed_mps_, 3.5);
  pnh_.param<double>("curvature_speed/max_speed_mps", curvature_max_speed_mps_, 0.0);
  pnh_.param<double>("urban_control/preview_distance_m", preview_distance_urban_m_, 28.0);
  pnh_.param<double>("highway_control/preview_distance_m", preview_distance_highway_m_, 50.0);
  pnh_.param<double>("curvature_speed/preview_distance_m", curvature_preview_distance_m_, preview_distance_urban_m_);
  pnh_.param<double>("curvature_speed/min_lookahead_m", curvature_min_lookahead_m_, 0.5);
  pnh_.param<double>("curvature_speed/deadband_inv_m", curvature_deadband_inv_m_, 0.01);
  pnh_.param<double>("curvature_speed/rate_limit_up_mps2", curvature_speed_rate_up_mps2_, 1.0);
  pnh_.param<double>("urban_control/curvature_lateral_accel_limit_mps2", urban_curvature_lateral_accel_limit_mps2_,
                    curvature_lateral_accel_limit_mps2_);
  pnh_.param<double>("urban_control/curvature_rate_limit_up_mps2", urban_curvature_speed_rate_up_mps2_,
                    curvature_speed_rate_up_mps2_);
  pnh_.param<double>("curvature_speed/rate_limit_down_mps2", curvature_speed_rate_down_mps2_, 3.0);
  pnh_.param<int>("curvature_speed/sample_count", curvature_sample_count_, 40);
  pnh_.param<double>("curvature_speed/percentile", curvature_percentile_, 0.90);
  pnh_.param<bool>("curvature_speed/use_waypoint_rk", curvature_use_waypoint_rk_, true);
  pnh_.param<bool>("curvature_speed/fallback_to_polyfit", curvature_fallback_to_polyfit_, true);
  curvature_percentile_ = clampValue(curvature_percentile_, 0.0, 1.0);

  pnh_.param<double>("delay_comp/fallback_period_s", dc_fallback_period_s_, 0.020);

  pnh_.param<std::string>("state_input/source", state_source_, std::string("odom"));
  pnh_.param<bool>("state_filter/enabled", sf_enabled_, false);
  pnh_.param<double>("state_filter/pos_tau_s", sf_pos_tau_s_, 1.0);
  pnh_.param<double>("state_filter/yaw_tau_s", sf_yaw_tau_s_, 1.0);
  pnh_.param<double>("state_filter/kappa_full", sf_kappa_full_, 0.004);
  pnh_.param<double>("state_filter/kappa_zero", sf_kappa_zero_, 0.012);
  pnh_.param<double>("state_filter/preview_s", sf_preview_s_, 2.5);
  pnh_.param<double>("state_filter/yaw_rate_full_radps", sf_wz_full_, 0.03);
  pnh_.param<double>("state_filter/yaw_rate_zero_radps", sf_wz_zero_, 0.10);
  pnh_.param<double>("state_filter/straightness_up_rate_per_s", sf_s_up_rate_, 0.5);
  pnh_.param<double>("state_filter/max_lat_offset_m", sf_max_lat_m_, 0.04);
  double sf_max_yaw_deg = 0.3, sf_reset_yaw_deg = 3.0;
  pnh_.param<double>("state_filter/max_yaw_offset_deg", sf_max_yaw_deg, 0.3);
  sf_max_yaw_rad_ = sf_max_yaw_deg * M_PI / 180.0;
  pnh_.param<double>("state_filter/reset_pos_m", sf_reset_pos_m_, 0.3);
  pnh_.param<double>("state_filter/reset_yaw_deg", sf_reset_yaw_deg, 3.0);
  sf_reset_yaw_rad_ = sf_reset_yaw_deg * M_PI / 180.0;

  // ---- 발행 (원본과 같은 이름) ----
  ackermann_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
  mpc_target_speed_pub_ = nh_.advertise<std_msgs::Float64>("/control_feedback/mpc_target_speed_mps", 10);
  curvature_speed_limit_pub_ = nh_.advertise<std_msgs::Float64>("/control_feedback/curvature_speed_limit_mps", 10);
  curvature_capacity_speed_pub_ = nh_.advertise<std_msgs::Float64>("/control_feedback/curvature_capacity_speed_mps", 10);
  path_curvature_for_speed_limit_pub_ =
      nh_.advertise<std_msgs::Float64>("/control_feedback/path_curvature_for_speed_limit", 10);
  mission_speed_target_pub_ = nh_.advertise<std_msgs::Float64>("/control_feedback/mission_speed_target_mps", 10);
  is_highway_mode_pub_ = nh_.advertise<std_msgs::Bool>("/control_feedback/is_highway_mode", 10);
  steering_delay_pub_ = nh_.advertise<std_msgs::Float64>("/control_feedback/steering_delay_s", 10);
  dc_debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/control_feedback/delay_comp", 10);
  controller_time_pub_ = nh_.advertise<std_msgs::Float64>("/mpc/controller_time_ms", 10);
  // [seq, status, link(0 ok 1 timeout 2 rx), rtt_ms, mcu_total_ms, mcu_solve_ms, transport_ms, qp_iters,
  //  delta0, vt_raw, vt_final, steer_cmd, accel, brake, state_delay, compute, actuation, lf_used,
  //  fallback(0/1), consecutive_fail]
  bmpc_debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/bmpc/debug", 10);
  bmpc_safety_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/bmpc/safety", 10);
  inject_log_pub_ = nh_.advertise<std_msgs::String>("/bmpc/inject_log", 50);
  mcu_diag_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/bmpc/mcu_diag", 10);
  subs_.push_back(nh_.subscribe("/bmpc/inject", 10, &BmpcBridge::InjectCallback, this));
  pnh_.param<double>("inject/cmd_delay_s", inject_cmd_delay_s_, 0.0);
  if (inject_cmd_delay_s_ > 0.0) {
    pub_thread_ = std::thread(&BmpcBridge::publisherThread, this);
    ROS_WARN("[BMPC] inject: vehicle command delayed by %.0f ms (added to the transport estimate sent to the MCU)",
             inject_cmd_delay_s_ * 1e3);
  }

  // ---- 구독 (원본과 같은 이름) ----
  subs_.push_back(nh_.subscribe("/imu", 50, &BmpcBridge::DelayImuCallback, this));
  if (state_source_ == "ekf") {
    subs_.push_back(nh_.subscribe("/ekf_pose_with_covariance", 1, &BmpcBridge::EkfPoseCallback, this));
    subs_.push_back(nh_.subscribe("/ekf_twist_with_covariance", 1, &BmpcBridge::EkfTwistCallback, this));
  } else {
    state_source_ = "odom";
    subs_.push_back(nh_.subscribe("/odom", 1, &BmpcBridge::OdomCallback, this));
  }
  if (sf_enabled_) {
    subs_.push_back(nh_.subscribe("/localization/gps_mode", 5, &BmpcBridge::GpsModeCallback, this));
    sf_debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/control_feedback/state_filter", 10);
  }
  subs_.push_back(nh_.subscribe("/local_path", 1, &BmpcBridge::PathCallback, this));
  subs_.push_back(nh_.subscribe("/final_waypoint", 1, &BmpcBridge::FinalWaypointCallback, this));
  subs_.push_back(nh_.subscribe("/gps_state", 1, &BmpcBridge::StateCallback, this));
  subs_.push_back(nh_.subscribe("lidar_topic", 1, &BmpcBridge::LidarLaneCallback, this));
  subs_.push_back(nh_.subscribe("/smpc/target_speed_mps", 10, &BmpcBridge::ExternalAccSpeedCallback, this));
  subs_.push_back(nh_.subscribe("/Competition_topic", 100, &BmpcBridge::CompetitionCallback, this));
  subs_.push_back(nh_.subscribe("/obstacle_avoidance/target_speed_mps", 5, &BmpcBridge::AvoidanceSpeedCallback, this));
  subs_.push_back(
      nh_.subscribe("/obstacle_avoidance/static_speed_scale", 5, &BmpcBridge::StaticSpeedScaleCallback, this));
  subs_.push_back(nh_.subscribe("/mission/active_section", 10, &BmpcBridge::ActiveSectionCallback, this));

  // ---- MCU 링크 ----
  bmpc_bridge::loadMcuConfig(pnh_, &mcu_cfg_);
  std::string transport = "udp", host = "127.0.0.1", ifname = "can0", err;
  int local_port = BMPC_UDP_PORT_PC, peer_port = BMPC_UDP_PORT_MCU;
  pnh_.param<std::string>("transport", transport, transport);
  pnh_.param<int>("udp_local_port", local_port, local_port);
  pnh_.param<std::string>("udp_peer_host", host, host);
  pnh_.param<int>("udp_peer_port", peer_port, peer_port);
  pnh_.param<std::string>("can_ifname", ifname, ifname);
  std::string usb_port = "/dev/ttyUSB0";
  int usb_bitrate = 500000;
  double usb_gap_s = 0.004;
  pnh_.param<std::string>("usbcan_port", usb_port, usb_port);
  pnh_.param<int>("usbcan_bitrate", usb_bitrate, usb_bitrate);
  pnh_.param<double>("usbcan_gap_s", usb_gap_s, usb_gap_s);
  pnh_.param<double>("response_timeout_s", response_timeout_s_, 0.030);
  pnh_.param<double>("config_timeout_s", config_timeout_s_, 0.5);
  pnh_.param<int>("fallback_hold_cycles", fallback_hold_cycles_, 3);
  std::unique_ptr<bmpc_bridge::CanTransport> t;
  if (transport == "socketcan") t = bmpc_bridge::makeSocketCanTransport(ifname, &err);
  else if (transport == "usbcan") t = bmpc_bridge::makeUsbCanTransport(usb_port, usb_bitrate, usb_gap_s, &err);
  else t = bmpc_bridge::makeUdpTransport(local_port, host, peer_port, &err);
  if (!t) {
    ROS_FATAL("[BMPC] transport %s: %s", transport.c_str(), err.c_str());
    return false;
  }
  link_.reset(new bmpc_bridge::McuLink(std::move(t)));
  pnh_.param<bool>("safety/enabled", safety_enabled_, false);
  pnh_.param<double>("safety/accel_gain", safety_accel_gain_, 3.0);
  pnh_.param<double>("safety/brake_gain", safety_brake_gain_, 5.0);
  pnh_.param<double>("safety/wait_s", safety_wait_s_, 0.025);
  pnh_.param<bool>("safety/reset_on_start", safety_reset_on_start_, true);
  if (safety_enabled_) {
    link_->setSideHandler([this](const bmpc_can_frame_t &f) { onSideFrame(f); });
    ROS_INFO("[BMPC] safety controller on MCU: publish SAFE_OUT (gains %.1f/%.1f, wait %.0f ms)", safety_accel_gain_,
             safety_brake_gain_, safety_wait_s_ * 1e3);
  }
  ROS_INFO("[BMPC] bridge: %s, N=%d dt=%.3f Lf=%.2f, delay_comp %s, state filter %s, state input %s, timeout %.0f ms",
           link_->describe().c_str(), (int)mcu_cfg_.N, (double)mcu_cfg_.dt, (double)mcu_cfg_.Lf,
           mcu_cfg_.dc_enabled ? "ON (MCU)" : "off", sf_enabled_ ? "ON" : "off", state_source_.c_str(),
           response_timeout_s_ * 1e3);
  ensureConfigured();
  return true;
}

// =====================================================================
// 콜백 (원본 그대로)
// =====================================================================
void BmpcBridge::EkfTwistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg)
{
  ekf_twist_ok_ = std::isfinite(msg->twist.twist.linear.x) && std::isfinite(msg->twist.twist.angular.z);
  if (!ekf_twist_ok_) return;
  ekf_yaw_rate_ = msg->twist.twist.angular.z;
  odom_yaw_rate_ = ekf_yaw_rate_;
  ekf_vx_ = msg->twist.twist.linear.x;
}

void BmpcBridge::EkfPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  const auto &p = msg->pose.pose;
  const bool finite = std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.orientation.x) &&
                      std::isfinite(p.orientation.y) && std::isfinite(p.orientation.z) &&
                      std::isfinite(p.orientation.w) && std::isfinite(msg->pose.covariance[0]) &&
                      std::isfinite(msg->pose.covariance[7]);
  if (!ekf_twist_ok_ || !finite) return;
  nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
  odom->header = msg->header;
  odom->pose = msg->pose;
  odom->twist.twist.angular.z = ekf_yaw_rate_;
  OdomCallback(odom);
}

void BmpcBridge::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  ros::Time cur_stamp = odom_msg->header.stamp;
  cur_pose_.pose.position = odom_msg->pose.pose.position;
  cur_pose_.pose.orientation = odom_msg->pose.pose.orientation;
  double cur_time_sec = cur_stamp.toSec();
  double ex_time_sec = ex_stamp.toSec();
  double dtt = cur_time_sec - ex_time_sec;
  if (dtt <= 1e-3) dtt = 0.1;
  if (dtt > SOME_MIN && dtt < SOME_MAX) {
    const double distance = std::hypot(odom_msg->pose.pose.position.x - ex_x_, odom_msg->pose.pose.position.y - ex_y_);
    cur_speed_ = distance / dtt;
  }
  ex_stamp = cur_stamp;
  last_odom_stamp_ = cur_stamp;
  odom_yaw_rate_ = std::isfinite(odom_msg->twist.twist.angular.z) ? odom_msg->twist.twist.angular.z : 0.0;
  if (state_source_ == "odom" && std::isfinite(odom_msg->twist.twist.linear.x)) ekf_vx_ = odom_msg->twist.twist.linear.x;
  ex_x_ = cur_pose_.pose.position.x;
  ex_y_ = cur_pose_.pose.position.y;
  is_pose_ = true;
  tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                   odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  cur_course_ = yaw;
  is_course_ = true;
}

void BmpcBridge::PathCallback(const nav_msgs::Path::ConstPtr &path_msg)
{
  waypoints_ = path_msg->poses;
  waypoints_size_ = (int)waypoints_.size();
  is_lane_ = !waypoints_.empty();
}

void BmpcBridge::FinalWaypointCallback(const waypoint_maker::Lane::ConstPtr &lane_msg)
{
  speed_waypoints_ = lane_msg->waypoints;
}

void BmpcBridge::StateCallback(const waypoint_maker::State::ConstPtr &state_msg)
{
  dist_ = state_msg->dist;
  current_mission_state_ = state_msg->current_state;
  loader_number_ = state_msg->lane_number;
}

void BmpcBridge::LidarLaneCallback(const new_lane::lidar_topic_msg::ConstPtr &msg)
{
  is_acc = msg->ACC_flag;
  acc_index = msg->ACC_index_gap;
}

void BmpcBridge::ExternalAccSpeedCallback(const std_msgs::Float64::ConstPtr &msg)
{
  if (!std::isfinite(msg->data)) return;
  external_acc_target_speed_mps_ = std::max(0.0, msg->data);
  external_acc_target_stamp_ = ros::Time::now();
  have_external_acc_target_ = true;
}

void BmpcBridge::DelayImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu_yaw_rate_dps_ = msg->angular_velocity.z * 180.0 / M_PI;
  imu_long_acc_ = msg->linear_acceleration.x;
  imu_lat_acc_ = msg->linear_acceleration.y;
  const double stamp = msg->header.stamp.toSec();
  if (stamp <= 0.0) return;
  if (dc_last_imu_stamp_ > 0.0) {
    const double d = stamp - dc_last_imu_stamp_;
    if (d <= 1e-6) return;
    if (d > 0.005 && d < 0.2) {
      dc_intervals_.push_back(d);
      while (dc_intervals_.size() > 15) dc_intervals_.pop_front();
      if (dc_intervals_.size() >= 5) {
        std::vector<double> sorted(dc_intervals_.begin(), dc_intervals_.end());
        std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
        dc_period_s_ = sorted[sorted.size() / 2];
      }
    }
  }
  dc_last_imu_stamp_ = stamp;
  dc_last_imu_rx_ = ros::Time::now();
}

// 실제 차량 응답으로 측정된 현재 차량 거동의 lf 측정 -> 지연보상에서 명령이 반영되기 전까지 차의 거동을 실효 lf를 사용하여 예측 (자전거 모델)

void BmpcBridge::updateEffectiveLf(const morai_msgs::EgoVehicleStatus &msg)
{
  const double stamp = msg.header.stamp.toSec();
  if (stamp <= dc_lf_last_stamp_ + 1e-6) return;
  const double dtt = dc_lf_last_stamp_ > 0.0 ? std::min(stamp - dc_lf_last_stamp_, 0.2) : 0.05;
  dc_lf_last_stamp_ = stamp;
  const double v = std::hypot(msg.velocity.x, msg.velocity.y);
  const double delta = msg.wheel_angle * M_PI / 180.0;  // deg -> rad
  const double yaw_rate = -msg.angular_velocity.z;
  if (!std::isfinite(v) || !std::isfinite(delta) || !std::isfinite(yaw_rate)) return;
  const double a = 1.0 - std::exp(-dtt / 0.5);
  // lpf
  dc_lf_x_ += a * (-v * delta - dc_lf_x_);
  dc_lf_y_ += a * (yaw_rate - dc_lf_y_);
  if (v < 3.0 || std::fabs(dc_lf_y_) < 0.08 || dc_lf_x_ * dc_lf_y_ <= 0.0) return;
  const double lf = dc_lf_x_ / dc_lf_y_;
  if (lf < 3.0 || lf > 6.0) return;
  const double b = 1.0 - std::exp(-dtt / 3.0);
  dc_lf_est_ = dc_lf_est_ <= 0.0 ? lf : dc_lf_est_ + b * (lf - dc_lf_est_);
}

void BmpcBridge::CompetitionCallback(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
{
  competition_speed_mps_ = std::hypot(msg->velocity.x, msg->velocity.y);
  competition_wheel_angle_deg_ = msg->wheel_angle;
  // MORAI UDP 상태의 acceleration 은 비어 있다 (항상 0). ESC 가 자체로 내는 값처럼 만든다:
  // 횡가속도 = 속도 x 요레이트 (요레이트는 updateEffectiveLf 와 같은 부호, 좌회전 +), 종가속도 = 속도 미분 (저역 통과)
  {
    const double stamp = msg->header.stamp.toSec();
    const double v = competition_speed_mps_;
    if (comp_prev_stamp_ > 0.0 && stamp > comp_prev_stamp_ + 1e-4) {
      const double dvdt = (v - comp_prev_v_) / (stamp - comp_prev_stamp_);
      comp_long_acc_ += 0.2 * (dvdt - comp_long_acc_);
    }
    comp_prev_stamp_ = stamp;
    comp_prev_v_ = v;
    comp_lat_acc_ = v * (-msg->angular_velocity.z);
  }
  comp_accel_ = msg->accel;
  comp_brake_ = msg->brake;
  have_competition_feedback_ = true;
  updateEffectiveLf(*msg);
}

void BmpcBridge::AvoidanceSpeedCallback(const std_msgs::Float64::ConstPtr &msg)
{
  avoidance_speed_limit_mps_ = msg->data;
  last_avoidance_speed_time_ = ros::Time::now();
  have_avoidance_speed_ = true;
}

void BmpcBridge::StaticSpeedScaleCallback(const std_msgs::Float64::ConstPtr &msg)
{
  static_speed_scale_ = msg->data;
  last_static_speed_scale_time_ = ros::Time::now();
  have_static_speed_scale_ = true;
}

void BmpcBridge::ActiveSectionCallback(const mission_supervisor::SectionState::ConstPtr &msg)
{
  in_exempt_section_ = msg->speed_limit_exempt;
  active_section_id_ = msg->section_id;
  const bool entry =
      msg->index_valid && active_section_id_ == "M7B" &&
      std::find(msg->executors.begin(), msg->executors.end(), "static_obstacle") != msg->executors.end() &&
      std::find(msg->executors.begin(), msg->executors.end(), "dynamic_obstacle") != msg->executors.end();
  avoidance_section_key_ = entry ? "M8" : active_section_id_;
  last_active_section_time_ = ros::Time::now();
  have_active_section_ = true;
}

// =====================================================================
// 원본 함수 그대로
// =====================================================================
double BmpcBridge::applyACC_kmh(double spd_state_kmh, bool is_acc, int acc_index)
{
  double input_speed_kmh = spd_state_kmh;
  if (!is_acc) return input_speed_kmh;
  if (acc_index > 150) {
    return spd_state_kmh;
  } else if (acc_index <= 150 && acc_index > 80) {
    const double x = static_cast<double>(acc_index);
    const double sig = 1.0 / (1.0 + std::exp(-0.017 * (x - 60.0)));
    double delta = 0.0;
    if (spd_state_kmh == 20.0) delta = (83.3 * sig - 68.67);
    else if (spd_state_kmh == 18.0) delta = (75.0 * sig - 61.8);
    else if (spd_state_kmh == 16.0) delta = (66.7 * sig - 54.93);
    else if (spd_state_kmh == 14.0) delta = (58.3 * sig - 48.067);
    else if (spd_state_kmh == 12.0) delta = (50.0 * sig - 41.2);
    input_speed_kmh = spd_state_kmh + delta;
    return input_speed_kmh;
  } else if (acc_index <= 80 && acc_index > 0) {
    return 0.0;
  }
  return input_speed_kmh;
}

bool BmpcBridge::determineIsHighway() const
{
  int wp_idx = -1;
  int wp_lane = loader_number_;
  if (!speed_waypoints_.empty()) {
    wp_idx = speed_waypoints_.front().waypoint_index;
    wp_lane = speed_waypoints_.front().lane_number;
  }
  if (wp_lane == 0) {
    return wp_idx >= 11124;
  } else if (wp_lane == 1 || wp_lane == 2) {
    return true;
  } else if (wp_lane == 3) {
    if (wp_idx >= 0 && wp_idx < 6197) return true;
    if (wp_idx >= 6197) return false;
  }
  const bool section_fresh = have_active_section_ && (ros::Time::now() - last_active_section_time_).toSec() < 1.0;
  if (section_fresh) return in_exempt_section_;
  return (current_mission_state_ == 3);
}

double BmpcBridge::missionStateTargetSpeedMps() const
{
  const bool is_hwy = determineIsHighway();
  const double current_max_speed_limit = is_hwy ? speed_limit_highway_max_mps_ : speed_limit_urban_max_mps_;
  const double effective_speed_upper =
      (current_max_speed_limit > 0.0) ? std::min(speed_upper_nominal_, current_max_speed_limit) : speed_upper_nominal_;
  const double nominal_speed = std::max(speed_lower_nominal_, std::min(ref_v_nominal_, effective_speed_upper));
  if (!mission_speed_enabled_) return nominal_speed;
  if (is_hwy && current_mission_state_ == 3) {
    return clampValue(mission_speed_state_mps_[3], speed_lower_nominal_, effective_speed_upper);
  }
  double target_speed = mission_speed_default_mps_;
  if (current_mission_state_ >= 0 && current_mission_state_ <= 12) target_speed = mission_speed_state_mps_[current_mission_state_];
  if (!std::isfinite(target_speed) || target_speed <= 0.0) target_speed = nominal_speed;
  return clampValue(target_speed, speed_lower_nominal_, effective_speed_upper);
}

double BmpcBridge::polynomialCurvatureAbs(const VectorXd &coeffs, double x) const
{
  if (coeffs.size() < 3) return 0.0;
  double dy = coeffs[1];
  double ddy = 2.0 * coeffs[2];
  if (coeffs.size() >= 4) {
    dy += 2.0 * coeffs[2] * x + 3.0 * coeffs[3] * x * x;
    ddy += 6.0 * coeffs[3] * x;
  } else {
    dy += 2.0 * coeffs[2] * x;
  }
  const double denominator = std::pow(1.0 + dy * dy, 1.5);
  if (denominator < 1e-9) return 0.0;
  return std::fabs(ddy / denominator);
}

double BmpcBridge::percentileCurvatureSample(std::vector<double> &curvature_samples) const
{
  if (curvature_samples.empty()) return 0.0;
  std::sort(curvature_samples.begin(), curvature_samples.end());
  const double percentile = clampValue(curvature_percentile_, 0.0, 1.0);
  const int index = static_cast<int>(std::round(percentile * static_cast<double>(curvature_samples.size() - 1)));
  return curvature_samples[std::max(0, std::min(index, static_cast<int>(curvature_samples.size() - 1)))];
}

// 곡률에 따른 속도 조절
double BmpcBridge::previewWaypointRkForSpeedLimit() const
{
  if (speed_waypoints_.size() < 2) return 0.0;
  double path_s = 0.0;
  std::vector<double> curvature_samples;
  curvature_samples.reserve(speed_waypoints_.size());
  for (size_t i = 1; i < speed_waypoints_.size(); ++i) {
    const auto &prev = speed_waypoints_[i - 1].pose.pose.position;
    const auto &curr = speed_waypoints_[i].pose.pose.position;
    const double ds = std::hypot(curr.x - prev.x, curr.y - prev.y);
    if (!std::isfinite(ds)) continue;
    path_s += ds;
    if (path_s < curvature_min_lookahead_m_) continue;
    if (path_s > curvature_preview_distance_m_) break;
    const double kappa_abs = std::fabs(speed_waypoints_[i].rk);
    if (std::isfinite(kappa_abs)) curvature_samples.push_back(kappa_abs);
  }
  return percentileCurvatureSample(curvature_samples);
}

double BmpcBridge::previewPolyfitCurvatureForSpeedLimit(const VectorXd &coeffs, const VectorXd &vehicle_waypoints_x) const
{
  double max_x = 0.0;
  for (int i = 0; i < vehicle_waypoints_x.size(); ++i) {
    const double x = vehicle_waypoints_x[i];
    if (std::isfinite(x) && x > max_x) max_x = x;
  }
  const double x_start = std::max(0.0, curvature_min_lookahead_m_);
  const double x_end = std::min(curvature_preview_distance_m_, max_x);
  if (x_end <= x_start + 1e-3) return 0.0;
  const int samples = std::max(2, curvature_sample_count_);
  std::vector<double> curvature_samples;
  curvature_samples.reserve(samples);
  for (int i = 0; i < samples; ++i) {
    const double ratio = (samples == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(samples - 1);
    const double x = x_start + ratio * (x_end - x_start);
    const double kappa_abs = polynomialCurvatureAbs(coeffs, x);
    if (std::isfinite(kappa_abs)) curvature_samples.push_back(kappa_abs);
  }
  return percentileCurvatureSample(curvature_samples);
}

double BmpcBridge::computeCurvatureSpeedLimit(const VectorXd &coeffs, const VectorXd &vehicle_waypoints_x)
{
  const double active_nominal_limit = std::min(ref_v, speed_upperBound);
  const double nominal_limit =
      (curvature_max_speed_mps_ > 0.0) ? std::min(curvature_max_speed_mps_, active_nominal_limit) : active_nominal_limit;
  const double safe_nominal_limit = std::max(0.0, nominal_limit);
  const bool m2_avoidance =
      have_active_section_ && (ros::Time::now() - last_active_section_time_).toSec() < 0.5 &&
      std::find(avoidance_sections_.begin(), avoidance_sections_.end(), avoidance_section_key_) !=
          avoidance_sections_.end() &&
      have_avoidance_speed_ && (ros::Time::now() - last_avoidance_speed_time_).toSec() < 0.5 &&
      avoidance_speed_limit_mps_ < 500.0;
  const double min_limit =
      std::min(m2_avoidance ? avoidance_curvature_min_speed_mps_ : curvature_min_speed_mps_, safe_nominal_limit);

  double preview_curvature = 0.0;
  if (curvature_use_waypoint_rk_) preview_curvature = previewWaypointRkForSpeedLimit();
  if ((!curvature_use_waypoint_rk_ ||
       (curvature_fallback_to_polyfit_ && preview_curvature <= curvature_deadband_inv_m_)) &&
      curvature_fallback_to_polyfit_) {
    preview_curvature = previewPolyfitCurvatureForSpeedLimit(coeffs, vehicle_waypoints_x);
  }
  last_path_curvature_for_speed_limit_ = preview_curvature;

  const bool ordinary_urban = !is_highway_mode && !m2_avoidance;
  const double lateral_accel_limit =
      m2_avoidance ? avoidance_lateral_accel_limit_mps2_
                   : (ordinary_urban ? urban_curvature_lateral_accel_limit_mps2_ : curvature_lateral_accel_limit_mps2_);
  const double curve_rate_up = ordinary_urban ? urban_curvature_speed_rate_up_mps2_ : curvature_speed_rate_up_mps2_;
  double raw_limit = safe_nominal_limit;
  if (curvature_speed_enabled_ && lateral_accel_limit > 0.0 &&
      last_path_curvature_for_speed_limit_ > curvature_deadband_inv_m_) {
    raw_limit = std::sqrt(lateral_accel_limit / last_path_curvature_for_speed_limit_);
  }
  double capacity = std::max(0.0, speed_upper_nominal_);
  const double profile_ceiling = is_highway_mode ? speed_limit_highway_max_mps_ : speed_limit_urban_max_mps_;
  if (profile_ceiling > 0.0) capacity = std::min(capacity, profile_ceiling);
  if (curvature_max_speed_mps_ > 0.0) capacity = std::min(capacity, curvature_max_speed_mps_);
  if (curvature_speed_enabled_ && lateral_accel_limit > 0.0 &&
      last_path_curvature_for_speed_limit_ > curvature_deadband_inv_m_)
    capacity = std::min(capacity, std::max(m2_avoidance ? avoidance_curvature_min_speed_mps_ : curvature_min_speed_mps_,
                                           std::sqrt(lateral_accel_limit / last_path_curvature_for_speed_limit_)));
  publishFloat64(curvature_capacity_speed_pub_, capacity);
  raw_limit = clampValue(raw_limit, min_limit, safe_nominal_limit);

  if (!have_filtered_curve_speed_limit_) {
    filtered_curve_speed_limit_mps_ = raw_limit;
    have_filtered_curve_speed_limit_ = true;
  } else {
    const double up_step = std::max(0.0, curve_rate_up) * dt;
    const double down_step = std::max(0.0, curvature_speed_rate_down_mps2_) * dt;
    filtered_curve_speed_limit_mps_ =
        clampValue(raw_limit, filtered_curve_speed_limit_mps_ - down_step, filtered_curve_speed_limit_mps_ + up_step);
    filtered_curve_speed_limit_mps_ = clampValue(filtered_curve_speed_limit_mps_, min_limit, safe_nominal_limit);
  }
  last_curve_speed_limit_mps_ = curvature_speed_enabled_ ? filtered_curve_speed_limit_mps_ : safe_nominal_limit;
  publishFloat64(curvature_speed_limit_pub_, last_curve_speed_limit_mps_);
  publishFloat64(path_curvature_for_speed_limit_pub_, last_path_curvature_for_speed_limit_);
  return last_curve_speed_limit_mps_;
}

// =====================================================================
// MCU 호출
// =====================================================================
bool BmpcBridge::ensureConfigured()
{
  if (configured_) return true;
  const ros::WallTime now = ros::WallTime::now();
  if (!last_config_try_.isZero() && (now - last_config_try_).toSec() < 1.0) return false;
  last_config_try_ = now;
  std::string msg;
  configured_ = link_->configure(mcu_cfg_, config_timeout_s_, &msg);
  if (configured_) {
    ROS_INFO("[BMPC] %s (%d fields)", msg.c_str(), (int)BMPC_CONFIG_COUNT);
    reset_next_ = true;
    if (safety_enabled_ && safety_reset_on_start_ && !safety_reset_sent_) {
      // 안전 제어기의 SAFE_STOP 은 래치된다 (이전 시험이 끝나며 명령이 끊기면 걸려 있다). 새 주행을 시작할 때 한 번만 푼다.
      // BENCH_RESET 은 시험대 전용 프레임이다 (실차에서는 운전자 확인 후 시동으로 해제).
      bmpc_can_frame_t f;
      std::memset(&f, 0, sizeof(f));
      f.id = bmpc_bridge::vcan::ID_BENCH_RESET;
      f.dlc = 2;
      f.data[0] = 0x5A;
      f.data[1] = 0xA5;
      link_->sendRaw(f);
      safety_reset_sent_ = true;
      ROS_WARN("[BMPC] safety: BENCH_RESET sent once at start (clears a latched SAFE_STOP from a previous run)");
    }
  } else {
    ROS_WARN_THROTTLE(5.0, "[BMPC] MCU not configured: %s", msg.c_str());
  }
  return configured_;
}

// 원본 지연보상 블록의 시간 측정 부분 (계산은 MCU)
void BmpcBridge::fillTiming(bmpc_request_t &req)
{
  const ros::Time now_t = ros::Time::now();
  double period = dc_period_s_;
  if (period <= 0.0 || dc_last_imu_rx_.isZero() || (now_t - dc_last_imu_rx_).toSec() > 0.5) period = dc_fallback_period_s_;
  req.period = clampValue(period, 0.005, 0.2);
  req.odom_age = last_odom_stamp_.isZero() ? 0.0 : clampValue((now_t - last_odom_stamp_).toSec(), 0.0, 0.2);
  req.transport = link_->transportEstimate() + inject_cmd_delay_s_;
  req.yaw_rate = odom_yaw_rate_;
  req.lf_est = dc_lf_est_ > 0.0 ? dc_lf_est_ : 0.0;
}

// 요청을 보내고 응답(또는 대체 명령)을 /ctrl_cmd 로 낸다. 원본 publishTargetSpeed + publishControlCommand 자리.
void BmpcBridge::runMcu(bmpc_request_t &req, bool stop)
{
  if (stop) req.flags |= BMPC_FLAG_STOP;
  if (reset_next_) req.flags |= BMPC_FLAG_RESET;
  fillTiming(req);

  bmpc_response_t resp;
  std::memset(&resp, 0, sizeof(resp));
  const ros::WallTime now_w = ros::WallTime::now();
  if (now_w < inject_busoff_until_) {
    // 결함 주입 (버스오프, 팀 F8a 방식): 어댑터를 잘못된 비트레이트(125 kbps)로 바꿔 계속 보낸다.
    // MCU 가 보내는 프레임마다 비트 오류가 나 송신 오류 카운터가 올라가고 버스오프가 된다.
    if (!busoff_active_) {
      busoff_active_ = link_->setBitrate(125000);
      injectLog(busoff_active_ ? "bus_off adapter at 125 kbps" : "bus_off: adapter cannot change bitrate");
    }
    bmpc_can_frame_t junk;
    std::memset(&junk, 0, sizeof(junk));
    junk.id = 0x050;
    junk.dlc = 8;
    std::memset(junk.data, 0x55, 8);
    const ros::WallTime t_end = now_w + ros::WallDuration(0.04);
    while (ros::WallTime::now() < t_end) link_->sendRaw(junk);
  } else if (busoff_active_) {
    link_->setBitrate(500000);
    busoff_active_ = false;
    injectLog("bus_off end: adapter back at 500 kbps");
  }
  const bool blackout = now_w < inject_blackout_until_ || busoff_active_;
  bmpc_bridge::McuLink::Result r = bmpc_bridge::McuLink::TIMEOUT;
  if (busoff_active_) {
    // 버스오프 주입 중에는 받지 않는다 (어댑터 비트레이트가 다르다)
  } else if (blackout) {
    // 결함 주입: 아무것도 보내지 않는다. MCU 가 내는 SAFE_OUT 은 계속 받는다 (안전 제어기 반응을 본다)
    link_->waitSide([]() { return false; }, 0.035);
  } else {
    if (safety_enabled_) {
      sendVehicleCan();
      safe_echo_ = -1;  // Alive 에코는 16 주기마다 같은 값이 돌아오므로 이번 요청 전 값을 지운다
    }
    if (inject_req_crc_n_ > 0) {
      --inject_req_crc_n_;
      link_->corruptNextRequest();
      injectLog("req_crc frame sent");
    }
    r = link_->exchange(req, &resp, response_timeout_s_);
  }
  const bool ok = r == bmpc_bridge::McuLink::OK && (resp.status == BMPC_OK || resp.status == BMPC_ERR_MAXITER);
  // 안전 제어기: 이번 MPC 명령이 반영된 SAFE_OUT (SAFE_STAT 의 Alive 에코 = 요청 seq 하위 4 비트) 을 기다린다
  bool safe_matched = false;
  double safe_wait_ms = 0.0;
  if (safety_enabled_ && r == bmpc_bridge::McuLink::OK && !blackout) {
    const int want = req.seq & 0x0F;
    const ros::WallTime w0 = ros::WallTime::now();
    safe_matched = link_->waitSide([this, want]() { return safe_echo_ == want; }, safety_wait_s_);
    safe_wait_ms = (ros::WallTime::now() - w0).toSec() * 1e3;
    if (!safe_matched) ++safe_echo_miss_;
  }

  double steer, accel, brake;
  bool fallback = false;
  if (ok) {
    consecutive_fail_ = 0;
    reset_next_ = false;
    steering_angle = resp.delta0;
    target_velocity = resp.vt_final;
    steer = resp.steer_cmd;
    accel = resp.accel;
    brake = resp.brake;
    have_last_cmd_ = true;
  } else {
    ++consecutive_fail_;
    ++total_fail_;
    reset_next_ = true;
    if (r == bmpc_bridge::McuLink::OK && resp.status == BMPC_ERR_NOT_CONFIGURED) configured_ = false;
    ROS_WARN_THROTTLE(1.0, "[BMPC] MCU %s (status %d), fallback %d cycles, total fails %lu",
                      r == bmpc_bridge::McuLink::TIMEOUT ? "timeout" : (r == bmpc_bridge::McuLink::RX_ERROR ? "rx error" : "error"),
                      (int)resp.status, consecutive_fail_, (unsigned long)total_fail_);
    if (!have_last_cmd_) return;  // 아직 한 번도 명령을 낸 적이 없으면 아무것도 내지 않는다
    fallback = true;
    steer = last_steer_;
    if (consecutive_fail_ <= fallback_hold_cycles_) {
      accel = last_accel_;
      brake = last_brake_;
    } else {
      // 계속 응답이 없으면 액셀 0, 브레이크를 원본 증가율로 올려 세운다
      accel = 0.0;
      brake = std::min((double)mcu_cfg_.lg_brake_limit, last_brake_ + (double)mcu_cfg_.lg_brake_rate * dt);
    }
  }
  const double steer_mpc = steer, accel_mpc = accel, brake_mpc = brake;
  if (!safety_enabled_) {
    // 조향 명령 결함 (안전 제어기 끔 비교용). 시간은 다음 명령부터 잰다: MCU 훅과 같다
    const ros::WallTime wall_now = ros::WallTime::now();
    if (inject_ctrl_pending_) {
      inject_ctrl_pending_ = false;
      inject_ctrl_active_ = true;
      inject_ctrl_until_ = wall_now + ros::WallDuration(inject_ctrl_ms_ * 1e-3);
    }
    if (inject_ctrl_active_ && wall_now >= inject_ctrl_until_) inject_ctrl_active_ = false;
    if (inject_ctrl_active_) steer = inject_ctrl_hold_ ? inject_ctrl_last_steer_ : steer + inject_ctrl_offset_rad_;
    else inject_ctrl_last_steer_ = steer;
  }
  bool safe_used = false;
  if (safety_enabled_ && have_safe_out_ && (ros::Time::now() - safe_out_rx_).toSec() < 0.1) {
    // 차량에는 안전 제어기가 검사한 값이 간다. 같은 gain 으로 페달로 되돌린다 (안전 제어기가 바꾸지 않았으면 MPC 값과 같다)
    steer = safe_out_.steer_deg * M_PI / 180.0;
    accel = safe_out_.accel_mps2 >= 0.0 ? std::min(1.0, safe_out_.accel_mps2 / safety_accel_gain_) : 0.0;
    brake = safe_out_.accel_mps2 < 0.0 ? std::min(1.0, -safe_out_.accel_mps2 / safety_brake_gain_) : 0.0;
    safe_used = true;
  }
  last_steer_ = steer;
  last_accel_ = accel;
  last_brake_ = brake;

  if (ok) publishTargetSpeed(target_velocity);

  morai_msgs::CtrlCmd cmd;
  cmd.longlCmdType = 1;
  cmd.steering = steer;
  cmd.accel = accel;
  cmd.brake = brake;
  cmd.velocity = 0.0;
  cmd.acceleration = 0.0;
  publishCmd(cmd);

  if (ok && !stop && mcu_cfg_.dc_enabled) {
    std_msgs::Float64MultiArray dbg;
    dbg.data = {req.period, req.odom_age, (double)resp.state_delay, (double)resp.compute, (double)resp.actuation, 1.0,
                (double)resp.lf_used};
    dc_debug_pub_.publish(dbg);
    publishFloat64(steering_delay_pub_, resp.state_delay + resp.compute + resp.actuation);
  }

  const bmpc_bridge::LinkStats &ls = link_->stats();
  std_msgs::Float64MultiArray d;
  d.data = {(double)req.seq, (double)resp.status, (double)r, ls.rtt_last * 1e3, resp.total_us * 1e-3,
            resp.solve_us * 1e-3, link_->transportEstimate() * 1e3, (double)resp.iters, (double)resp.delta0,
            (double)resp.vt_raw, (double)resp.vt_final, steer, accel, brake, (double)resp.state_delay,
            (double)resp.compute, (double)resp.actuation, (double)resp.lf_used, fallback ? 1.0 : 0.0,
            (double)consecutive_fail_};
  bmpc_debug_pub_.publish(d);

  if (safety_enabled_) {
    // [seq, matched, wait_ms, state, flags(OR), flags2(OR), rx_age_10ms, steer_out_deg, accel_out, steer_mpc_deg, accel_mpc(m/s2),
    //  used, crc_err, echo_miss, inputs_fresh, lat_acc_meas, bus_off, SAFE_OUT 수(지난 발행 뒤), OutAlive 건너뜀 누적]
    std_msgs::Float64MultiArray sd;
    sd.data = {(double)req.seq, safe_matched ? 1.0 : 0.0, safe_wait_ms, (double)safe_out_.state, (double)safe_flags_or_,
               (double)safe_flags2_or_, (double)safe_out_.rx_age_10ms, safe_out_.steer_deg, safe_out_.accel_mps2,
               steer_mpc * 180.0 / M_PI, accel_mpc * safety_accel_gain_ - brake_mpc * safety_brake_gain_, safe_used ? 1.0 : 0.0,
               (double)safe_crc_err_, (double)safe_echo_miss_, (double)safe_stat_.inputs_fresh, safe_stat_.lat_acc_meas,
               (double)safe_out_.bus_off, (double)safe_out_n_, (double)safe_alive_skips_};
    bmpc_safety_pub_.publish(sd);
    safe_flags_or_ = 0;
    safe_flags2_or_ = 0;
    safe_out_n_ = 0;
  }
}

void BmpcBridge::onSideFrame(const bmpc_can_frame_t &f)
{
  if (f.id == bmpc_bridge::vcan::ID_SAFE_OUT) {
    if (bmpc_bridge::vcan::decode_out(f, &safe_out_)) {
      safe_out_rx_ = ros::Time::now();
      have_safe_out_ = true;
      safe_flags_or_ |= safe_out_.flags;
      ++safe_out_n_;
      if (safe_last_alive_ >= 0) safe_alive_skips_ += (unsigned long)((safe_out_.alive - safe_last_alive_ - 1) & 0x0F);
      safe_last_alive_ = safe_out_.alive;
      if (safe_out_.state != last_safe_state_) {  // 안전 상태가 바뀐 시각 (SAFE_OUT 수신 시각, 10 ms 해상도)
        char b[96];
        std::snprintf(b, sizeof(b), "safety_state %d -> %d flags 0x%02x", last_safe_state_, safe_out_.state, safe_out_.flags);
        injectLog(b);
        last_safe_state_ = safe_out_.state;
      }
    } else {
      ++safe_crc_err_;
    }
  } else if (f.id == 0x3F4 && f.dlc == 8) {
    // MCU 안전 주기 진단: 누적 sc_step 횟수, 지난 0.5 s 간격 최대 (10 us), 실행 시간 최대 (us), 버스오프 횟수
    std_msgs::Float64MultiArray d;
    const uint32_t steps = f.data[0] | (f.data[1] << 8) | (f.data[2] << 16) | ((uint32_t)f.data[3] << 24);
    d.data = {(double)steps, (f.data[4] | (f.data[5] << 8)) * 0.01, (double)f.data[6], (double)f.data[7]};
    mcu_diag_pub_.publish(d);
  } else if (f.id == bmpc_bridge::vcan::ID_SAFE_STAT) {
    if (bmpc_bridge::vcan::decode_stat(f, &safe_stat_)) {
      safe_echo_ = safe_stat_.ctrl_alive_echo;
      safe_flags2_or_ |= safe_stat_.flags2;
    }
    else ++safe_crc_err_;
  }
}

// 차량 CAN 4 개 (요청 직전, 매 주기): MORAI 상태를 ESC·조향각 센서·VCU·IMU 가 낸 것처럼 만든다 (팀 vehicle_can.py 와 같은 원천)
void BmpcBridge::sendVehicleCan()
{
  if (ros::WallTime::now() < inject_veh_loss_until_) return;  // 결함 주입: 차량 CAN 두절
  bmpc_can_frame_t esc = vcan_enc_.esc(competition_speed_mps_, comp_long_acc_, comp_lat_acc_);
  if (inject_veh_crc_n_ > 0) {
    --inject_veh_crc_n_;
    esc.data[0] ^= 0x01;  // CRC 는 그대로 두고 값만 바꾼다 -> 안전 제어기가 E2E 로 걸러야 한다
    injectLog("veh_crc ESC frame sent");
  }
  link_->sendRaw(esc);
  link_->sendRaw(vcan_enc_.sas(competition_wheel_angle_deg_));
  // 운전자 페달: 시뮬레이터에는 운전자가 없으므로 0 (MORAI 의 accel/brake 는 우리 명령이 반영된 값이라 운전자 입력이 아니다).
  // 운전자 개입은 결함 주입 "driver_brake <0..1> <s>" 로만 넣는다 (TSR-10)
  const double drv_brake = ros::WallTime::now() < inject_driver_until_ ? inject_driver_brake_ : 0.0;
  link_->sendRaw(vcan_enc_.pedal(0.0, drv_brake));
  link_->sendRaw(vcan_enc_.imu(imu_yaw_rate_dps_, imu_long_acc_, imu_lat_acc_));
}

// =====================================================================
// 한 주기 (원본 modelPredictiveController 의 PC 부분)
// =====================================================================
void BmpcBridge::step()
{
  if (!ensureConfigured()) {
    is_pose_ = false;
    is_course_ = false;
    return;
  }

  const bool have_pose_and_course = is_pose_ && is_course_;
  if (have_pose_and_course && (!is_lane_ || waypoints_size_ < path_end_min_polyfit_points_)) {
    if (path_end_stop_enabled_) {
      // 원본 publishPathEndStop(): 목표 0, 직전 조향 유지. 후처리는 MCU 가 한다.
      bmpc_request_t req;
      std::memset(&req, 0, sizeof(req));
      req.post_cap = -1;
      req.snap = -1;
      req.velocity = have_competition_feedback_ ? competition_speed_mps_ : cur_speed_;
      if (determineIsHighway()) req.flags |= BMPC_FLAG_HIGHWAY;
      if (static_speed_scale_was_active_) req.flags |= BMPC_FLAG_KICK;
      target_velocity = 0.0;
      runMcu(req, true);
      publishTargetSpeed(0.0);
      ROS_WARN_THROTTLE(1.0, "[BMPC] path-end stop (%s): /local_path points=%d, required >=%d for cubic polyfit",
                        is_lane_ ? "too_few_local_path_points" : "no_local_path", waypoints_size_,
                        path_end_min_polyfit_points_);
    } else {
      ROS_WARN_THROTTLE(1.0, "[BMPC] skip control: /local_path points=%d, required >=%d for cubic polyfit",
                        waypoints_size_, path_end_min_polyfit_points_);
    }
    is_pose_ = false;
    is_course_ = false;
    return;
  }

  if (!(is_pose_ && is_course_ && is_lane_ && waypoints_size_ >= path_end_min_polyfit_points_)) {
    is_pose_ = false;
    is_course_ = false;
    return;
  }

  double px = ex_x_;
  double py = ex_y_;
  double psi = cur_course_;
  const double velocity = have_competition_feedback_ ? competition_speed_mps_ : cur_speed_;
  const double dt_ = 0.05;

  // ---- 입력 위치 필터 (원본 그대로) ----
  if (sf_enabled_) {
    const double stamp = last_odom_stamp_.toSec();
    const double dt_f = stamp - sf_last_stamp_;
    const double ekf_yaw = psi;
    bool reset = !sf_init_ || gps_mode_ != "normal" || ekf_vx_ < 0.3 || dt_f < 0.0 || dt_f > 0.5;
    if (!reset && dt_f > 1e-4) {
      const double yaw_mid = sf_yaw_ + 0.5 * odom_yaw_rate_ * dt_f;
      sf_x_ += velocity * std::cos(yaw_mid) * dt_f;
      sf_y_ += velocity * std::sin(yaw_mid) * dt_f;
      sf_yaw_ = wrapToPi(sf_yaw_ + odom_yaw_rate_ * dt_f);
      const double s = clampValue(sf_straightness_, 0.0, 1.0);
      const double tau_p = sf_pos_tau_s_ * s;
      const double tau_y = sf_yaw_tau_s_ * s;
      const double a_p = tau_p > 1e-3 ? 1.0 - std::exp(-dt_f / tau_p) : 1.0;
      const double a_y = tau_y > 1e-3 ? 1.0 - std::exp(-dt_f / tau_y) : 1.0;
      const double c = std::cos(sf_yaw_), sn = std::sin(sf_yaw_);
      const double ex = px - sf_x_, ey = py - sf_y_;
      const double d_along = ex * c + ey * sn;
      const double d_lat = -ex * sn + ey * c;
      sf_x_ += d_along * c - a_p * d_lat * sn;
      sf_y_ += d_along * sn + a_p * d_lat * c;
      sf_yaw_ = wrapToPi(sf_yaw_ + a_y * wrapToPi(ekf_yaw - sf_yaw_));
      {
        const double dyaw = wrapToPi(ekf_yaw - sf_yaw_);
        if (std::fabs(dyaw) > sf_max_yaw_rad_) sf_yaw_ = wrapToPi(ekf_yaw - std::copysign(sf_max_yaw_rad_, dyaw));
        const double c2 = std::cos(sf_yaw_), s2 = std::sin(sf_yaw_);
        const double lat = -(px - sf_x_) * s2 + (py - sf_y_) * c2;
        const double over = lat - clampValue(lat, -sf_max_lat_m_, sf_max_lat_m_);
        sf_x_ -= over * s2;
        sf_y_ += over * c2;
      }
      if (std::fabs((1.0 - a_p) * d_lat) > sf_reset_pos_m_ || std::fabs(wrapToPi(ekf_yaw - sf_yaw_)) > sf_reset_yaw_rad_) {
        reset = true;
        ++sf_reset_count_;
      }
    }
    if (reset) {
      sf_x_ = px;
      sf_y_ = py;
      sf_yaw_ = ekf_yaw;
      sf_init_ = true;
    }
    sf_last_stamp_ = stamp;
    std_msgs::Float64MultiArray sf_dbg;
    sf_dbg.data = {sf_straightness_, -(px - sf_x_) * std::sin(sf_yaw_) + (py - sf_y_) * std::cos(sf_yaw_),
                   wrapToPi(ekf_yaw - sf_yaw_) * 180.0 / M_PI, static_cast<double>(sf_reset_count_),
                   gps_mode_ == "normal" ? 1.0 : 0.0};
    sf_debug_pub_.publish(sf_dbg);
    px = sf_x_;
    py = sf_y_;
    psi = sf_yaw_;
  }

  const bool is_highway = determineIsHighway();
  is_highway_mode = is_highway;
  curvature_preview_distance_m_ = is_highway ? preview_distance_highway_m_ : preview_distance_urban_m_;
  {
    std_msgs::Bool hwy_msg;
    hwy_msg.data = is_highway;
    is_highway_mode_pub_.publish(hwy_msg);
  }

  // ---- 3차 polyfit (원본 그대로) ----
  const double kMaxPolyfitDistanceM =
      is_highway ? std::max(24.0, std::min(42.0, velocity * 1.6)) : std::max(12.0, std::min(24.0, velocity * 1.3));
  int fit_points_count = 0;
  double accum_s = 0.0;
  for (int i = 0; i < waypoints_size_; i++) {
    fit_points_count++;
    if (i > 0)
      accum_s += std::hypot(waypoints_[i].pose.position.x - waypoints_[i - 1].pose.position.x,
                            waypoints_[i].pose.position.y - waypoints_[i - 1].pose.position.y);
    if (accum_s >= kMaxPolyfitDistanceM && fit_points_count >= path_end_min_polyfit_points_) break;
  }
  if (fit_points_count < path_end_min_polyfit_points_)
    fit_points_count = std::min(waypoints_size_, path_end_min_polyfit_points_);
  VectorXd vehicle_waypoints_x(fit_points_count), vehicle_waypoints_y(fit_points_count);
  for (int i = 0; i < fit_points_count; i++) {
    const double diff_x = waypoints_[i].pose.position.x - px;
    const double diff_y = waypoints_[i].pose.position.y - py;
    vehicle_waypoints_x[i] = diff_x * cos(-psi) - diff_y * sin(-psi);
    vehicle_waypoints_y[i] = diff_y * cos(-psi) + diff_x * sin(-psi);
  }
  const VectorXd coeffs = polyfit(vehicle_waypoints_x, vehicle_waypoints_y, 3);

  VectorXd preview_coeffs = coeffs;
  VectorXd preview_x = vehicle_waypoints_x;
  {
    const double kPreviewFitDistanceM = std::max(kMaxPolyfitDistanceM, std::min(60.0, velocity * 2.5));
    if (kPreviewFitDistanceM > kMaxPolyfitDistanceM + 1e-3) {
      int pv_count = 0;
      double pv_s = 0.0;
      for (int i = 0; i < waypoints_size_; i++) {
        pv_count++;
        if (i > 0)
          pv_s += std::hypot(waypoints_[i].pose.position.x - waypoints_[i - 1].pose.position.x,
                             waypoints_[i].pose.position.y - waypoints_[i - 1].pose.position.y);
        if (pv_s >= kPreviewFitDistanceM && pv_count >= path_end_min_polyfit_points_) break;
      }
      if (pv_count > fit_points_count && pv_count >= path_end_min_polyfit_points_) {
        VectorXd pvx(pv_count), pvy(pv_count);
        for (int i = 0; i < pv_count; i++) {
          const double dx = waypoints_[i].pose.position.x - px;
          const double dy = waypoints_[i].pose.position.y - py;
          pvx[i] = dx * cos(-psi) - dy * sin(-psi);
          pvy[i] = dy * cos(-psi) + dx * sin(-psi);
        }
        preview_coeffs = polyfit(pvx, pvy, 3);
        preview_x = pvx;
      }
    }
  }

  if (sf_enabled_) {
    const double x_end = std::max(1.0, std::min(std::max(1.0, velocity) * sf_preview_s_,
                                                 preview_x.size() > 0 ? preview_x.maxCoeff() : 1.0));
    double ksum = 0.0;
    const int n = 12;
    for (int i = 0; i <= n; ++i) {
      const double xs = x_end * i / n;
      const double d1 = preview_coeffs[1] + 2.0 * preview_coeffs[2] * xs + 3.0 * preview_coeffs[3] * xs * xs;
      const double d2 = 2.0 * preview_coeffs[2] + 6.0 * preview_coeffs[3] * xs;
      ksum += std::fabs(d2) / std::pow(1.0 + d1 * d1, 1.5);
    }
    const double kmean = ksum / (n + 1);
    const double s_k = clampValue((sf_kappa_zero_ - kmean) / std::max(1e-6, sf_kappa_zero_ - sf_kappa_full_), 0.0, 1.0);
    const double s_w =
        clampValue((sf_wz_zero_ - std::fabs(odom_yaw_rate_)) / std::max(1e-6, sf_wz_zero_ - sf_wz_full_), 0.0, 1.0);
    const double now_s = ros::Time::now().toSec();
    const double dt_s = sf_s_last_t_ > 0.0 ? clampValue(now_s - sf_s_last_t_, 0.0, 0.5) : 0.0;
    sf_s_last_t_ = now_s;
    sf_straightness_ = std::min(s_k * s_w, sf_straightness_ + std::max(0.0, sf_s_up_rate_) * dt_s);
  }

  // ---- 속도 제한 체인 (원본 그대로) ----
  const double ref_v_saved = ref_v;
  const double speed_ub_saved = speed_upperBound;
  const double speed_lb_saved = speed_lowerBound;
  auto restore = [&]() {
    ref_v = ref_v_saved;
    speed_upperBound = speed_ub_saved;
    speed_lowerBound = speed_lb_saved;
  };

  const double current_max_speed_limit = is_highway ? speed_limit_highway_max_mps_ : speed_limit_urban_max_mps_;
  if (current_max_speed_limit > 0.0) {
    ref_v = std::min(ref_v, current_max_speed_limit);
    speed_upperBound = std::min(speed_upperBound, current_max_speed_limit);
    if (speed_lowerBound > speed_upperBound) speed_lowerBound = speed_upperBound;
  }

  const double raw_mission_speed_target_mps = missionStateTargetSpeedMps();
  const double actual_speed_now = velocity;
  if (!have_filtered_mission_speed_target_) {
    filtered_mission_speed_target_mps_ = std::max(0.0, actual_speed_now);
    if (filtered_mission_speed_target_mps_ < 1.0)
      filtered_mission_speed_target_mps_ = std::min(raw_mission_speed_target_mps, 3.0);
    have_filtered_mission_speed_target_ = true;
  } else {
    if (actual_speed_now < 1.0 && filtered_mission_speed_target_mps_ > actual_speed_now + 3.0)
      filtered_mission_speed_target_mps_ = actual_speed_now + 3.0;
    const double up_step = std::max(0.0, mission_speed_rate_limit_up_mps2_) * dt_;
    const double down_step = std::max(0.0, mission_speed_rate_limit_down_mps2_) * dt_;
    filtered_mission_speed_target_mps_ = clampValue(raw_mission_speed_target_mps,
                                                    filtered_mission_speed_target_mps_ - down_step,
                                                    filtered_mission_speed_target_mps_ + up_step);
  }
  const double mission_speed_target_mps = filtered_mission_speed_target_mps_;
  publishFloat64(mission_speed_target_pub_, mission_speed_target_mps);
  if (mission_speed_enabled_) {
    ref_v = mission_speed_target_mps;
    speed_upperBound = std::min(speed_upperBound, mission_speed_target_mps);
    if (speed_lowerBound > speed_upperBound) speed_lowerBound = speed_upperBound;
  }

  const double curve_speed_limit_mps = computeCurvatureSpeedLimit(preview_coeffs, preview_x);
  if (curvature_speed_enabled_) {
    ref_v = std::min(ref_v, curve_speed_limit_mps);
    speed_upperBound = std::min(speed_upperBound, curve_speed_limit_mps);
    if (speed_lowerBound > speed_upperBound) speed_lowerBound = speed_upperBound;
  }

  bool applied_avoidance_speed_limit = false;
  if (have_avoidance_speed_ && (ros::Time::now() - last_avoidance_speed_time_).toSec() < 0.5 &&
      avoidance_speed_limit_mps_ < 500.0) {
    ref_v = std::min(ref_v, avoidance_speed_limit_mps_);
    speed_upperBound = std::min(speed_upperBound, avoidance_speed_limit_mps_);
    if (speed_lowerBound > speed_upperBound) speed_lowerBound = speed_upperBound;
    applied_avoidance_speed_limit = true;
  }
  is_avoidance_mode = applied_avoidance_speed_limit;

  bool applied_static_speed_scale = false;
  double static_speed_scale_target_mps = 0.0;
  if (have_static_speed_scale_ && (ros::Time::now() - last_static_speed_scale_time_).toSec() < 0.5 &&
      static_speed_scale_ < 500.0) {
    ref_v = std::min(ref_v, static_speed_scale_);
    speed_upperBound = std::min(speed_upperBound, static_speed_scale_);
    if (speed_lowerBound > speed_upperBound) speed_lowerBound = speed_upperBound;
    applied_static_speed_scale = true;
    static_speed_scale_target_mps = ref_v;
  }

  // ACC 0 이면 원본은 바로 정지 명령을 냈다: 같은 일을 MCU 가 하도록 STOP 요청을 보낸다
  auto acc_stop = [&]() {
    bmpc_request_t req;
    std::memset(&req, 0, sizeof(req));
    req.post_cap = -1;
    req.snap = -1;
    req.velocity = velocity;
    if (determineIsHighway()) req.flags |= BMPC_FLAG_HIGHWAY;
    if (static_speed_scale_was_active_) req.flags |= BMPC_FLAG_KICK;
    target_velocity = 0.0;
    publishTargetSpeed(0.0);
    restore();
    runMcu(req, true);
    is_pose_ = false;
    is_course_ = false;
  };

  const bool external_acc_fresh = external_acc_enabled_ && have_external_acc_target_ &&
                                  (ros::Time::now() - external_acc_target_stamp_).toSec() <= external_acc_timeout_sec_;
  if (external_acc_fresh) {
    const double acc_ms = std::max(0.0, external_acc_target_speed_mps_);
    if (acc_ms < 0.01) {
      acc_stop();
      return;
    }
    ref_v = std::min(ref_v, acc_ms);
    speed_upperBound = std::min(speed_upperBound, acc_ms);
    speed_lowerBound = 0.0;
  }
  if (is_acc) {
    double spd_state_kmh = cur_speed_ * 3.6;
    spd_state_kmh = std::round(spd_state_kmh / 2.0) * 2.0;
    if (spd_state_kmh < 12.0) spd_state_kmh = 12.0;
    if (spd_state_kmh > 20.0) spd_state_kmh = 20.0;
    double acc_kmh = applyACC_kmh(spd_state_kmh, true, acc_index);
    if (acc_kmh > 20.0) acc_kmh = 20.0;
    if (acc_kmh < 0.01) acc_kmh = 0.0;
    const double acc_ms = acc_kmh / 3.6;
    if (acc_ms < 0.01) {
      acc_stop();
      return;
    }
    ref_v = std::min(ref_v, acc_ms);
    speed_upperBound = std::min(speed_upperBound, acc_ms);
    speed_lowerBound = 0.0;
  }

  // ---- MCU 요청 ----
  bmpc_request_t req;
  std::memset(&req, 0, sizeof(req));
  req.ref_type = BMPC_REF_CUBIC;
  for (int i = 0; i < 4; ++i) req.c[i] = coeffs[i];
  req.x_max = vehicle_waypoints_x.size() > 0 ? std::max(0.0, vehicle_waypoints_x.maxCoeff()) : 0.0;
  req.path_counter = 0;
  req.ref_v = ref_v;
  req.v_lb = speed_lowerBound;
  req.v_ub = speed_upperBound;
  req.velocity = velocity;
  if (is_highway) req.flags |= BMPC_FLAG_HIGHWAY;
  if (applied_avoidance_speed_limit) req.flags |= BMPC_FLAG_AVOIDANCE;

  // 속도 후처리 값: 회피·정적 장애물이 켜진 첫 주기는 바로 맞춘다(정적이 나중이라 덮어쓴다)
  req.snap = -1.0;
  if (applied_avoidance_speed_limit && !avoidance_speed_limit_was_active_) req.snap = avoidance_speed_limit_mps_;
  avoidance_speed_limit_was_active_ = applied_avoidance_speed_limit;
  if (applied_static_speed_scale && !static_speed_scale_was_active_) req.snap = static_speed_scale_target_mps;
  static_speed_scale_was_active_ = applied_static_speed_scale;
  if (static_speed_scale_was_active_) req.flags |= BMPC_FLAG_KICK;

  // 구간 상한과 경로 끝 감속을 합친 후처리 속도 상한
  double post_cap = 1e9;
  if (current_max_speed_limit > 0.0) post_cap = std::min(post_cap, current_max_speed_limit);
  if (path_end_stop_enabled_ && waypoints_size_ >= path_end_min_polyfit_points_) {
    double remaining_s = 0.0;
    for (int i = 1; i < waypoints_size_; i++)
      remaining_s += std::hypot(waypoints_[i].pose.position.x - waypoints_[i - 1].pose.position.x,
                                waypoints_[i].pose.position.y - waypoints_[i - 1].pose.position.y);
    if (remaining_s < path_end_approach_trigger_m_) {
      const double brake_s = std::max(0.0, remaining_s - path_end_stop_margin_m_);
      const double v_allow = std::sqrt(2.0 * std::max(0.1, path_end_approach_decel_mps2_) * brake_s);
      post_cap = std::min(post_cap, v_allow);
      ROS_WARN_THROTTLE(0.5, "[BMPC] path-end approach: remaining=%.1f m -> v_allow=%.1f m/s", remaining_s, v_allow);
    }
  }
  req.post_cap = post_cap < 1e8 ? post_cap : -1.0;

  restore();
  runMcu(req, false);

  ROS_INFO_THROTTLE(1.0, "[BMPC %s] v=%.1f m/s (tgt=%.1f) steer=%.3f rad | rtt=%.2f ms, transport=%.2f ms | fails=%lu",
                    is_highway ? "HIGHWAY" : "URBAN", velocity, target_velocity, steering_angle,
                    link_->stats().rtt_last * 1e3, link_->transportEstimate() * 1e3, (unsigned long)total_fail_);

  is_pose_ = false;
  is_course_ = false;
}

void BmpcBridge::spinLoop()
{
  double rate_hz = 20.0;
  pnh_.param<double>("loop_rate_hz", rate_hz, 20.0);
  ros::Rate loop_rate(rate_hz);
  while (ros::ok()) {
    ros::spinOnce();
    const ros::WallTime t0 = ros::WallTime::now();
    step();
    publishFloat64(controller_time_pub_, (ros::WallTime::now() - t0).toSec() * 1000.0);
    loop_rate.sleep();
  }
  const bmpc_bridge::LinkStats &s = link_->stats();
  ROS_INFO("[BMPC] exchanges=%lu ok=%lu timeouts=%lu rx_errors=%lu stale=%lu rtt mean %.3f ms max %.3f ms",
           (unsigned long)s.exchanges, (unsigned long)s.ok, (unsigned long)s.timeouts, (unsigned long)s.rx_errors,
           (unsigned long)s.stale_frames, s.ok ? s.rtt_sum / s.ok * 1e3 : 0.0, s.rtt_max * 1e3);
}

void BmpcBridge::publishCmd(const morai_msgs::CtrlCmd &cmd)
{
  if (inject_cmd_delay_s_ <= 0.0) {
    ackermann_pub_.publish(cmd);
    return;
  }
  {
    std::lock_guard<std::mutex> lk(pub_mu_);
    pub_q_.emplace_back(std::chrono::steady_clock::now() +
                            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                std::chrono::duration<double>(inject_cmd_delay_s_)),
                        cmd);
  }
  pub_cv_.notify_all();
}

void BmpcBridge::publisherThread()
{
  std::unique_lock<std::mutex> lk(pub_mu_);
  while (!pub_stop_) {
    if (pub_q_.empty()) {
      pub_cv_.wait(lk);
      continue;
    }
    const auto due = pub_q_.front().first;
    if (std::chrono::steady_clock::now() < due) {
      pub_cv_.wait_until(lk, due);
      continue;
    }
    const morai_msgs::CtrlCmd cmd = pub_q_.front().second;
    pub_q_.pop_front();
    lk.unlock();
    ackermann_pub_.publish(cmd);
    lk.lock();
  }
}

void BmpcBridge::injectLog(const std::string &s)
{
  std_msgs::String m;
  m.data = s;
  inject_log_pub_.publish(m);
  ROS_WARN("[BMPC inject] %s", s.c_str());
}

void BmpcBridge::InjectCallback(const std_msgs::String::ConstPtr &msg)
{
  char kind[32] = {0};
  double val = 0.0;
  const int n = std::sscanf(msg->data.c_str(), "%31s %lf", kind, &val);
  if (n < 1) return;
  const std::string k(kind);
  const ros::WallTime now = ros::WallTime::now();
  if (k == "blackout") inject_blackout_until_ = now + ros::WallDuration(val);
  else if (k == "veh_loss") inject_veh_loss_until_ = now + ros::WallDuration(val);
  else if (k == "bus_off") inject_busoff_until_ = now + ros::WallDuration(val);
  else if (k == "req_crc") inject_req_crc_n_ = (int)val;
  else if (k == "driver_brake") {
    double dur = 1.0;
    std::sscanf(msg->data.c_str(), "%*s %*f %lf", &dur);
    inject_driver_brake_ = val;
    inject_driver_until_ = now + ros::WallDuration(dur);
  }
  else if (k == "veh_crc") inject_veh_crc_n_ = (int)val;
  else if (k == "ctrl_offset" || k == "ctrl_hold") {
    // 조향 명령 결함. 안전 제어기를 쓰면 MCU 안에서 CTRL_CMD 에 넣고 (펌웨어 FAULT_HOOK=1), 안 쓰면 여기서 같은 값을 넣는다
    double deg = 0.0, ms = val;
    if (k == "ctrl_offset") {
      deg = val;
      std::sscanf(msg->data.c_str(), "%*s %*f %lf", &ms);
    }
    if (safety_enabled_) {
      bmpc_can_frame_t f;
      const int16_t centideg = (int16_t)std::lround(deg * 100.0);
      const uint16_t duration = (uint16_t)std::lround(ms);
      std::memset(&f, 0, sizeof(f));
      f.id = bmpc_bridge::vcan::ID_BENCH_RESET;
      f.dlc = 6;
      f.data[0] = 0xFA;
      f.data[1] = k == "ctrl_offset" ? 1 : 2;
      f.data[2] = (uint8_t)((uint16_t)centideg & 0xFF);
      f.data[3] = (uint8_t)((uint16_t)centideg >> 8);
      f.data[4] = (uint8_t)(duration & 0xFF);
      f.data[5] = (uint8_t)(duration >> 8);
      link_->sendRaw(f);
    } else {
      inject_ctrl_hold_ = k == "ctrl_hold";
      inject_ctrl_offset_rad_ = deg * M_PI / 180.0;
      inject_ctrl_ms_ = ms;
      inject_ctrl_pending_ = true;
    }
  }
  else if (k == "solve_fail") {
    // 시험대: MCU 의 MPC 풀이를 val ms 동안 실패(status 3)로 만든다 (FAULT_HOOK 빌드, 코어 BMPC_TEST_HOOK). SWR-45 시험
    if (!safety_enabled_) {
      ROS_WARN("[BMPC inject] solve_fail needs safety:=true and a FAULT_HOOK firmware");
      return;
    }
    const uint16_t duration = (uint16_t)std::lround(std::max(0.0, std::min(val, 60000.0)));
    bmpc_can_frame_t f;
    std::memset(&f, 0, sizeof(f));
    f.id = bmpc_bridge::vcan::ID_BENCH_RESET;
    f.dlc = 6;
    f.data[0] = 0xFA;
    f.data[1] = 4;
    f.data[4] = (uint8_t)(duration & 0xFF);
    f.data[5] = (uint8_t)(duration >> 8);
    link_->sendRaw(f);
  }
  else if (k == "wdt_hang") {
    // 시험대: 펌웨어 메인 루프를 멈춰 워치독 복귀를 시험한다 (FAULT_HOOK + SWT 빌드)
    bmpc_can_frame_t f;
    std::memset(&f, 0, sizeof(f));
    f.id = bmpc_bridge::vcan::ID_BENCH_RESET;
    f.dlc = 6;
    f.data[0] = 0xFA;
    f.data[1] = 3;
    link_->sendRaw(f);
  }
  else if (k == "safety_reset") {
    bmpc_can_frame_t f;
    std::memset(&f, 0, sizeof(f));
    f.id = bmpc_bridge::vcan::ID_BENCH_RESET;
    f.dlc = 2;
    f.data[0] = 0x5A;
    f.data[1] = 0xA5;
    link_->sendRaw(f);
  } else {
    ROS_WARN("[BMPC inject] unknown: %s", msg->data.c_str());
    return;
  }
  injectLog("start " + msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bmpc_bridge");
  BmpcBridge bridge;
  if (!bridge.init()) return 1;
  bridge.spinLoop();
  return 0;
}
