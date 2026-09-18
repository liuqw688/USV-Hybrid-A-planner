#pragma once
#include "hybrid_a_star_planner/colregs.hpp"
#include <limits>

namespace hybrid_a_star_planner {
constexpr double deg = kPi / 180;
enum class Risk { CLEAR, MONITOR, ACTION, EMERGENCY };
struct CPA { double range, tcpa, dcpa; };
// All thresholds are copied from YAML by the ROS node.  Defaults preserve the
// library's standalone behaviour and make unit tests usable without ROS.
struct RiskThresholds {
  double monitor_dcpa{20}, action_dcpa{15}, emergency_dcpa{8};
  double monitor_range{80}, action_range{45}, emergency_range{20};
  double monitor_tcpa{90}, action_tcpa{35}, emergency_tcpa{12};
};
// [功能与联系] 用相对位置/速度计算当前距离、TCPA和DCPA；risk与Policy更新共用。
inline CPA cpa(double x, double y, double vx, double vy, const VesselState &t) {
  const double rx=t.x-x, ry=t.y-y, dx=t.vx-vx, dy=t.vy-vy;
  const double vv=dx*dx+dy*dy;
  const double time=vv>1e-9 ? -(rx*dx+ry*dy)/vv : 0;
  return {std::hypot(rx,ry),time,std::hypot(rx+dx*time,ry+dy*time)};
}
// [功能与联系] 按DCPA与(当前距离或TCPA)共同分层，负TCPA为CLEAR；不是三个圈中的任一个单独触发动作。
inline Risk risk(const CPA &c, const RiskThresholds &p=RiskThresholds{}) {
  if(c.tcpa<0) return Risk::CLEAR;
  if(c.dcpa<=p.emergency_dcpa && (c.range<=p.emergency_range || c.tcpa<=p.emergency_tcpa)) return Risk::EMERGENCY;
  if(c.dcpa<=p.action_dcpa && (c.range<=p.action_range || c.tcpa<=p.action_tcpa)) return Risk::ACTION;
  if(c.dcpa<=p.monitor_dcpa && (c.range<=p.monitor_range || c.tcpa<=p.monitor_tcpa)) return Risk::MONITOR;
  return Risk::CLEAR;
}
// [功能与联系] 将风险枚举转换为诊断文本，供摘要及可视化。
inline const char* riskName(Risk r) {
  switch(r) {case Risk::MONITOR:return "MONITOR";case Risk::ACTION:return "ACTION";
    case Risk::EMERGENCY:return "EMERGENCY";default:return "CLEAR";}
}
struct Policy {
  RiskThresholds thresholds{};
  double release_tcpa{-5}, release_range{30}, manoeuvre_grace{8};
  int release_observations{3};
  EncounterType type{EncounterType::NONE};
  Risk level{Risk::CLEAR};
  bool locked{false}, takeover{false}, effective{false};
  // 会遇记录保留滞回，但安全通过后不再继续命令避让/保向。
  bool recovering{false};
  double reference{0}, speed{0}, target_heading{0}, target_speed{0}, baseline_dcpa{0};
  double onset{0}, last_observation{-1e9};
  int ineffective_count{0}, release_count{0};
  CPA metric{};
  // [功能与联系] 判断当前策略是否要求保向保速；recovering安全恢复标志会立即退出此动作约束。
  bool standOn() const {
    return locked && !recovering && !takeover &&
      (type==EncounterType::CROSSING_PORT || type==EncounterType::BEING_OVERTAKEN);
  }
  // [功能与联系] 判断当前策略是否要求右转让路/接管；历史会遇锁存在安全恢复后不继续约束航向。
  bool right() const {
    return locked && !recovering && (takeover || type==EncounterType::HEAD_ON ||
      type==EncounterType::CROSSING_STARBOARD);
  }
  // [功能与联系] 根据风险层给出行动角里程碑；达到后不应每周期重复要求新增相同角度。
  double angle() const {
    return (level==Risk::EMERGENCY ? 20 : level==Risk::ACTION ? 12 : 5)*deg;
  }
  // [功能与联系] 联合通过几何、实际CPA已过及恢复方向安全性判定能否恢复追踪；不等待历史会遇记录计时解锁。
  bool safeToRecover(double x,double y,const VesselState &target,double safe,
                     bool recovery_risk) const {
    if(!locked || recovery_risk || metric.tcpa>=0 || metric.range<=safe) return false;
    const double dx=target.x-x,dy=target.y-y;
    const double forward=dx*std::cos(reference)+dy*std::sin(reference);
    switch(type) {
      case EncounterType::HEAD_ON:
      case EncounterType::OVERTAKING:return forward < -safe;
      case EncounterType::BEING_OVERTAKEN:return forward > safe;
      case EncounterType::CROSSING_STARBOARD:
      case EncounterType::CROSSING_PORT: {
        const double v=std::hypot(target.vx,target.vy);
        // 已在对方船尾安全域后方；仅仅右转令CPA变负还不够。
        return v>0.05 && (dx*target.vx+dy*target.vy)/v > safe;
      }
      default:return false;
    }
  }
  // Called once per fresh observation, not once per search node.
  // [功能与联系] 按最新观测更新运动/策略状态；Policy版本使用锁定参考反事实CPA与释放滞回，仿真版本则调用运动积分及发布。
  void update(double now, double x, double y, double heading, double velocity,
              const VesselState &target) {
    metric=cpa(x,y,velocity*std::cos(heading),velocity*std::sin(heading),target);
    auto level_now=risk(metric,thresholds);
    if(!locked && level_now!=Risk::CLEAR) {
      auto a=assessEncounter(x,y,heading,velocity,target,1e9,1e9,
                            20*deg,20*deg,112.5*deg,0.05);
      if(!a.active) return;
      type=a.type; locked=true; reference=heading; speed=velocity;
      onset=now;
      target_heading=std::atan2(target.vy,target.vx);
      target_speed=std::hypot(target.vx,target.vy);
      baseline_dcpa=metric.dcpa;
      takeover=false; ineffective_count=release_count=0; level=level_now;
    }
    if(!locked) return;
    const auto counterfactual=cpa(x,y,speed*std::cos(reference),speed*std::sin(reference),target);
    // Decisions use the locked course/speed counterfactual.  Otherwise a
    // fail-safe stop caused by an infeasible stand-on path would make CPA look
    // harmless and incorrectly cancel Rule 17 takeover.
    const auto decision_level=risk(counterfactual,thresholds);
    // Keep the strongest required action until passing and clear; own manoeuvring
    // must not make a head-on encounter turn into a port crossing.
    level=static_cast<Risk>(std::max(static_cast<int>(level),static_cast<int>(decision_level)));
    const bool manoeuvre=std::abs(normalizeAngle(std::atan2(target.vy,target.vx)-target_heading))>=3*deg ||
      std::abs(std::hypot(target.vx,target.vy)-target_speed)>=0.15;
    effective=counterfactual.dcpa>=thresholds.monitor_dcpa ||
      (manoeuvre && counterfactual.dcpa-baseline_dcpa>=1.5);
    if(now-last_observation>=0.8) {
      last_observation=now;
      const bool clear=metric.tcpa < release_tcpa && metric.range>release_range;
      release_count=clear ? release_count+1 : 0;
      if(release_count>=release_observations) {locked=false;type=EncounterType::NONE;level=Risk::CLEAR;return;}
      if(standOn()) {
        // Rule 17 takeover is for a give-way vessel that is not taking action.
        // Do not mistake the first seconds of an observable manoeuvre for
        // non-compliance merely because its DCPA benefit has not accumulated
        // to 1.5 m yet.  Emergency risk still overrides this grace period.
        if(decision_level>=Risk::ACTION && !effective) {
          if(!manoeuvre || now-onset>=manoeuvre_grace) ++ineffective_count;
        } else {
          ineffective_count=0;
        }
        if((decision_level==Risk::EMERGENCY && !effective) || ineffective_count>=3) {
          takeover=true;
        }
      }
    }
  }
};

// Per-candidate constraints. A bounded initial action interval lets the search
// complete a finite route; the live policy remains locked until safely passed.
// [功能与联系] 对预测候选施加有界初始COLREG航向约束；安全恢复后关闭动作约束，但动态硬净空仍检查。
inline bool headingAllowed(const Policy &p, double heading, double future,
                           double, double start_heading) {
  if(!p.locked || p.recovering || future>12) return true;
  const double deviation=normalizeAngle(heading-p.reference);
  if(p.standOn()) return deviation<=1*deg+1e-6 && deviation>=-5*deg-1e-6;
  if(!p.right()) return true;
  const double start=normalizeAngle(start_heading-p.reference);
  // The required substantial alteration is an action milestone, not a fresh
  // 12-second turn command on every re-plan.  Once the vessel has reached the
  // risk-level angle, dynamic separation (and the crossing-astern constraint)
  // decide when it is safe to return to the route.
  if(start<=-p.angle()+0.002) return true;
  const double wanted=std::max(-p.angle(), start-0.035*future);
  return deviation<=std::max(wanted,-p.angle())+0.002;
}
} // namespace hybrid_a_star_planner
