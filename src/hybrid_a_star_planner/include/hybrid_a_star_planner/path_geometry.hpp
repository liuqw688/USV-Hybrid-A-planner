#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "hybrid_a_star_planner/colregs.hpp"

namespace hybrid_a_star_planner::planning
{
// Shared spatial sampling for before/after comparisons. Dense RK4 output must
// not appear smoother merely because it has more points than a geometric path.
inline std::vector<geometry_msgs::msg::Pose> resampleGroundPath(
  const std::vector<geometry_msgs::msg::Pose> & path, double spacing)
{
  if (path.size() < 2) {return path;}
  spacing = std::max(.1, spacing);
  std::vector<double> arc(path.size(), 0.0);
  for (std::size_t i=1; i<path.size(); ++i) {
    const double ds=std::hypot(path[i].position.x-path[i-1].position.x,
      path[i].position.y-path[i-1].position.y);
    if (!std::isfinite(ds)) {return {};}
    arc[i]=arc[i-1]+ds;
  }
  if (arc.back()<1e-6) {return {path.front(),path.back()};}
  std::vector<geometry_msgs::msg::Pose> result;
  std::size_t segment=1;
  for (double s=0; s<arc.back(); s+=spacing) {
    while(segment+1<arc.size() && arc[segment]<s) {++segment;}
    const double f=std::clamp((s-arc[segment-1])/std::max(1e-9,arc[segment]-arc[segment-1]),0.0,1.0);
    auto pose=path[segment-1];
    pose.position.x+=f*(path[segment].position.x-pose.position.x);
    pose.position.y+=f*(path[segment].position.y-pose.position.y);
    const double yaw=tf2::getYaw(pose.orientation)+f*normalizeAngle(
      tf2::getYaw(path[segment].orientation)-tf2::getYaw(pose.orientation));
    tf2::Quaternion q;q.setRPY(0,0,yaw);pose.orientation=tf2::toMsg(q);
    result.push_back(pose);
  }
  result.front()=path.front();result.push_back(path.back());
  return result;
}

struct GroundPathQuality
{
  double length{0.0}, peak_curvature{0.0}, rms_curvature{0.0};
  int reversals{0}, dominant_turn{0};
};

// Signed three-point circle curvature; no heading/body-yaw substitution.
// A lobe must turn >=3 degrees over >=0.5m to count as an S reversal.
inline GroundPathQuality groundPathQuality(const std::vector<geometry_msgs::msg::Pose> & path)
{
  const auto points=resampleGroundPath(path,.25);
  GroundPathQuality result;
  if (points.empty() && !path.empty()) {
    result.peak_curvature=result.rms_curvature=std::numeric_limits<double>::infinity();
    return result;
  }
  double energy=0,positive=0,negative=0,lobe_angle=0,lobe_length=0;
  int sign=0,last_sign=0;
  auto finish=[&]() {
    if(std::abs(lobe_angle)>=3*M_PI/180 && lobe_length>=.5) {
      if(last_sign && last_sign!=sign) {++result.reversals;}
      last_sign=sign;
    }
    lobe_angle=0;lobe_length=0;
  };
  for(std::size_t i=1; i<points.size(); ++i) {
    result.length+=std::hypot(points[i].position.x-points[i-1].position.x,
      points[i].position.y-points[i-1].position.y);
    if(i+1>=points.size()) {continue;}
    const double ax=points[i].position.x-points[i-1].position.x;
    const double ay=points[i].position.y-points[i-1].position.y;
    const double bx=points[i+1].position.x-points[i].position.x;
    const double by=points[i+1].position.y-points[i].position.y;
    const double ab=std::hypot(ax,ay),bc=std::hypot(bx,by),ac=std::hypot(ax+bx,ay+by);
    if(ab*bc*ac<1e-12) {continue;}
    const double k=2*(ax*by-ay*bx)/(ab*bc*ac),ds=.5*(ab+bc);
    result.peak_curvature=std::max(result.peak_curvature,std::abs(k));
    energy+=k*k*ds;
    positive+=std::max(0.0,k*ds);negative+=std::max(0.0,-k*ds);
    const int next=k>.002 ? 1:(k<-.002 ? -1:0);
    if(next && next!=sign) {finish();sign=next;}
    if(next) {lobe_angle+=k*ds;lobe_length+=ds;}
  }
  finish();
  result.rms_curvature=std::sqrt(energy/std::max(1e-6,result.length));
  result.dominant_turn=positive>negative ? 1:(negative>positive ? -1:0);
  return result;
}

inline bool groundPathsWithinDeviation(const std::vector<geometry_msgs::msg::Pose> & a,
  const std::vector<geometry_msgs::msg::Pose> & b,double limit)
{
  if(limit<=0) {return true;}
  const auto first=resampleGroundPath(a,.5),second=resampleGroundPath(b,.5);
  if(first.empty() || second.empty()) {return false;}
  const double squared=limit*limit;
  auto near=[&](const auto &source,const auto &target) {
    for(const auto &p:source) {
      double best=std::numeric_limits<double>::infinity();
      for(const auto &q:target) {
        const double dx=p.position.x-q.position.x,dy=p.position.y-q.position.y;
        best=std::min(best,dx*dx+dy*dy);
      }
      if(best>squared) {return false;}
    }
    return true;
  };
  return near(first,second) && near(second,first);
}

// Preserve the original main turn when a whole wider arc cannot fit the map;
// only reshape the oscillating tail after the largest turning lobe.
inline std::size_t dominantTurnEndIndex(const std::vector<geometry_msgs::msg::Pose> & raw)
{
  const auto sampled=resampleGroundPath(raw,.25);
  int sign=0;double angle=0,best_angle=0,previous_course=0;std::size_t best_end=0;
  auto finish=[&](std::size_t end) {
    if(std::abs(angle)>best_angle) {best_angle=std::abs(angle);best_end=end;}
    angle=0;
  };
  for(std::size_t i=1;i<sampled.size();++i) {
    const double course=std::atan2(sampled[i].position.y-sampled[i-1].position.y,
      sampled[i].position.x-sampled[i-1].position.x);
    if(i>1) {
      const double change=normalizeAngle(course-previous_course);
      const int next=change>.0005 ? 1:(change<-.0005 ? -1:0);
      if(next && next!=sign) {finish(i-1);sign=next;}
      if(next) {angle+=change;}
    }
    previous_course=course;
  }
  finish(sampled.size()>1 ? sampled.size()-1:0);
  if(best_angle<1.2 || best_end+3>=sampled.size()) {return 0;}
  const double target_arc=.25*best_end;
  double arc=0;
  for(std::size_t i=1;i<raw.size();++i) {
    arc+=std::hypot(raw[i].position.x-raw[i-1].position.x,raw[i].position.y-raw[i-1].position.y);
    if(arc>=target_arc) {return i;}
  }
  return 0;
}
}  // namespace hybrid_a_star_planner::planning
