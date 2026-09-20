#define main planner_entry_unused
#include "../src/hybrid_a_star_node.cpp"
#undef main
#include <gtest/gtest.h>
using namespace hybrid_a_star_planner;

TEST(CurrentDistanceSafety, TwoMetreStopBoundaryIsIndependentOfCPA) {
  EXPECT_TRUE(insideStopDistance(-270,-270,-268.01,-270,2));
  EXPECT_FALSE(insideStopDistance(-270,-270,-268,-270,2));
  EXPECT_FALSE(insideStopDistance(-270,-270,-267.99,-270,2));
  EXPECT_TRUE(insideStopDistance(0,0,0,-1.9,2));
}

TEST(EventDrivenPlanning, ReplansOnlyWhenRemainingPathBecomesUnsafe) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=100;map->info.height=80;map->info.resolution=1;
  map->info.origin.position.x=-10;map->info.origin.position.y=-40;
  map->info.origin.orientation.w=1;map->data.assign(100*80,0);
  planner.setCostmap(map);planner.setParams(2,9,5,0.5,0.1,200000,88);
  planner.setDynamicParams(4,0.5,1,60,3,6,12,8,0.05,3);
  std::vector<geometry_msgs::msg::Pose> straight;
  for(int x=0;x<=50;++x) {
    geometry_msgs::msg::Pose pose;pose.position.x=x;pose.orientation.w=1;
    straight.push_back(pose);
  }
  EXPECT_FALSE(planner.remainingPathNeedsReplan(straight,0,0,0,2.5,5));

  // 能被感知但运动轨迹与本船路径不相交，不应仅因“附近有船”重规划。
  DynamicObstacle harmless;harmless.state={20,15,0,2,0.5};
  planner.setDynamicObstacles({harmless});
  EXPECT_FALSE(planner.remainingPathNeedsReplan(straight,0,0,0,2.5,5));

  // 正面对遇会进入预测碰撞域并违反右舷行动要求，必须触发一次新搜索。
  DynamicObstacle head_on;head_on.state={25,0,-2,0,0.5};
  head_on.policy.locked=true;head_on.policy.type=EncounterType::HEAD_ON;
  head_on.policy.level=Risk::ACTION;head_on.policy.reference=0;
  head_on.encounter.active=true;
  planner.setDynamicObstacles({head_on});
  EXPECT_TRUE(planner.remainingPathNeedsReplan(straight,0,0,0,2.5,5));

  // 同一次会遇成功规划后不再重复消费规则触发；若缓存路径本身没有进入
  // 硬碰撞域，关闭规则复查应保持执行，防止匀速目标导致每秒重搜。
  head_on.state.y=4.0;planner.setDynamicObstacles({head_on});
  EXPECT_TRUE(planner.remainingPathNeedsReplan(straight,0,0,0,2.0,5,true));
  EXPECT_FALSE(planner.remainingPathNeedsReplan(straight,0,0,0,2.0,5,false));
  head_on.state.y=0.0;planner.setDynamicObstacles({head_on});

  // 已规划到右舷且净空足够的路径继续交给DWA执行，不重复搜索。
  std::vector<geometry_msgs::msg::Pose> starboard;
  for(int x=0;x<=50;++x) {
    geometry_msgs::msg::Pose pose;pose.position.x=x;
    pose.position.y=-0.25*std::min(x,48);pose.orientation.w=1;
    starboard.push_back(pose);
  }
  EXPECT_FALSE(planner.remainingPathNeedsReplan(starboard,0,0,0,2.5,5));

  // 地图中新出现的硬障碍落在剩余路径上时，即使没有目标船也要重规划。
  planner.setDynamicObstacles({});
  map->data[40*100+20]=100;planner.setCostmap(map);
  EXPECT_TRUE(planner.remainingPathNeedsReplan(straight,0,0,0,2.5,5));
}

TEST(EventDrivenPlanning, RenewedCollisionCancelsRecoveryImmediately) {
  Policy policy;policy.locked=true;policy.recovering=true;
  policy.type=EncounterType::HEAD_ON;policy.level=Risk::ACTION;
  policy.reference=0;policy.speed=4;
  policy.update(1,0,0,0,4,{25,0,-2,0,0.5});
  EXPECT_FALSE(policy.recovering);
}
// 回归：预测减速时间与无等待的软连续性风险门控。
// 行为恢复必须独立于会遇记录的解除滞回。
TEST(RecoveryCompletion, DoesNotWaitForHistoricalReleaseTimer) {
  Policy p;p.locked=true;p.type=EncounterType::CROSSING_STARBOARD;
  p.level=Risk::EMERGENCY;p.reference=0;p.metric={10,-0.1,8};
  // 船尾净空满足7.5m，但尚未满足TCPA<-2s且连续两次的历史解锁条件。
  EXPECT_TRUE(p.safeToRecover(0,0,{0,10,0,2,2},7.5,false));
  EXPECT_FALSE(p.safeToRecover(0,0,{0,10,0,2,2},7.5,true));
  EXPECT_FALSE(p.safeToRecover(0,0,{10,-1,0,2,2},7.5,false));
  p.recovering=true;
  EXPECT_TRUE(headingAllowed(p,0.2,1,0,-0.1));
  EXPECT_FALSE(p.right());
  EXPECT_TRUE(p.locked);
}
TEST(RecoveryContinuity, ArrivalModelAccountsForGoalBraking) {
  HybridAStarPlanner planner;
  planner.setDynamicParams(4,1.5,4,60,1.2,9,8,12,0.05,9);
  EXPECT_DOUBLE_EQ(planner.estimateSegmentTime(-5,0,0,0,5),1.25);
  planner.setRecoveryParams(2,6,true,0.5,0.3,0.4);
  EXPECT_GT(planner.estimateSegmentTime(-5,0,0,0,5),4.0);
  EXPECT_TRUE(std::isfinite(planner.estimateSegmentTime(0,0,0,0,1)));
}
TEST(RecoveryContinuity, RiskGateClearsWithoutWaitingForPolicyUnlock) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=220;map->info.height=180;map->info.resolution=1;
  map->info.origin.position.x=-50;map->info.origin.position.y=-90;
  map->data.assign(220*180,0);planner.setCostmap(map);
  planner.setParams(2,9,5,0.5,0.1,250000,88);
  planner.setDynamicParams(4,1.5,4,60,1.2,9,8,12,0.05,9);
  planner.setRecoveryParams(2,6,true,0.5,0.3,0.4);
  DynamicObstacle o;o.state={0,-20,0,2,2};
  planner.setDynamicObstacles({o});
  auto avoiding=planner.search(-40,0,0,55,0,0);
  EXPECT_TRUE(planner.recoveryContinuityActive());ASSERT_FALSE(avoiding.empty());
  avoiding=planner.postProcessPath(avoiding,1,120,0.26,0.015,6,6,0.12);
  double elapsed=0;
  for(std::size_t i=1;i<avoiding.size();++i) {
    const auto &a=avoiding[i-1].position;const auto &b=avoiding[i].position;
    const double length=std::hypot(b.x-a.x,b.y-a.y);
    const int count=std::max(1,static_cast<int>(std::ceil(length/0.25)));
    double px=a.x,py=a.y;
    for(int j=1;j<=count;++j) {
      const double f=static_cast<double>(j)/count;
      const double x=a.x+(b.x-a.x)*f,y=a.y+(b.y-a.y)*f;
      elapsed+=planner.estimateSegmentTime(px,py,x,y,length/count);px=x;py=y;
      EXPECT_GE(std::hypot(x-o.state.x-o.state.vx*elapsed,
        y-o.state.y-o.state.vy*elapsed),7.45);
    }
  }
  planner.setReferencePath(avoiding);
  // 实际通过后，即使旧策略尚有锁定余量，额外连续性也立即关闭。
  o.state={0,40,0,2,2};o.policy.locked=true;
  o.policy.type=EncounterType::CROSSING_STARBOARD;o.policy.reference=0;
  o.policy.level=Risk::ACTION;
  planner.setDynamicObstacles({o});
  EXPECT_FALSE(planner.search(20,-5,-0.3,55,0,0).empty());
  EXPECT_FALSE(planner.recoveryContinuityActive());
  EXPECT_TRUE(headingAllowed(o.policy,0,1,0,-25*deg));
}

// 连续规划的近场参考应抑制左右换边，但不得覆盖动态碰撞硬约束。
TEST(TemporalStability, NearFieldPrefersPreviousSafeSide) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=130;map->info.height=80;map->info.resolution=1;
  map->info.origin.position.x=-40;map->info.origin.position.y=-40;
  map->info.origin.orientation.w=1;map->data.assign(130*80,0);
  planner.setCostmap(map);planner.setParams(2,9,5,0.5,0.10,250000,88);
  planner.setDynamicParams(4,1.5,4,60,1.2,9,8,12,0.05,9);
  planner.setStabilityParams(0.5,12,1,1);
  planner.setNearFieldStabilityParams(12,45,3);
  std::vector<geometry_msgs::msg::Pose> previous;
  for(int x=-30;x<=50;++x) {
    geometry_msgs::msg::Pose pose;pose.position.x=x;
    pose.position.y=3.0*std::sin(M_PI*(x+30)/80.0);pose.orientation.w=1;
    previous.push_back(pose);
  }
  planner.setReferencePath(previous);
  const auto path=planner.search(-30,0,0,50,0,0);
  ASSERT_FALSE(path.empty());
  double near_sum=0;int near_count=0;
  for(const auto &pose:path) if(pose.position.x>-20 && pose.position.x<10) {
    near_sum+=pose.position.y;++near_count;
  }
  ASSERT_GT(near_count,5);
  EXPECT_GT(near_sum/near_count,0.4);
}

TEST(TemporalStability, DynamicSafetyOverridesPreviousPath) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=130;map->info.height=100;map->info.resolution=1;
  map->info.origin.position.x=-40;map->info.origin.position.y=-50;
  map->info.origin.orientation.w=1;map->data.assign(130*100,0);
  planner.setCostmap(map);planner.setParams(2,9,5,0.5,0.10,250000,88);
  planner.setDynamicParams(4,0.5,1,60,3,6,8,12,0.05,3);
  planner.setNearFieldStabilityParams(12,45,3);
  std::vector<geometry_msgs::msg::Pose> previous;
  for(int x=-30;x<=50;++x) {geometry_msgs::msg::Pose p;p.position.x=x;p.orientation.w=1;previous.push_back(p);}
  planner.setReferencePath(previous);
  DynamicObstacle obstacle;obstacle.state={0,0,0,0,0.5};
  obstacle.policy.locked=true;obstacle.policy.type=EncounterType::HEAD_ON;
  obstacle.encounter.active=true;
  planner.setDynamicObstacles({obstacle});
  const auto path=planner.search(-30,0,0,50,0,0);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(planner.referenceSoftConflictActive());
  for(const auto &pose:path)
    EXPECT_GE(std::hypot(pose.position.x,pose.position.y),1.99);
}

TEST(FourMetrePerSecondPlanning, SmallerVesselsStillAvoidAllEncounterTypes) {
  for(const auto type:{EncounterType::HEAD_ON,EncounterType::CROSSING_STARBOARD,
      EncounterType::CROSSING_PORT,EncounterType::OVERTAKING}) {
    SCOPED_TRACE(toString(type));
    HybridAStarPlanner planner;
    auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.width=220;map->info.height=180;map->info.resolution=1;
    map->info.origin.position.x=-50;map->info.origin.position.y=-90;
    map->info.origin.orientation.w=1;map->data.assign(220*180,0);
    planner.setCostmap(map);planner.setParams(2,9,5,0.5,0.10,250000,88);
    planner.setDynamicParams(4,0.5,1,60,3,6,8,8,0.05,3);
    planner.setStabilityParams(0.5,12,1,1);
    DynamicObstacle obstacle;
    obstacle.policy.type=type;obstacle.policy.locked=true;
    obstacle.policy.level=Risk::ACTION;obstacle.policy.speed=4;
    obstacle.policy.takeover=type==EncounterType::CROSSING_PORT;
    if(type==EncounterType::HEAD_ON) obstacle.state={60,0,-2,0,0.5};
    if(type==EncounterType::CROSSING_STARBOARD) obstacle.state={0,-20,0,2,0.5};
    if(type==EncounterType::CROSSING_PORT) obstacle.state={0,20,0,-2,0.5};
    if(type==EncounterType::OVERTAKING) obstacle.state={-10,0,2,0,0.5};
    planner.setDynamicObstacles({obstacle});
    auto path=planner.search(-40,0,0,55,0,0);
    ASSERT_GT(path.size(),20U);
    path=planner.postProcessPath(path,1,120,0.26,0.015,6,6,0.12);
    double distance=0;
    for(std::size_t i=1;i<path.size();++i) {
      const double dx=path[i].position.x-path[i-1].position.x;
      const double dy=path[i].position.y-path[i-1].position.y;
      const double length=std::hypot(dx,dy);
      const int samples=std::max(1,static_cast<int>(std::ceil(length/0.25)));
      for(int sample=1;sample<=samples;++sample) {
        const double fraction=static_cast<double>(sample)/samples;
        const double time=(distance+fraction*length)/4;
        const double x=path[i-1].position.x+fraction*dx;
        const double y=path[i-1].position.y+fraction*dy;
        EXPECT_GE(std::hypot(x-obstacle.state.x-obstacle.state.vx*time,
          y-obstacle.state.y-obstacle.state.vy*time),
          type==EncounterType::OVERTAKING ? 2.99 : 1.99);
      }
      distance+=length;
    }
  }
}

TEST(V4Policy, RiskAndRule17Sequence) {
  EXPECT_EQ(risk({80,90,20}),Risk::MONITOR);
  EXPECT_EQ(risk({45,35,15}),Risk::ACTION);
  EXPECT_EQ(risk({20,12,8}),Risk::EMERGENCY);
  EXPECT_EQ(risk({10,-1,0}),Risk::CLEAR);
  Policy p;
  VesselState target{30,30,0,-0.8,4};
  p.update(0,0,0,0,0.8,target);
  EXPECT_EQ(p.type,EncounterType::CROSSING_PORT);
  EXPECT_FALSE(p.takeover);
  p.update(1,0.8,0,0,0.8,{30,29.2,0,-0.8,4});
  EXPECT_FALSE(p.takeover);
  p.update(2,1.6,0,0,0.8,{30,28.4,0,-0.8,4});
  EXPECT_TRUE(p.takeover);
  EXPECT_FALSE(headingAllowed(p,0,2,2,0));
  EXPECT_TRUE(headingAllowed(p,-0.1,2,2,0));
}
TEST(V4Policy, ConfigurableRiskThresholds) {
  RiskThresholds t;t.monitor_dcpa=16;t.action_dcpa=13;t.emergency_dcpa=9;
  t.monitor_range=60;t.action_range=35;t.emergency_range=18;
  t.monitor_tcpa=60;t.action_tcpa=30;t.emergency_tcpa=10;
  EXPECT_EQ(risk({55,50,15.5},t),Risk::MONITOR);
  EXPECT_EQ(risk({34,29,12.5},t),Risk::ACTION);
  EXPECT_EQ(risk({17,9,8.5},t),Risk::EMERGENCY);
  EXPECT_EQ(risk({70,70,17},t),Risk::CLEAR);
}
TEST(V4Policy, EffectiveTargetAndLockRelease) {
  Policy p;
  p.update(0,0,0,0,0.8,{30,30,0,-0.8,4});
  // Target turns to its starboard (west) and clears the own-ship track.
  p.update(1,0.8,0,0,0.8,{30,29.2,-0.8,0,4});
  EXPECT_TRUE(p.effective); EXPECT_FALSE(p.takeover);
  EXPECT_EQ(p.type,EncounterType::CROSSING_PORT);
  // Three distinct observations after CPA, separated by a full release margin.
  for(int i=2;i<=4;++i) p.update(i,80+i,0,0,0.8,{-40,0,-0.8,0,4});
  EXPECT_FALSE(p.locked);
}
TEST(V4Policy, ObservableTargetManoeuvreDoesNotTriggerPrematureRule17) {
  Policy p;
  p.update(0,0,0,0,0.8,{30,30,0,-0.8,4});
  // A developing starboard alteration is observable, but its DCPA benefit is
  // intentionally still below the effectiveness threshold.
  p.update(1,0.8,0,0,0.8,{29.98,29.2,-0.045,-0.799,4});
  p.update(2,1.6,0,0,0.8,{29.91,28.4,-0.056,-0.798,4});
  EXPECT_FALSE(p.takeover);
  EXPECT_TRUE(p.standOn());
}
TEST(V4Policy, FailSafeStopCannotMaskRule17Risk) {
  Policy p;
  p.update(0,0,0,0,0.8,{30,30,0,-0.8,4});
  // Simulate the local controller stopping while the other vessel continues.
  p.update(1,0,0,0,0.0,{30,29.2,0,-0.8,4});
  p.update(2,0,0,0,0.0,{30,28.4,0,-0.8,4});
  EXPECT_TRUE(p.takeover);
  EXPECT_TRUE(p.right());
}
TEST(V4Policy, AnglesAndActionLevels) {
  for(double rotation:{0.0,1.0,-2.9}) {
    for(int angle=-19;angle<=19;++angle) {
      double bearing=rotation+angle*deg;
      auto a=assessEncounter(0,0,rotation,0.8,
        {50*std::cos(bearing),50*std::sin(bearing),-0.8*std::cos(rotation),
        -0.8*std::sin(rotation),4},1e9,1e9,20*deg,20*deg,112.5*deg,0.05);
      EXPECT_EQ(a.type,EncounterType::HEAD_ON);
    }
  }
  for(Risk r:{Risk::MONITOR,Risk::ACTION,Risk::EMERGENCY}) {
    Policy p;p.locked=true;p.type=EncounterType::HEAD_ON;p.level=r;
    EXPECT_FALSE(headingAllowed(p,0,4,0,0));
    EXPECT_FALSE(headingAllowed(p,0.1,4,0,0));
    EXPECT_TRUE(headingAllowed(p,-0.15,4,0,0));
  }
  Policy completed;completed.locked=true;completed.takeover=true;
  completed.type=EncounterType::CROSSING_PORT;completed.level=Risk::EMERGENCY;
  EXPECT_TRUE(headingAllowed(completed,0.0,1,0,-21*deg));
}

TEST(PathPostProcessing, ReducesGlobalZigZagWithoutMovingEndpoints) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=80;map->info.height=60;map->info.resolution=1.0;
  map->info.origin.position.x=-10;map->info.origin.position.y=-30;
  map->info.origin.orientation.w=1.0;map->data.assign(80*60,0);
  planner.setCostmap(map);
  planner.setParams(2,9,5,0.5,0.10,250000,88);
  planner.setDynamicParams(1.2,2,3,120,1.35,12,8,8,0.05);
  std::vector<geometry_msgs::msg::Pose> raw;
  for(int x=0;x<=30;++x) {
    geometry_msgs::msg::Pose pose;pose.position.x=x;
    pose.position.y=(x==0 || x==30)?0.0:(x%2==0?1.0:-1.0);
    pose.orientation.w=1.0;raw.push_back(pose);
  }
  auto length=[](const auto &path) {
    double result=0;
    for(std::size_t i=1;i<path.size();++i) result+=std::hypot(
      path[i].position.x-path[i-1].position.x,
      path[i].position.y-path[i-1].position.y);
    return result;
  };
  auto heading_variation=[](const auto &path) {
    double result=0;
    for(std::size_t i=2;i<path.size();++i) {
      const double previous=std::atan2(
        path[i-1].position.y-path[i-2].position.y,
        path[i-1].position.x-path[i-2].position.x);
      const double current=std::atan2(
        path[i].position.y-path[i-1].position.y,
        path[i].position.x-path[i-1].position.x);
      result+=std::abs(normalizeAngle(current-previous));
    }
    return result;
  };
  const auto smooth=planner.postProcessPath(raw,1.0,120,0.26,0.015,6.0,6,0.12);
  ASSERT_GE(smooth.size(),3U);
  EXPECT_NEAR(smooth.front().position.x,raw.front().position.x,1e-9);
  EXPECT_NEAR(smooth.front().position.y,raw.front().position.y,1e-9);
  EXPECT_NEAR(smooth.back().position.x,raw.back().position.x,1e-9);
  EXPECT_NEAR(smooth.back().position.y,raw.back().position.y,1e-9);
  EXPECT_LT(length(smooth),0.85*length(raw));
  // 即使输入为每米左右交替的极端锯齿，整体累计转向波动也必须至少降低60%。
  EXPECT_LT(heading_variation(smooth),0.40*heading_variation(raw));
}

TEST(PathPostProcessing, NeverCutsAcrossDynamicShipSafetyArea) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=60;map->info.height=50;map->info.resolution=1.0;
  map->info.origin.position.x=-10;map->info.origin.position.y=-20;
  map->info.origin.orientation.w=1.0;map->data.assign(60*50,0);
  planner.setCostmap(map);
  planner.setParams(2,9,5,0.5,0.10,250000,88);
  planner.setDynamicParams(1.2,2,3,120,1.35,12,8,8,0.05);

  DynamicObstacle obstacle;
  obstacle.state={15,0,0,0,2};
  obstacle.policy.type=EncounterType::NONE;
  planner.setDynamicObstacles({obstacle});

  std::vector<geometry_msgs::msg::Pose> raw;
  for(const auto &[x,y]:std::vector<std::pair<double,double>>{
      {0,0},{5,8},{10,9},{15,9},{20,9},{25,8},{30,0}}) {
    geometry_msgs::msg::Pose pose;pose.position.x=x;pose.position.y=y;
    pose.orientation.w=1.0;raw.push_back(pose);
  }
  const auto smooth=planner.postProcessPath(raw,1.0,120,0.26,0.015,6.0,6,0.12);
  ASSERT_GE(smooth.size(),3U);
  const double safe_radius=2.0+2.0+3.0;
  for(std::size_t i=1;i<smooth.size();++i) {
    const double dx=smooth[i].position.x-smooth[i-1].position.x;
    const double dy=smooth[i].position.y-smooth[i-1].position.y;
    const int samples=std::max(1,static_cast<int>(std::ceil(std::hypot(dx,dy)/0.25)));
    for(int sample=0;sample<=samples;++sample) {
      const double ratio=static_cast<double>(sample)/samples;
      const double x=smooth[i-1].position.x+ratio*dx;
      const double y=smooth[i-1].position.y+ratio*dy;
      EXPECT_GE(std::hypot(x-obstacle.state.x,y-obstacle.state.y),safe_radius-1.0e-6);
    }
  }
}

TEST(PathPostProcessing, SuppressesLongWavelengthSerpentine) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=140;map->info.height=60;map->info.resolution=1.0;
  map->info.origin.position.x=-10;map->info.origin.position.y=-30;
  map->info.origin.orientation.w=1.0;map->data.assign(140*60,0);
  planner.setCostmap(map);
  planner.setParams(2,9,5,0.5,0.10,250000,88);
  planner.setDynamicParams(1.2,2,3,120,1.35,12,8,8,0.05);
  std::vector<geometry_msgs::msg::Pose> raw;
  for(int x=0;x<=100;++x) {
    geometry_msgs::msg::Pose pose;pose.position.x=x;
    pose.position.y=5.0*std::sin(2.0*M_PI*x/50.0);
    pose.orientation.w=1.0;raw.push_back(pose);
  }
  const auto smooth=planner.postProcessPath(raw,1.0,120,0.26,0.015,6.0,6,0.12);
  ASSERT_GE(smooth.size(),3U);
  double raw_peak=0.0,smooth_peak=0.0;
  for(const auto &pose:raw) {
    if(pose.position.x>=10.0 && pose.position.x<=90.0)
      raw_peak=std::max(raw_peak,std::abs(pose.position.y));
  }
  for(const auto &pose:smooth) {
    if(pose.position.x>=10.0 && pose.position.x<=90.0)
      smooth_peak=std::max(smooth_peak,std::abs(pose.position.y));
  }
  EXPECT_LT(smooth_peak,0.65*raw_peak);
  EXPECT_NEAR(smooth.front().position.y,raw.front().position.y,1e-9);
  EXPECT_NEAR(smooth.back().position.y,raw.back().position.y,1e-9);
}

class PlannerV4: public ::testing::Test {
protected:
  HybridAStarPlanner planner;
  // [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
  void SetUp() override {
    auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.width=160;map->info.height=140;map->info.resolution=1;
    map->info.origin.position.x=-50;map->info.origin.position.y=-70;
    map->info.origin.orientation.w=1;map->data.assign(160*140,0);
    planner.setCostmap(map);planner.setParams(2,9,5,0.5,0.10,250000,40);
    planner.setDynamicParams(0.8,2,2,120,2,12,8,8,0.05);
    planner.setStabilityParams(0.0,12.0,1.0,0.6);
  }
  // [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
  void check(EncounterType type, VesselState t, bool takeover=false) {
    DynamicObstacle o;o.state=t;o.policy.type=type;o.policy.locked=true;
    o.policy.level=Risk::ACTION;o.policy.takeover=takeover;
    o.encounter.type=type;o.encounter.active=true;o.encounter.tcpa=30;o.encounter.urgency=0.8;
    o.encounter.preference=o.policy.right()?TurnPreference::RIGHT:TurnPreference::NONE;
    planner.setDynamicObstacles({o});
    const auto path=planner.search(-30,0,0,55,0,0);
    ASSERT_GT(path.size(),20u);
    double time=0, minimum=1e9;
    for(std::size_t i=1;i<path.size();++i) {
      time+=std::hypot(path[i].position.x-path[i-1].position.x,
                      path[i].position.y-path[i-1].position.y)/0.8;
      minimum=std::min(minimum,std::hypot(path[i].position.x-t.x-t.vx*time,
                                        path[i].position.y-t.y-t.vy*time));
      EXPECT_TRUE(headingAllowed(o.policy,tf2::getYaw(path[i].orientation),time,0,0));
      if(type==EncounterType::HEAD_ON) {
        const double target_x=t.x+t.vx*time;
        const double forward=target_x-path[i].position.x;
        if(forward>=-8.0) {
          EXPECT_LE(normalizeAngle(tf2::getYaw(path[i].orientation)),2.01*deg);
          if(forward<=0.0) {
            const double target_y=t.y+t.vy*time;
            EXPECT_LE(path[i].position.y-target_y,0.251);
          }
        }
      }
    }
    EXPECT_GE(minimum,type==EncounterType::OVERTAKING?14.99:7.99);
    EXPECT_LT(std::hypot(path.back().position.x-55,path.back().position.y),0.6);
    EXPECT_LT(std::abs(tf2::getYaw(path.back().orientation)),0.15);
  }
};
TEST_F(PlannerV4, HeadOn) {check(EncounterType::HEAD_ON,{25,0,-0.6,0,4});}
TEST_F(PlannerV4, HeadOnInitiallyOffset) {check(EncounterType::HEAD_ON,{25,-2,-0.6,0,4});}
TEST_F(PlannerV4, StarboardCrossing) {check(EncounterType::CROSSING_STARBOARD,{0,-30,0,0.8,4});}
TEST_F(PlannerV4, PortRule17Takeover) {check(EncounterType::CROSSING_PORT,{0,30,0,-0.8,4},true);}
TEST_F(PlannerV4, PortCooperative) {check(EncounterType::CROSSING_PORT,{0,30,-0.8,0,4});}
TEST_F(PlannerV4, Overtaking) {check(EncounterType::OVERTAKING,{-5,0,0.3,0,4});}
TEST_F(PlannerV4, OpenWater) {
  auto path=planner.search(-30,0,0,50,14.1,10*deg);
  ASSERT_GT(path.size(),20u);
  EXPECT_LT(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)-10*deg)),0.15);
}

TEST(ChannelFusion, ShoreInflationIsIgnoredOnlyInsideLane) {
  HybridAStarPlanner planner;
  auto local=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  local->info.width=20;local->info.height=20;local->info.resolution=1;
  local->info.origin.orientation.w=1;local->data.assign(400,90);
  auto lane=std::make_shared<nav_msgs::msg::OccupancyGrid>(*local);
  lane->data.assign(400,45);
  lane->data[10*20+10]=0;
  planner.setCostmap(local);planner.setLaneCostmap(lane);
  planner.setChannelParams(true,88,85,10,65,0.65,1.1,0.8,2,5,18);
  EXPECT_FALSE(planner.isLethal(10.5,10.5));
  EXPECT_TRUE(planner.isLethal(11.5,10.5));
}

TEST(ChannelFusion, ChannelObstacleInflationIsNeverCleared) {
  HybridAStarPlanner planner;
  auto local=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  local->info.width=20;local->info.height=20;local->info.resolution=1;
  local->info.origin.orientation.w=1;local->data.assign(400,90);
  auto lane=std::make_shared<nav_msgs::msg::OccupancyGrid>(*local);
  lane->data.assign(400,0);
  auto obstacle=std::make_shared<nav_msgs::msg::OccupancyGrid>(*local);
  obstacle->data.assign(400,0);obstacle->data[10*20+10]=85;
  planner.setCostmap(local);planner.setLaneCostmap(lane);
  planner.setChannelObstacleMap(obstacle);
  planner.setChannelParams(true,88,85,10,65,0.65,1.1,0.8,2,5,18);
  EXPECT_TRUE(planner.isLethal(10.5,10.5));
  EXPECT_FALSE(planner.isLethal(9.5,10.5));
}
