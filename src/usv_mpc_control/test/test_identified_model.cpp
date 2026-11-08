#include <gtest/gtest.h>
#include "usv_mpc_control/tracking_model.hpp"

namespace m=usv_mpc_control;
TEST(IdentifiedModel, SurgeMatchesIndependentAnalyticResponse) {
  m::TrackingModelParameters p;p.identified=true;
  m::VesselState s{0,0,0,.2,0,0};
  s=m::predict(s,{1.1,.15},p,7.3);
  const double equilibrium=(.400421432*1.1+.032263988)/.433264281;
  EXPECT_NEAR(s.u,equilibrium+(.2-equilibrium)*std::exp(-.433264281*7.3),1e-10);
  EXPECT_GT(std::abs(s.u-1.1),.01);  // Command does not overwrite actual u.
  EXPECT_GT(std::abs(s.r-.15),.001);
}
TEST(IdentifiedModel, BiasAndSwayYawCouplingRemainPresent) {
  const auto d=m::identified::derivative({0,0,0,0,.2,-.1},{0,.3});
  EXPECT_DOUBLE_EQ(d[3],.032263988);
  EXPECT_NEAR(d[4],-.313324916*.2-.006741061*(-.1)-.407958500*.3-.000993619,1e-14);
  EXPECT_NEAR(d[5],.166118168*.2-.544509453*(-.1)+.837536662*.3+.000626529,1e-14);
  const auto s=m::identified::integrate({0,0,0,0,0,0},{0,0},100);
  EXPECT_GT(s[3],.07);  // Zero command is not a zero-velocity equilibrium.
}
TEST(IdentifiedModel, FeedbackCancelsSurgeAndYawBiasWithoutVelocityClipping) {
  m::TrackingModelParameters p;p.identified=true;
  const m::VesselState s{0,0,0,.01,.03,-.02};
  const auto c=m::feedbackControl(s,0,0,p,.8,.8,1.5,.7,-.4);
  const auto d=m::identified::derivative(m::identifiedState(s),{c.surge,c.yaw});
  EXPECT_NEAR(d[3],-.01/.8,1e-14);EXPECT_NEAR(d[5],.02/.8,1e-14);
  const auto reverse=m::predict({}, {-.4,0},p,1);
  EXPECT_LT(reverse.u,0);  // No force-mode clamp fabricates the response.
}
TEST(IdentifiedModel, Rk4ConvergesAndKeepsContinuousHeading) {
  const m::identified::State s{1,2,3.13,.8,.1,.3};
  const auto a=m::identified::integrate(s,{1,.4},3,.02);
  const auto b=m::identified::integrate(s,{1,.4},3,.002);
  for(size_t i=0;i<6;++i)EXPECT_NEAR(a[i],b[i],2e-8);
  EXPECT_GT(a[2],3.141592653589793);
}
