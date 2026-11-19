#include <cmath>
#include <gtest/gtest.h>
#include "usv_mpc_control/underactuated_model.hpp"

TEST(VesselModel, PlannerEquationsAndForwardCommands)
{
  using namespace usv_mpc_control;
  const VesselState s{1,2,0.4,1.1,0.2,-0.1};
  const VesselControl c{1.4,0.2};
  const auto d = derivative(s,c);
  EXPECT_NEAR(d.x, s.u*std::cos(s.yaw)-s.v*std::sin(s.yaw),1e-12);
  EXPECT_NEAR(d.y, s.u*std::sin(s.yaw)+s.v*std::cos(s.yaw),1e-12);
  EXPECT_NEAR(d.u, -0.433264281*s.u+0.400421432*c.v_cmd+0.032263988,1e-12);
  EXPECT_NEAR(d.v, -0.313324916*s.v-0.006741061*s.r-0.407958500*c.w_cmd-0.000993619,1e-12);
  EXPECT_NEAR(d.r, 0.166118168*s.v-0.544509453*s.r+0.837536662*c.w_cmd+0.000626529,1e-12);
  const auto seed = trackSetpoint(s,0,0,{},0.8,0.8,2.0,0.3);
  EXPECT_GE(seed.v_cmd,0.0);
  EXPECT_LE(seed.v_cmd,2.0);
  EXPECT_LE(std::abs(seed.w_cmd),0.3);
}
TEST(VesselModel, ZeroCommandHasPositiveSurgeEquilibrium)
{
  using namespace usv_mpc_control;
  VesselState s{};
  for (int i=0;i<2000;++i) s=usv_mpc_control::integrateRk4(s,VesselControl{}, VesselModelParameters{},0.05);
  EXPECT_NEAR(s.u,0.032263988/0.433264281,1e-8);
}
