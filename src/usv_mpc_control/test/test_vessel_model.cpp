#include <array>
#include <gtest/gtest.h>
#include "usv_mpc_control/underactuated_model.hpp"

TEST(VesselModel, SharedIntegratorPreservesPreCleanupTrajectories)
{
  // Golden states were generated from the old controller/simulator model,
  // before removing its duplicate equations. Includes turning and braking.
  const std::array<usv_mpc_control::VesselControl, 3> controls{{{4,.2},{4,-.2},{-2,0}}};
  const std::array<std::array<double,6>,3> expected{{
    {{20.176893640009752,10.95787866956047,.71792488973396273,3.985259887216813,-.24638540293738481,.11634224211388147}},
    {{21.830692315263061,4.0191796603827834,-.27328849341475081,3.9889288183186382,.27327795191112297,-.11626932699259877}},
    {{7.5452367506679643,4.4710462228566286,.22231819815960591,-.62423057571279561,.013446274486869121,3.6457560641345055e-05}}
  }};
  for(std::size_t k=0;k<controls.size();++k) {
    usv_mpc_control::VesselState state{2,3,.2,4,.15,.03};
    for(int i=0;i<100;++i) state=usv_mpc_control::integrateRk4(state,controls[k],{},.05);
    const std::array<double,6> actual{{state.x,state.y,state.yaw,state.u,state.v,state.r}};
    for(std::size_t i=0;i<actual.size();++i) EXPECT_NEAR(actual[i],expected[k][i],1e-12);
  }
}
