#include "joint_target_guard.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace interactive_b2z1;
using namespace interactive_b2z1::sim_control;
int main() {
  Joint18 z{};
  const auto base=makeLegTargets(z);
  assert(base.saturated==0);
  // Policy FR hip is simulator index 3, policy FL hip is simulator index 0.
  assert(std::fabs(base.q_target_motor[3]-(-0.1f))<1e-6f);
  assert(std::fabs(base.q_target_motor[0]-(0.1f))<1e-6f);
  z[0]=1.f; z[3]=-1.f; z[6]=2.f; z[9]=-2.f;
  const auto x=makeLegTargets(z);
  assert(std::fabs(x.q_target_motor[3]-0.3f)<1e-5f);
  assert(std::fabs(x.q_target_motor[0]-(-0.3f))<1e-5f);
  assert(std::fabs(x.q_target_motor[9]-0.7f)<1e-5f);
  assert(std::fabs(x.q_target_motor[6]-(-0.7f))<1e-5f);
  z[0]=100.f;
  assert(makeLegTargets(z).saturated>=1);
  z[0]=std::numeric_limits<float>::quiet_NaN();
  try { (void)makeLegTargets(z); assert(false); }catch(const std::invalid_argument&){}
  auto prev=base.q_target_motor;
  Joint12 desired{}; desired.fill(0.5f);
  const auto next=slewMotorTargets(desired,prev,0.008f);
  for(std::size_t i=0;i<12;++i) assert(std::abs(next[i]-prev[i])<=0.00801f);
  checkSimFeedback(base.q_target_motor,Joint12{}, {1,0,0,0});
  try { checkSimFeedback(base.q_target_motor,Joint12{}, {0,1,0,0}); assert(false); }
  catch(const std::runtime_error&){}
  std::cout << "M1.5 TARGET GUARD: PASS\n";
}
