// Local fixture mirrors structural observation fields only; build on user's PC
// links the exact original interactive_observation.cpp, not this fixture.
#include "interactive_observation.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace interactive_b2z1 {
ObservationConfig ObservationConfig::baseline(){ObservationConfig c{};c.default_joint_pos={
-.1f,.8f,-1.5f,.1f,.8f,-1.5f,-.1f,.8f,-1.5f,.1f,.8f,-1.5f,
0.f,1.48f,-.63f,-.84f,0.f,1.57f};return c;}
ObservationBuilder::ObservationBuilder(ObservationConfig c):cfg_(c){}
Observation799 ObservationBuilder::build(const RobotSample& s,const Command3& cmd){
 Proprio71 p{};
 p[0]=std::atan2(2*(s.imu_quat_wxyz[0]*s.imu_quat_wxyz[1]+s.imu_quat_wxyz[2]*s.imu_quat_wxyz[3]),1.-2*(s.imu_quat_wxyz[1]*s.imu_quat_wxyz[1]+s.imu_quat_wxyz[2]*s.imu_quat_wxyz[2]));
 p[1]=std::asin(std::clamp(2*(s.imu_quat_wxyz[0]*s.imu_quat_wxyz[2]-s.imu_quat_wxyz[3]*s.imu_quat_wxyz[1]),-1.f,1.f));
 for(unsigned i=0;i<3;i++)p[2+i]=s.imu_gyro_xyz[i]*cfg_.ang_vel_scale;
 for(unsigned i=0;i<18;i++){p[5+i]=(s.joint_q[i]-cfg_.default_joint_pos[i])*cfg_.dof_pos_scale;p[23+i]=s.joint_dq[i]*cfg_.dof_vel_scale;}
 for(unsigned i=0;i<12;i++)p[41+i]=last_action_[i];
 for(unsigned i=0;i<3;i++){p[57+i]=cmd[i]*cfg_.cmd_scale[i];p[60+i]=s.ee_local_xyz[i];}
 gait_phase_+=cfg_.control_dt*cfg_.gait_frequency_hz;
 if(gait_phase_>=1.)gait_phase_-=1.;
 if(std::abs(cmd[0])<=.1f&&std::abs(cmd[1])<=.1f&&std::abs(cmd[2])<=.1f)gait_phase_=0;
 p[66]=static_cast<float>(gait_phase_);
 for(unsigned i=0;i<4;i++){const double shift[4]={.5,0,0,.5};p[67+i]=static_cast<float>(std::sin(2.*3.141592653589793*(gait_phase_+shift[i])));}
 Observation799 o{};std::copy(p.begin(),p.end(),o.begin());o[71+5]=1.f;
 for(unsigned h=0;h<10;h++)std::copy(hist_[h].begin(),hist_[h].end(),o.begin()+89+h*71);
 if(step_count_++==0){for(auto& x:hist_)x=p;}else{for(unsigned i=0;i<9;i++)hist_[i]=hist_[i+1];hist_[9]=p;}
 return o;
}
void ObservationBuilder::acceptPolicyAction(const Joint18& a){last_action_=a;}
}
