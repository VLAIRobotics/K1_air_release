// All SDK entrypoints are test stubs: no CAN socket or motor can be accessed.
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include "xarm_hardware/v10_simple_hardware.hpp"
#include "legacy_joint_limits_fixture.hpp"
namespace {
using HW = xarm_hardware::XArm_v10HW;
using Ret = hardware_interface::return_type;
using Callback = hardware_interface::CallbackReturn;
std::array<xarm_sdk_mit_param_t,7> sent{};
double measured=0.3;
double joint1_override=std::numeric_limits<double>::quiet_NaN();
int transmissions=0,enables=0,disables=0;
bool fail_read=false;
void require(bool ok,const char* why) {if(!ok) throw std::runtime_error(why);}
void close(double a,double b,const char* why) {require(std::abs(a-b)<1e-5,why);}
hardware_interface::HardwareInfo info(bool hand=true,const std::string& prefix="") {
 hardware_interface::HardwareInfo i;i.name="offline_hold";i.type="system";
 i.hardware_parameters["hand"]=hand?"true":"false";
 std::ostringstream x;x<<"<robot name='test'><link name='base'/>";
 for(int n=1;n<=7;++n) {
  const auto bound=legacy_test::joint_position_limits(n-1,prefix=="left_");
  x<<"<link name='xarm_link"<<n<<"'/><joint name='xarm_joint"<<n<<"' type='revolute'><parent link='"<<(n==1?"base":"xarm_link"+std::to_string(n-1))<<"'/><child link='xarm_link"<<n<<"'/><origin xyz='0.1 0 0'/><axis xyz='0 1 0'/><limit lower='"<<bound.lower<<"' upper='"<<bound.upper<<"' effort='100' velocity='1'/></joint>";
 }
 x<<"<link name='xarm_hand'><inertial><origin xyz='0.1 0 0'/><mass value='1'/><inertia ixx='0.01' iyy='0.01' izz='0.01' ixy='0' ixz='0' iyz='0'/></inertial></link><joint name='hand_fixed' type='fixed'><parent link='xarm_link7'/><child link='xarm_hand'/><origin xyz='0.1 0 0'/></joint></robot>";
 i.original_xml=x.str();i.hardware_parameters["arm_prefix"]=prefix;
 if(!prefix.empty()) {
  std::size_t pos=0;
  while((pos=i.original_xml.find("xarm_",pos))!=std::string::npos) {
   i.original_xml.insert(pos+5,prefix);pos+=5+prefix.size();
  }
 }
 for(int n=1;n<=7;++n) {hardware_interface::ComponentInfo j;j.name="xarm_"+prefix+"joint"+std::to_string(n);i.joints.push_back(j);}
 if(hand){hardware_interface::ComponentInfo j;j.name="xarm_"+prefix+"finger_joint1";i.joints.push_back(j);}
 return i;
}
std::vector<std::string> keys(const std::string& mode) {
 std::vector<std::string> k;for(int n=1;n<=7;++n)k.push_back("xarm_joint"+std::to_string(n)+"/"+mode);return k;
}
void set_commands(HW& h,const std::string& mode,double value) {
 auto c=h.export_command_interfaces();for(auto& v:c)if(v.get_interface_name()==mode && v.get_name().find("finger")==std::string::npos)v.set_value(value);
}
}
extern "C" {
int xarm_sdk_create(const char*,int,xarm_sdk_handle_t* h){*h=&transmissions;return 0;}
int xarm_sdk_destroy(xarm_sdk_handle_t){return 0;}
int xarm_sdk_init_arm_motors(xarm_sdk_handle_t,const int*,const uint32_t*,const uint32_t*,int){return 0;}
int xarm_sdk_init_gripper_motor(xarm_sdk_handle_t,int,uint32_t,uint32_t){return 0;}
int xarm_sdk_enable_all(xarm_sdk_handle_t){++enables;return 0;}
int xarm_sdk_disable_all(xarm_sdk_handle_t){++disables;return 0;}
int xarm_sdk_set_callback_mode_state_all(xarm_sdk_handle_t){return 0;}
int xarm_sdk_refresh_all(xarm_sdk_handle_t){return fail_read?-4:0;}
int xarm_sdk_recv_all(xarm_sdk_handle_t,int){return 0;}
int xarm_sdk_get_arm_joint_states(xarm_sdk_handle_t,xarm_sdk_joint_state_t* s,int n){for(int j=0;j<n;++j)s[j]={static_cast<float>(j==0 && std::isfinite(joint1_override)?joint1_override:measured),0,0};return 0;}
int xarm_sdk_get_gripper_state(xarm_sdk_handle_t,xarm_sdk_joint_state_t* s){*s={0,0,0};return 0;}
int xarm_sdk_arm_mit_control(xarm_sdk_handle_t,const xarm_sdk_mit_param_t* p,int n){require(n==7,"DOF");std::copy_n(p,n,sent.begin());++transmissions;return 0;}
int xarm_sdk_gripper_mit_control(xarm_sdk_handle_t,const xarm_sdk_mit_param_t*){return 0;}
int xarm_sdk_get_last_error(char* b,int n){if(n)*b=0;return 0;}
}
int main(){try{
 rclcpp::Time t(0);auto dt=rclcpp::Duration::from_seconds(.01);rclcpp_lifecycle::State state;
 HW h;require(h.on_init(info())==Callback::SUCCESS,"init");
 require(h.write(t,dt)==Ret::ERROR && transmissions==0,"write before feedback");
 require(h.on_activate(state)==Callback::SUCCESS,"activate");
 require(enables==1 && transmissions==0,"activation issued motion/zero trajectory");
 require(h.write(t,dt)==Ret::OK,"first write");
 for(const auto& p:sent)close(p.pos,.3,"startup did not hold measured pose");
 double gravity=0;for(const auto& p:sent)gravity+=std::abs(p.torque);
 require(gravity>1,"hand mass missing from gravity compensation");
 set_commands(h,"position",0);require(h.write(t,dt)==Ret::OK,"explicit zero command");
 for(const auto& p:sent)close(p.pos,0,"explicit zero incorrectly treated as no command");
 require(h.perform_command_mode_switch(keys("effort"),keys("position"))==Ret::OK,"effort switch");
 set_commands(h,"effort",.2);require(h.write(t,dt)==Ret::OK,"effort write");
 for(const auto& p:sent){close(p.kp,0,"effort kp");close(p.kd,0,"effort kd");close(p.torque,.2,"gravity double added in effort mode");}
 require(h.perform_command_mode_switch(keys("position"),keys("effort"))==Ret::OK,"position switch");
 auto commands=h.export_command_interfaces();for(auto& c:commands)if(c.get_name().find("finger")==std::string::npos){
  if(c.get_interface_name()=="position")close(c.get_value(),.3,"stale position restored after effort");
  else close(c.get_value(),0,"stale velocity or effort");
 }
 require(h.write(t,dt)==Ret::OK,"position after effort");for(const auto& p:sent)close(p.pos,.3,"position switch moved toward zero");
 set_commands(h,"position",.5);require(h.perform_command_mode_switch({},keys("position"))==Ret::OK,"stop only switch");
 require(h.write(t,dt)==Ret::OK,"hold after stop");for(const auto& p:sent)close(p.pos,.3,"old trajectory continued after controller stop");
 require(h.prepare_command_mode_switch({"xarm_joint1/position","xarm_joint1/effort"},{})==Ret::ERROR,"conflicting modes accepted");
 require(h.prepare_command_mode_switch(keys("velocity"),{})==Ret::ERROR,"unsupported velocity mode accepted");
 set_commands(h,"position",std::numeric_limits<double>::infinity());int count=transmissions;
 require(h.write(t,dt)==Ret::ERROR && transmissions==count,"nonfinite command reached SDK");
 set_commands(h,"position",std::numeric_limits<double>::quiet_NaN());require(h.write(t,dt)==Ret::OK,"missing position fallback");
 for(const auto& p:sent)close(p.pos,.3,"missing command did not hold");
 HW bad;require(bad.on_init(info())==Callback::SUCCESS,"bad init");measured=std::numeric_limits<double>::quiet_NaN();count=transmissions;
 require(bad.on_activate(state)==Callback::ERROR && disables==1 && transmissions==count,"invalid activation feedback not rejected");
 measured=4;HW limits;require(limits.on_init(info())==Callback::SUCCESS,"limits init");
 require(limits.on_activate(state)==Callback::ERROR && disables==2,"out of bounds activation accepted");
 measured=.3;fail_read=true;HW failed;require(failed.on_init(info())==Callback::SUCCESS,"SDK-fail init");
 require(failed.on_activate(state)==Callback::ERROR && disables==3,"feedback SDK error ignored");fail_read=false;
 HW nohand;require(nohand.on_init(info(false))==Callback::SUCCESS,"no-hand init");require(nohand.read(t,dt)==Ret::OK,"nohand read");require(nohand.write(t,dt)==Ret::OK,"nohand write");
 for(const auto& p:sent)close(p.torque,0,"unexpected torque for massless arm without hand");
 // Mirrored left J1 must use the same range at activation, mode switch and write.
 joint1_override=-3.0;HW left;require(left.on_init(info(true,"left_"))==Callback::SUCCESS,"left init");
 require(left.on_activate(state)==Callback::SUCCESS,"valid left J1 rejected at activation");
 require(left.write(t,dt)==Ret::OK,"left write");close(sent[0].pos,-3.0,"left J1 clamped to right-arm limit");
 std::vector<std::string> leftkeys={"xarm_left_joint1/position"};
 require(left.perform_command_mode_switch(leftkeys,{})==Ret::OK,"left switch rejected");
 require(left.write(t,dt)==Ret::OK,"left switch write");close(sent[0].pos,-3.0,"left switch changed target");
 joint1_override=2.0;HW outside;require(outside.on_init(info(true,"left_"))==Callback::SUCCESS,"left upper init");
 count=disables;require(outside.on_activate(state)==Callback::ERROR && disables==count+1,"left upper range not applied at activation");
 joint1_override=-1.69;HW right;require(right.on_init(info())==Callback::SUCCESS,"right init");
 require(right.on_activate(state)==Callback::SUCCESS,"perfect J1 lower extension rejected");
 require(right.write(t,dt)==Ret::OK,"right write");close(sent[0].pos,-1.69,"right J1 range not updated");
 joint1_override=std::numeric_limits<double>::quiet_NaN();
 std::cout<<"startup hold, mode transitions, zero target, invalid feedback, hand gravity and torque ownership: PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
