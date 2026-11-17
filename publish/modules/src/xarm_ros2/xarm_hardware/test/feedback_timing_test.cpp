// Every SDK call is stubbed: this test never opens CAN or enables motors.
#include <chrono>
#include <stdexcept>
#include <iostream>
#include "xarm_hardware/v10_simple_hardware.hpp"
namespace {
using Clock = std::chrono::steady_clock;
Clock::time_point ready;
int requested = 0, cached = 0;
bool pending = false, fail = false;
void check(bool ok, const char* why) {if (!ok) throw std::runtime_error(why);}
}
extern "C" {
int xarm_sdk_create(const char*, int, xarm_sdk_handle_t* h) {*h=&requested; return 0;}
int xarm_sdk_destroy(xarm_sdk_handle_t) {return 0;}
int xarm_sdk_init_arm_motors(xarm_sdk_handle_t,const int*,const uint32_t*,const uint32_t*,int) {return 0;}
int xarm_sdk_init_gripper_motor(xarm_sdk_handle_t,int,uint32_t,uint32_t) {return 0;}
int xarm_sdk_enable_all(xarm_sdk_handle_t) {throw std::runtime_error("unexpected enable");}
int xarm_sdk_disable_all(xarm_sdk_handle_t) {throw std::runtime_error("unexpected disable");}
int xarm_sdk_set_callback_mode_state_all(xarm_sdk_handle_t) {return 0;}
int xarm_sdk_refresh_all(xarm_sdk_handle_t) {
 if(fail) return -4;
 ++requested; pending=true; ready=Clock::now()+std::chrono::microseconds(1800); return 0;
}
int xarm_sdk_recv_all(xarm_sdk_handle_t,int timeout) {
 check(timeout==0,"blocking per-frame timeout");
 if(pending && Clock::now()>=ready) {cached=requested;pending=false;} return 0;
}
int xarm_sdk_get_arm_joint_states(xarm_sdk_handle_t,xarm_sdk_joint_state_t* s,int n) {
 for(int i=0;i<n;++i) {s[i]={0.3F,static_cast<float>(cached),0};}
 return 0;
}
int xarm_sdk_get_gripper_state(xarm_sdk_handle_t,xarm_sdk_joint_state_t* s) {*s={};return 0;}
int xarm_sdk_arm_mit_control(xarm_sdk_handle_t,const xarm_sdk_mit_param_t* p,int n) {
 check(n==7,"joint count");
 for(int i=0;i<n;++i) check(p[i].kp==0 && p[i].kd==0 && std::abs(p[i].torque-0.2F)<1e-6,"effort changed");
 return 0;
}
int xarm_sdk_gripper_mit_control(xarm_sdk_handle_t,const xarm_sdk_mit_param_t*) {return 0;}
int xarm_sdk_get_last_error(char* b,int n) {if(n) *b=0;return 0;}
}
int main() {
 try {
  hardware_interface::HardwareInfo info;
  info.name="timing_test";info.type="system";info.hardware_parameters["hand"]="false";
  info.original_xml="<robot name='test'><link name='base'/>";
  for(int n=1;n<=7;++n) {
    hardware_interface::ComponentInfo j;j.name="xarm_joint"+std::to_string(n);info.joints.push_back(j);
    auto parent=n==1?"base":"xarm_link"+std::to_string(n-1);
    info.original_xml+="<link name='xarm_link"+std::to_string(n)+"'/><joint name='"+j.name+"' type='revolute'><parent link='"+parent+"'/><child link='xarm_link"+std::to_string(n)+"'/><axis xyz='0 1 0'/><limit lower='-3' upper='3' effort='100' velocity='1'/></joint>";
  }
  info.original_xml+="</robot>";
  xarm_hardware::XArm_v10HW hw;
  check(hw.on_init(info)==hardware_interface::CallbackReturn::SUCCESS,"init");
  rclcpp::Time t(0);auto dt=rclcpp::Duration::from_seconds(.01);
  check(hw.read(t,dt)==hardware_interface::return_type::OK,"read");
  auto states=hw.export_state_interfaces();
  for(auto& s:states) if(s.get_interface_name()=="velocity") check(s.get_value()==1,"stale feedback");
  std::vector<std::string> keys;
  for(int j=1;j<=7;++j) keys.push_back("xarm_joint"+std::to_string(j)+"/effort");
  check(hw.perform_command_mode_switch(keys,{})==hardware_interface::return_type::OK,"mode");
  auto cmds=hw.export_command_interfaces();
  for(auto& c:cmds) if(c.get_interface_name()=="effort") c.set_value(.2);
  check(hw.write(t,dt)==hardware_interface::return_type::OK,"write");
  fail=true;check(hw.read(t,dt)==hardware_interface::return_type::ERROR,"SDK error ignored");fail=false;
  info.hardware_parameters["feedback_wait_us"]="0";
  xarm_hardware::XArm_v10HW old;
  check(old.on_init(info)==hardware_interface::CallbackReturn::SUCCESS,"zero wait init");
  check(old.read(t,dt)==hardware_interface::return_type::OK,"zero wait read");
  check(cached<requested,"zero wait rollback");
  info.hardware_parameters["feedback_wait_us"]="3000";
  xarm_hardware::XArm_v10HW invalid;
  check(invalid.on_init(info)==hardware_interface::CallbackReturn::ERROR,"invalid wait accepted");
  std::cout<<"fresh reply, torque invariance, error propagation, rollback: PASS\n";return 0;
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
