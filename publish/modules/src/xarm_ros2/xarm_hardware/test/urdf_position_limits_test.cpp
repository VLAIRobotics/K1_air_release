// All SDK entrypoints are test stubs: no CAN socket or motor can be accessed.
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <fstream>
#include <hardware_interface/component_parser.hpp>
#include <urdf_parser/urdf_parser.h>
#include <sstream>
#include <stdexcept>
#include "xarm_hardware/v10_simple_hardware.hpp"
namespace {
using HW = xarm_hardware::XArm_v10HW;
using Ret = hardware_interface::return_type;
using Callback = hardware_interface::CallbackReturn;
std::array<xarm_sdk_mit_param_t,7> sent{};
std::array<double,7> measured{};
int transmissions=0,enables=0,disables=0,creates=0;
std::string requested_can;
bool fail_read=false;
void require(bool ok,const char* why) {if(!ok) throw std::runtime_error(why);}

}
extern "C" {
int xarm_sdk_create(const char* name,int,xarm_sdk_handle_t* h){++creates;requested_can=name;*h=&transmissions;return 0;}
int xarm_sdk_destroy(xarm_sdk_handle_t){return 0;}
int xarm_sdk_init_arm_motors(xarm_sdk_handle_t,const int*,const uint32_t*,const uint32_t*,int){return 0;}
int xarm_sdk_init_gripper_motor(xarm_sdk_handle_t,int,uint32_t,uint32_t){return 0;}
int xarm_sdk_enable_all(xarm_sdk_handle_t){++enables;return 0;}
int xarm_sdk_disable_all(xarm_sdk_handle_t){++disables;return 0;}
int xarm_sdk_set_callback_mode_state_all(xarm_sdk_handle_t){return 0;}
int xarm_sdk_refresh_all(xarm_sdk_handle_t){return fail_read?-4:0;}
int xarm_sdk_recv_all(xarm_sdk_handle_t,int){return 0;}
int xarm_sdk_get_arm_joint_states(xarm_sdk_handle_t,xarm_sdk_joint_state_t* s,int n){for(int j=0;j<n;++j)s[j]={static_cast<float>(measured[j]),0,0};return 0;}
int xarm_sdk_get_gripper_state(xarm_sdk_handle_t,xarm_sdk_joint_state_t* s){*s={0,0,0};return 0;}
int xarm_sdk_arm_mit_control(xarm_sdk_handle_t,const xarm_sdk_mit_param_t* p,int n){require(n==7,"DOF");std::copy_n(p,n,sent.begin());++transmissions;return 0;}
int xarm_sdk_gripper_mit_control(xarm_sdk_handle_t,const xarm_sdk_mit_param_t*){return 0;}
int xarm_sdk_get_last_error(char* b,int n){if(n)*b=0;return 0;}
}
int main(int argc,char**argv){try{
 require(argc==2,"URDF path");std::ifstream f(argv[1]);std::string xml((std::istreambuf_iterator<char>(f)),{});
 auto resources=hardware_interface::parse_control_resources_from_urdf(xml);require(!resources.empty() && resources.size()<=2,"one or two arm resources");
 for(auto info:resources){
  auto model=urdf::parseURDF(info.original_xml);std::array<double,7> lower{},upper{};
  for(int j=0;j<7;++j){auto l=model->getJoint(info.joints[j].name)->limits;lower[j]=l->lower;upper[j]=l->upper;measured[j]=(lower[j]+upper[j])/2;}
  HW hw;require(hw.on_init(info)==Callback::SUCCESS,"valid init");rclcpp_lifecycle::State state;rclcpp::Time t(0);auto dt=rclcpp::Duration::from_seconds(.01);
  require(hw.on_activate(state)==Callback::SUCCESS,"midpoint activation");
  auto commands=hw.export_command_interfaces();
  for(int direction:{-1,1}){
   for(auto& c:commands)for(int j=0;j<7;++j)if(c.get_prefix_name()==info.joints[j].name && c.get_interface_name()=="position") c.set_value(direction<0?lower[j]-1:upper[j]+1);
   require(hw.write(t,dt)==Ret::OK,"clamp write");
   for(int j=0;j<7;++j)require(std::abs(sent[j].pos-(direction<0?lower[j]:upper[j]))<1e-5,"clamp differs from URDF");
  }
  std::vector<std::string> keys;for(int j=0;j<7;++j)keys.push_back(info.joints[j].name+"/position");
  // Exact URDF endpoints round through the actual SDK float representation.
  for(int direction:{-1,1}){
   for(int j=0;j<7;++j){float q=static_cast<float>(direction<0?lower[j]:upper[j]);measured[j]=q;}
   HW endpoint;require(endpoint.on_init(info)==Callback::SUCCESS,"endpoint init");require(endpoint.on_activate(state)==Callback::SUCCESS,"endpoint activation");require(endpoint.perform_command_mode_switch(keys,{})==Ret::OK,"endpoint switch");
  }
  measured[3]=std::nextafter(static_cast<float>(upper[3]), INFINITY);
  HW one_step;require(one_step.on_init(info)==Callback::SUCCESS,"float step init");require(one_step.on_activate(state)==Callback::ERROR,"one float step beyond upper accepted");
  measured[3]=lower[3]-.01;HW outside;require(outside.on_init(info)==Callback::SUCCESS,"outside init");require(outside.on_activate(state)==Callback::ERROR,"outside activation accepted");
  require(hw.read(t,dt)==Ret::OK,"outside read");require(hw.prepare_command_mode_switch(keys,{})==Ret::ERROR,"outside prepare accepted");require(hw.perform_command_mode_switch(keys,{})==Ret::ERROR,"outside switch accepted");
  for(int j=0;j<7;++j)measured[j]=(lower[j]+upper[j])/2;
  auto reject=[&](const std::string& mutation){auto bad=info;bad.original_xml=mutation;int before=creates;HW invalid;require(invalid.on_init(bad)==Callback::ERROR && creates==before,"bad limits opened SDK");};
  const auto name=info.joints[0].name;size_t start=xml.find("<joint name=\""+name+"\"");require(start!=std::string::npos,"joint XML");size_t end=xml.find("</joint>",start)+8;std::string joint=xml.substr(start,end-start);
  for(const std::string type:{"fixed","continuous","prismatic"}){auto changed=joint;auto at=changed.find("type=\"revolute\"");changed.replace(at,15,"type=\""+type+"\"");auto bad=xml;bad.replace(start,end-start,changed);reject(bad);}
  auto mutate_limit=[&](const std::string& limit){auto changed=joint;auto a=changed.find("<limit ");auto b=changed.find("/>",a)+2;changed.replace(a,b-a,limit);auto bad=xml;bad.replace(start,end-start,changed);reject(bad);};
  mutate_limit("");mutate_limit("<limit lower='nan' upper='2' effort='1' velocity='1'/>");mutate_limit("<limit lower='-1' upper='inf' effort='1' velocity='1'/>");mutate_limit("<limit lower='2' upper='-1' effort='1' velocity='1'/>");mutate_limit("<limit lower='1' upper='1' effort='1' velocity='1'/>");
  auto changed=joint;changed.insert(changed.find("</joint>"),"<mimic joint='"+info.joints[1].name+"' multiplier='1'/>");auto bad=xml;bad.replace(start,end-start,changed);reject(bad);
 }
 std::cout<<"URDF per-axis clamp, activation/switch bounds, invalid model pre-SDK guards PASS (all SDK calls stubbed)\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
