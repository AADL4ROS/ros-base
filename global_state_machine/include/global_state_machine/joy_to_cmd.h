#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

geometry_msgs::Twist joyToCmd(Variables_ptr v, Parameters_ptr p, const sensor_msgs::Joy::ConstPtr& msg) {
  if(msg->buttons[p->button_mapping.halt] == 1)
      v->smi.setGlobalState(0);
  if(msg->buttons[p->button_mapping.safe] == 1)
      v->smi.setGlobalState(1);
  if(msg->buttons[p->button_mapping.manual] == 1)
      v->smi.setGlobalState(2);
  if(msg->buttons[p->button_mapping.assisted] == 1)
      v->smi.setGlobalState(3);
  if(msg->buttons[p->button_mapping.autonomous] == 1)
      v->smi.setGlobalState(4);
  
  geometry_msgs::Twist out;
  if(msg->buttons[p->button_mapping.enabler] == 1) {
      //TODO parametrize this
      out.linear.x = msg->axes[p->axis_mapping.forward] * 0.7782;
      out.angular.z = msg->axes[p->axis_mapping.rotate] * 2.3809;
  }
  
  return out;
}

