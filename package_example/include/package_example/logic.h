#include "std_msgs/String.h"
#include "package_example/node_configuration.h"

std_msgs::String increment(Variables_ptr v, Parameters_ptr p, node_base::TransformationFrames * tf) {
    tf->retriveTransform("test", "test", 0.0);
    std_msgs::String msg;
    std::stringstream ss;
    v->incrementer += p->increment;
    ss << "counting to "<<v->incrementer;
    msg.data = ss.str().c_str();
    
    return msg;
}
