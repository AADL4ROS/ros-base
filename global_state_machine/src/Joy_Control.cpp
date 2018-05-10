/**
 * Node Joy_Control
 * File auto-generated on 22/11/2017 16:12:19
 */
#include "node_base/ROSNode.h"
#include "global_state_machine/Joy_Control_configuration.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "global_state_machine/joy_to_cmd.h"


class Joy_Control : public node_base::ROSNode {
private:
    bool prepare();
    void tearDown();
    void errorHandling();
    void joy_to_cmd_callback(const sensor_msgs::Joy::ConstPtr& msg);
    InternalState is;
    ros::Subscriber sub_joy_to_cmd;
    ros::Publisher pub_joy_to_cmd;
public:
    Joy_Control();
};

/**
 * Method nodeSigintHandler auto-generated
 */
void nodeSigintHandler(int sig) {
    g_request_shutdown = 1;
}

/**
 * Method main auto-generated
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "Joy_Control", ros::init_options::NoSigintHandler);
    signal(SIGINT, nodeSigintHandler);
    Joy_Control node;
    node.start();
    return 0;
}

/**
 * Method prepare auto-generated
 */
bool Joy_Control::prepare() {
    Parameters p;
    handle.param<int>("button_mapping/halt", p.button_mapping.halt, 9);
    handle.param<int>("button_mapping/safe", p.button_mapping.safe, 1);
    handle.param<int>("button_mapping/manual", p.button_mapping.manual, 0);
    handle.param<int>("button_mapping/assisted", p.button_mapping.assisted, 2);
    handle.param<int>("button_mapping/autonomous", p.button_mapping.autonomous, 3);
    handle.param<int>("button_mapping/enabler", p.button_mapping.enabler, 4);
    handle.param<int>("axis_mapping/forward", p.axis_mapping.forward, 7);
    handle.param<int>("axis_mapping/rotate", p.axis_mapping.rotate, 3);
    is.initialize(&p);
    sub_joy_to_cmd = handle.subscribe("/joy", 1, &Joy_Control::joy_to_cmd_callback, this);
    pub_joy_to_cmd = handle.advertise<geometry_msgs::Twist>("/joy_cmd", 10);
    return true;
}

/**
 * Method tearDown auto-generated
 */
void Joy_Control::tearDown() {
    ROS_INFO("Node is shutting down");
    return;
}

/**
 * Method errorHandling auto-generated
 */
void Joy_Control::errorHandling() {
    ROSNode::errorHandling();
}

/**
 * Method joy_to_cmd_callback auto-generated
 */
void Joy_Control::joy_to_cmd_callback(const sensor_msgs::Joy::ConstPtr& msg) {
    pub_joy_to_cmd.publish(joyToCmd( is.vars(), is.params(), msg));
}

/**
 * Method Joy_Control auto-generated
 */
Joy_Control::Joy_Control() {
    setName(ros::this_node::getName());
}

