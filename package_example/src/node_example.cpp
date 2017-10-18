#include "ros_base/ROSNode.h"
#include "std_msgs/String.h"

#include "package_example/node_configuration.h"
#include "package_example/logic.h"

class example_node : public ros_base::ROSNode {
private:
    bool prepare();
    void errorHandling();
    void tearDown();
    
    InternalState is;
    ros::Publisher pub;
    ros::Timer timer;
    void pubCallback(const ros::TimerEvent&);
public:
    example_node();
};

example_node::example_node() {
    setName(ros::this_node::getName());
}

void example_node::pubCallback(const ros::TimerEvent&) {
    pub.publish(increment(is.vars(), is.params()));
}

bool example_node::prepare() {
    Parameters p;
    handle.param<int>("increment", p.increment, 3);
    is.initialize(&p);
    
    pub = handle.advertise<std_msgs::String>("chatter", 10);
    timer = handle.createTimer(ros::Duration(1/2), &example_node::pubCallback, this);
    return true;
}

void example_node::errorHandling() {
    ROSNode::errorHandling();
}

void example_node::tearDown() {
    ROS_INFO("Node is shutting down");
    return;
}

void nodeSigintHandler(int sig) {
    g_request_shutdown = 1;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_example", ros::init_options::NoSigintHandler);
    signal(SIGINT, nodeSigintHandler);
    example_node node;
    node.start();
    return 0;
}
