#include "ros_base/Overseer.h"
#include "std_msgs/Int32.h"
#include "tinyxml2/tinyxml2.h"

Overseer::Overseer() : gsm(0) {
    tinyxml2::XMLDocument doc;
    doc.LoadFile("/home/gianluca/node_list.xml");
    tinyxml2::XMLElement * element = doc.FirstChild()->FirstChildElement("node");
    while(element != nullptr) {
        std::string name = element->FirstChildElement("name")->GetText();
        name = "/" + name;
        state_map[name] = 666;
        if(element->FirstChildElement("critical") != nullptr) {
            ROS_INFO_STREAM("Found a critical node: "<<name);
            float f = element->FirstChildElement("frequency")->FloatText();
            critical_map[name] = handle.createTimer(ros::Duration((1/f)*1.15), &Overseer::criticalViolation, this, false, false);
        }
        element = element->NextSiblingElement("node");
    }
}

bool Overseer::prepare() {
    state_receiver = handle.advertiseService("/state_notifier", &Overseer::receiveState, this);
    state_provider = handle.advertiseService("/state_query", &Overseer::provideState, this);
    set_global_state = handle.advertiseService("/set_global_state", &Overseer::setGlobalState, this);
    get_global_state = handle.advertiseService("/get_global_state", &Overseer::getGlobalState, this);
    notification = handle.advertise<std_msgs::Int32>("/global_state", 10);
    return true;
}


bool Overseer::receiveState(state_machine_msgs::SendState::Request& req, state_machine_msgs::SendState::Response& res) {
    state_map[req.node] = req.state;
    ROS_INFO_STREAM("Received state "<<req.state<<" from node "<<req.node);
    auto it = critical_map.find(req.node);
    if (it != critical_map.end()) {
        ROS_INFO_STREAM("Node "<<req.node<<" is critical, refreshing timer");
        ros::Timer& timer = it->second;
        timer.stop();
        timer.start();
    }
    return true;
}

bool Overseer::provideState(state_machine_msgs::GetState::Request& req, state_machine_msgs::GetState::Response& res) {
    if (state_map.count(req.node) > 0) {
        res.state = state_map[req.node];
        return true;        
    } else {
        return false;
    }
}

bool Overseer::getGlobalState(state_machine_msgs::GetGlobalState::Request& req, state_machine_msgs::GetGlobalState::Response& res) {
    res.state = gsm.getState();
    return true;
}

bool Overseer::setGlobalState(state_machine_msgs::SetGlobalState::Request& req, state_machine_msgs::SetGlobalState::Response& res) {
    return gsm.setState(req.state);
}


void Overseer::criticalViolation(const ros::TimerEvent& event) {
    gsm.setState(3);
    ROS_FATAL_STREAM("Critical node timeout, forcing "<<gsm.getState()<<" state");
}

void Overseer::run() {
    std_msgs::Int32 msg;
    msg.data = gsm.getState();
    notification.publish(msg);
}

void Overseer::shutdown() {
    gsm.cleanUp();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "overseer");
    Overseer node;
    ros::Rate r(10);
    if(!node.prepare())
        return -1;
    while(ros::ok()) {
        node.run();
        ros::spinOnce();
        r.sleep();
    }
    node.shutdown();
    return 0;
}
