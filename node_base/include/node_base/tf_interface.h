#ifndef _TF_INTERFACE_H_
#define _TF_INTERFACE_H_

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

namespace node_base {
    class TransformationFrames {
    private:
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        tf2_ros::TransformBroadcaster br;
    public:
        void sendTransform(std::string parent, std::string child, double time, geometry_msgs::Transform t);
        geometry_msgs::Transform retriveTransform(std::string, std::string, double time = 0);
        TransformationFrames();
    };
    
    geometry_msgs::Transform TransformationFrames::retriveTransform(std::string parent, std::string child, double time) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform(parent, child, ros::Time(time));
        } catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM(ex.what());
            ROS_WARN_STREAM("lookup failed, trying waiting for 0.5 s");
            ros::Duration(0.5).sleep();
            try {
                transformStamped = tfBuffer.lookupTransform(parent, child, ros::Time(time));
            } catch (tf2::TransformException &ex) {
                ROS_FATAL_STREAM(ex.what());
                ROS_FATAL_STREAM("Cannot find transform after waiting, abort");
                //TODO
            }
        }
        return transformStamped.transform;
    }
    
    void TransformationFrames::sendTransform(std::string parent, std::string child, double time, geometry_msgs::Transform t) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time(time);
        transformStamped.header.frame_id = parent;
        transformStamped.child_frame_id = child;
        transformStamped.transform = t;
        br.sendTransform(transformStamped);
    }
    
    TransformationFrames::TransformationFrames() : tfBuffer(), tfListener(tfBuffer) {}

};

#endif
