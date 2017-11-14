#ifndef _TF_INTERFACE_H_
#define _TF_INTERFACE_H_

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

namespace ros_base {
    struct Transform {
        std::array<double, 4> rotation;
        std::array<double, 3> position;
        std::string parent;
        std::string child;
        double time;
    };
    
    class TransformationFrames {
    private:
        tf::TransformBroadcaster broadcaster;
        tf::TransformListener listener;
        
    public:
        void sendTransform(Transform t);
        Transform retriveTransform(std::string, std::string, double time = 0);
    };
    
    Transform TransformationFrames::retriveTransform(std::string parent, std::string child, double time) {
        tf::StampedTransform tr;
        try {
            listener.lookupTransform(parent, child, ros::Time(time), tr);
        }
        catch (tf::TransformException ex) {
            //TODO
            throw ex;
        }
        
        Transform t;
        
        t.position = {tr.getOrigin().x(), tr.getOrigin().y(), tr.getOrigin().z()};
        t.rotation = {tr.getRotation().x(), tr.getRotation().y(), tr.getRotation().z(), tr.getRotation().w()};
        t.parent = parent;
        t.child = child;
        t.time = time;
        return t;
    }
    
    void TransformationFrames::sendTransform(Transform t) {
         tf::Transform transform(
             tf::Quaternion(t.rotation[0], t.rotation[1], t.rotation[2], t.rotation[3]),
             tf::Vector3(t.position[0], t.position[1], t.position[2])
         );
         broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time(t.time), t.parent, t.child));
    }

};

#endif
