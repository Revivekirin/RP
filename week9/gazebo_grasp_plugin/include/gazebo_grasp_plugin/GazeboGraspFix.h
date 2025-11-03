#ifndef GAZEBO_GAZEBOGRASPFIX_H
#define GAZEBO_GAZEBOGRASPFIX_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <map>
#include <string>
#include <mutex>
#include <vector>
#include <boost/thread/mutex.hpp>

#include "gazebo_grasp_plugin/GazeboGraspGripper.h"

namespace gazebo {

// 충돌 지점 정보
struct CollidingPoint {
    std::string gripperName;
    physics::CollisionPtr collLink;
    physics::CollisionPtr collObj;
    ignition::math::Vector3d force;
    ignition::math::Vector3d pos;
    int sum = 0;
};

// 객체별 접촉 정보
struct ObjectContactInfo {
    std::vector<ignition::math::Vector3d> appliedForces;
    std::map<std::string, int> grippersInvolved;
    int maxGripperContactCnt = 0;
    std::string maxContactGripper;
};

class GazeboGraspFix : public ModelPlugin {
public:
    GazeboGraspFix();
    virtual ~GazeboGraspFix();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

protected:
    void OnUpdate();
    void OnContact(const ConstContactsPtr &_msg);

private:
    physics::ModelPtr model;
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;

    // Gazebo transport
    transport::NodePtr node;
    transport::SubscriberPtr contactSub;
    std::string filterName;

    // grippers
    std::map<std::string, GazeboGraspGripper> grippers;
    std::map<std::string, std::string> collisions; // collision 이름 → gripper 이름 매핑

    // contacts
    boost::mutex mutexContacts;
    std::map<std::string, std::map<std::string, CollidingPoint>> contacts;

    // grasp counts
    std::map<std::string, int> gripCounts;

    // parameters
    bool disableCollisionsOnAttach;
    double forcesAngleTolerance;
    int maxGripCount;
    int gripCountThreshold;
    double releaseTolerance;

    int maxContactCount;        
    int contactCountThreshold;  
    int contact_counts; 

    common::Time prevUpdateTime;
};

} // namespace gazebo

#endif // GAZEBO_GAZEBOGRASPFIX_H

