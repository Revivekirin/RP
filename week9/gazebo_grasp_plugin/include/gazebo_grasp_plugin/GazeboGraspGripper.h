#ifndef GAZEBO_GAZEBOGRASPGRIPPER_H
#define GAZEBO_GAZEBOGRASPGRIPPER_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <string>
#include <vector>
#include <map>

namespace gazebo {

class GazeboGraspGripper {
public:
    GazeboGraspGripper();
    GazeboGraspGripper(const GazeboGraspGripper& o);
    GazeboGraspGripper& operator=(const GazeboGraspGripper& o);
    virtual ~GazeboGraspGripper();

    bool Init(physics::ModelPtr& _model,
        const std::string& _gripperName,
        const std::string& palmLinkName,
        const std::vector<std::string>& fingerLinkNames,
        bool _disableCollisionsOnAttach,
        std::map<std::string, physics::CollisionPtr>& _collisions);

    const std::string& getGripperName() const;
    bool hasLink(const std::string& linkName) const;
    bool hasCollisionLink(const std::string& linkName) const;

    bool isObjectAttached() const;
    const std::string& attachedObject() const;

    bool HandleAttach(const std::string& objName);
    void HandleDetach(const std::string& objName);

    // 공개 접근 필요 (Fix에서 palm pose 접근)
    physics::LinkPtr palmLink;
    physics::ModelPtr model;

private:
    std::string gripperName;

    std::vector<std::string> linkNames;
    std::map<std::string, physics::CollisionPtr> collisionElems;

    physics::JointPtr fixedJoint;
    bool disableCollisionsOnAttach;

    bool attached;
    std::string attachedObjName;
};

} // namespace gazebo

#endif  // GAZEBO_GAZEBOGRASPGRIPPER_H

