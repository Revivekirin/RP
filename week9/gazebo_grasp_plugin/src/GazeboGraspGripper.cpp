#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <boost/pointer_cast.hpp>
#include <iostream>

#include <gazebo_grasp_plugin/GazeboGraspGripper.h>

using namespace gazebo;

GazeboGraspGripper::GazeboGraspGripper() : attached(false) {}

GazeboGraspGripper::~GazeboGraspGripper() {
  this->model.reset();
}

GazeboGraspGripper::GazeboGraspGripper(const GazeboGraspGripper& o)
    : model(o.model),
      gripperName(o.gripperName),
      linkNames(o.linkNames),
      palmLink(o.palmLink),
      collisionElems(o.collisionElems),
      fixedJoint(o.fixedJoint),
      disableCollisionsOnAttach(o.disableCollisionsOnAttach),
      attached(o.attached),
      attachedObjName(o.attachedObjName) {}

GazeboGraspGripper& GazeboGraspGripper::operator=(const GazeboGraspGripper& o) {
    if (this != &o) {
        model = o.model;
        gripperName = o.gripperName;
        linkNames = o.linkNames;
        palmLink = o.palmLink;
        collisionElems = o.collisionElems;
        fixedJoint = o.fixedJoint;
        disableCollisionsOnAttach = o.disableCollisionsOnAttach;
        attached = o.attached;
        attachedObjName = o.attachedObjName;
    }
    return *this;
}


bool GazeboGraspGripper::Init(physics::ModelPtr &_model,
    const std::string &_gripperName,
    const std::string &palmLinkName,
    const std::vector<std::string> &fingerLinkNames,
    bool _disableCollisionsOnAttach,
    std::map<std::string, physics::CollisionPtr> &_collisionElems) {

  this->gripperName = _gripperName;
  this->disableCollisionsOnAttach = _disableCollisionsOnAttach;
  this->model = _model;

  physics::PhysicsEnginePtr physics = this->model->GetWorld()->Physics();
  this->fixedJoint = physics->CreateJoint("fixed");
  
  std::vector<physics::LinkPtr> links = this->model->GetLinks();
  std::cout << "[GazeboGraspGripper] Available links in model '" 
          << this->model->GetName() << "':" << std::endl;

  for (auto &l : links) {
    std::cout << "  - " << l->GetName() << std::endl;
  }

  this->palmLink = this->model->GetLink(palmLinkName);
  if (!this->palmLink) {
    std::cout << "[GazeboGraspGripper] Palm link not found: " << palmLinkName << std::endl;
    return false;
  }

  for (auto &finger : fingerLinkNames) {
    this->linkNames.push_back(finger);
    physics::LinkPtr link = this->model->GetLink(finger);
    if (!link) {
      std::cout << "[GazeboGraspGripper] Finger link not found: " << finger << std::endl;
      continue;
    }
    for (unsigned int j = 0; j < link->GetChildCount(); ++j) {
      physics::CollisionPtr coll = link->GetCollision(j);
      if (!coll) continue;
      std::string name = coll->GetScopedName();
      this->collisionElems[name] = coll;
      _collisionElems[name] = coll;
    }
  }
  return !this->collisionElems.empty();
}

const std::string& GazeboGraspGripper::getGripperName() const {
  return this->gripperName;
}

bool GazeboGraspGripper::hasLink(const std::string& linkName) const {
  for (auto &l : this->linkNames) {
    if (l == linkName) return true;
  }
  return false;
}

bool GazeboGraspGripper::hasCollisionLink(const std::string& linkName) const {
  return (this->collisionElems.find(linkName) != this->collisionElems.end());
}

bool GazeboGraspGripper::isObjectAttached() const {
  return this->attached;
}

const std::string& GazeboGraspGripper::attachedObject() const {
  return this->attachedObjName;
}

bool GazeboGraspGripper::HandleAttach(const std::string &objName) {
  if (!this->palmLink) return false;

  physics::WorldPtr world = this->model->GetWorld();
  physics::EntityPtr entity = world->EntityByName(objName);
  if (!entity) {
    gzerr << "Entity not found: " << objName << std::endl;
    return false;
  }

  physics::CollisionPtr obj = boost::dynamic_pointer_cast<physics::Collision>(entity);
  if (!obj) {
    gzerr << "Entity is not a collision: " << objName << std::endl;
    return false;
  }

  ignition::math::Pose3d diff =
      obj->GetLink()->WorldPose() - this->palmLink->WorldPose();
  this->fixedJoint->Load(this->palmLink, obj->GetLink(), diff);
  this->fixedJoint->Init();

#if GAZEBO_MAJOR_VERSION >= 8
  this->fixedJoint->SetUpperLimit(0, 0);
  this->fixedJoint->SetLowerLimit(0, 0);
#else
  this->fixedJoint->SetHighStop(0, 0);
  this->fixedJoint->SetLowStop(0, 0);
#endif

  if (this->disableCollisionsOnAttach) {
    obj->GetLink()->SetCollideMode("none");
  }

  this->attached = true;
  this->attachedObjName = objName;
  return true;
}

void GazeboGraspGripper::HandleDetach(const std::string &objName) {
  physics::WorldPtr world = this->model->GetWorld();
  physics::EntityPtr entity = world->EntityByName(objName);
  if (!entity) return;

  physics::CollisionPtr obj = boost::dynamic_pointer_cast<physics::Collision>(entity);
  if (obj && this->disableCollisionsOnAttach) {
    obj->GetLink()->SetCollideMode("all");
  }

  if (this->fixedJoint) this->fixedJoint->Detach();

  this->attached = false;
  this->attachedObjName.clear();
}

