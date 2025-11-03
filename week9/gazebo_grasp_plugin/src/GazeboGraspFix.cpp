#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <gazebo_grasp_plugin/GazeboGraspFix.h>
#include <gazebo_grasp_plugin/GazeboGraspGripper.h>

#include <iostream>
#include <map>
#include <set>
#include <cmath>

using namespace gazebo;

GazeboGraspFix::GazeboGraspFix() {
  this->disableCollisionsOnAttach = false;
//  this->forcesAngleTolerance = 120 * M_PI / 180.0;
  this->forcesAngleTolerance = 45 * M_PI / 180.0;

  this->maxGripCount = 10;
  this->gripCountThreshold = 5;
  this->maxContactCount = 10;
  this->contactCountThreshold = 5;
  this->contact_counts = 0;

  this->releaseTolerance = 0.005;
  this->prevUpdateTime = common::Time::GetWallTime(); 
}

GazeboGraspFix::~GazeboGraspFix() {
  this->updateConnection.reset();
  if (this->node) this->node->Fini();
  this->node.reset();
}

void GazeboGraspFix::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  std::cout << "[GazeboGraspFix] Loading plugin..." << std::endl;

  this->world = _parent->GetWorld();
  if (!this->world) {
    gzerr << "World is NULL" << std::endl;
    return;
  }
  std::cout << "[GazeboGraspFix] world OK" << std::endl;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->Name());

  for (sdf::ElementPtr armElem = _sdf->GetElement("arm"); armElem;
       armElem = armElem->GetNextElement("arm")) {
    std::string armName = armElem->Get<std::string>("arm_name");
    std::string palmLink = armElem->Get<std::string>("palm_link");
    
    std::cout << "[GazeboGraspFix] Initializing gripper: " << armName << std::endl;
   
    std::vector<std::string> fingerLinks;
    for (sdf::ElementPtr f = armElem->GetElement("gripper_link"); f;
         f = f->GetNextElement("gripper_link")) {
         std::cout << "[GazeboGraspFix] finger links: " << f->Get<std::string>() << std::endl;
      fingerLinks.push_back(f->Get<std::string>());
    }

    GazeboGraspGripper gripper;
    std::map<std::string, physics::CollisionPtr> colls;
    if (!gripper.Init(_parent, armName, palmLink, fingerLinks,
                      this->disableCollisionsOnAttach, colls)) {
      std::cout << "Failed to init gripper: " << armName << std::endl;
      continue;
    }
    
    std::cout << "[GazeboGraspFix] Gripper added: " << armName << std::endl;
    
    this->grippers[armName] = gripper;

    for (auto &kv : colls) {
      this->collisions[kv.first] = armName;
    }
  }

  if (this->grippers.empty()) {
    std::cout << "[GazeboGraspFix] No grippers configured, plugin will not work." << std::endl;
    return;
  }
  
  std::cout << "[GazeboGraspFix] Grippers ready, registering update event" << std::endl;

  // ContactManager subscribe
  physics::PhysicsEnginePtr physics = this->world->Physics();
  physics::ContactManager *contactManager = physics->GetContactManager();
  contactManager->PublishContacts();

  this->filterName = _parent->GetScopedName();
  std::vector<std::string> collNames;
  for (auto &kv : this->collisions) collNames.push_back(kv.first);

  std::string topic = contactManager->CreateFilter(this->filterName, collNames);
  this->contactSub = this->node->Subscribe(topic, &GazeboGraspFix::OnContact, this);

  //this->updateConnection = event::Events::ConnectWorldUpdateEnd(
  //    std::bind(&GazeboGraspFix::OnUpdate, this));
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboGraspFix::OnUpdate, this));
  if (this->updateConnection)
    std::cout << "[GazeboGraspFix] UpdateConnection registered" << std::endl;
  else
    std::cout << "[GazeboGraspFix] UpdateConnection FAILED" << std::endl;
}

static double AngularDistance(const ignition::math::Vector3d &v1,
                              const ignition::math::Vector3d &v2) {
  ignition::math::Vector3d a = v1; a.Normalize();
  ignition::math::Vector3d b = v2; b.Normalize();
  return acos(a.Dot(b));
}

static bool CheckGrip(const std::vector<ignition::math::Vector3d> &forces,
                      float minAngleDiff, float lengthRatio) {
  for (size_t i = 0; i < forces.size(); ++i) {
    for (size_t j = i + 1; j < forces.size(); ++j) {
      // std::cout << "[CheckGrip]---------------------------------" << std::endl;
      ignition::math::Vector3d v1 = forces[i];
      ignition::math::Vector3d v2 = forces[j];
      double l1 = v1.Length();
      double l2 = v2.Length();
      // std::cout << "[CheckGrip] lengths: " << l1 << ", " << l2 << std::endl;
      if (l1 < 1e-4 || l2 < 1e-4) continue;

      double angle = AngularDistance(v1, v2);
      // std::cout << "[CheckGrip] minAngleDiff=" << minAngleDiff << std::endl;
      // std::cout << "[CheckGrip] angle=" << angle << std::endl;
      // std::cout << "[CheckGrip] required length ratio: " << lengthRatio << std::endl;
      // std::cout << "[CheckGrip] length ratio : " << ((l1 > l2) ? l2 / l1 : l1 / l2) << std::endl;
      // std::cout << "[CheckGrip] v1=" << v1 << " v2=" << v2 
      //     << " angle=" << angle << std::endl;
      if (angle > minAngleDiff) {
        double ratio = (l1 > l2) ? l2 / l1 : l1 / l2;
        if (ratio >= lengthRatio) return true;
        return true;
      }
    }
  }
  return false;
}

void GazeboGraspFix::OnUpdate() {
  
  
  
  if ((common::Time::GetWallTime() - this->prevUpdateTime) < common::Time(0, 10000000))
    return;

  // std::cout << "#############################################33" << std::endl;
  // contacts 복사
  std::map<std::string, std::map<std::string, CollidingPoint>> contPoints;
  {
    std::lock_guard<boost::mutex> lock(this->mutexContacts);
    contPoints = this->contacts;
    this->contacts.clear();
  }

  // std::cout << "\n[DEBUG] contacts copied: " << contPoints.size() << " objects\n";
  // for (const auto &objPair : contPoints) {
  //     std::cout << "  Object: " << objPair.first 
  //               << " -> contact links: " << objPair.second.size() << std::endl;
  // }

  std::map<std::string, ObjectContactInfo> objectContactInfo;

  // 객체별 force 누적
  for (auto &objIt : contPoints) {
    const std::string &objName = objIt.first;
    ObjectContactInfo &objInfo = objectContactInfo[objName];

    for (auto &linkIt : objIt.second) {
      CollidingPoint &cp = linkIt.second;
      ignition::math::Vector3d avgForce = cp.force / cp.sum;
      objInfo.appliedForces.push_back(avgForce);
      objInfo.grippersInvolved[cp.gripperName]++;
      if (objInfo.grippersInvolved[cp.gripperName] > objInfo.maxGripperContactCnt) {
        objInfo.maxGripperContactCnt = objInfo.grippersInvolved[cp.gripperName];
        objInfo.maxContactGripper = cp.gripperName;
      }
    }
  }

  if (contPoints.size() == 0) {
    // std::cout << "[GazeboGraspFix] No contacts detected." << std::endl;
    if (this->contact_counts > 0) this->contact_counts--;
  }
  else {
    if (this->contact_counts < this->maxContactCount) this->contact_counts++;
  }
  // std::cout << "[GazeboGraspFix] Current contact counts: " << this->contact_counts << std::endl;

  // attach 처리
  for (auto &oc : objectContactInfo) {
    const std::string &objName = oc.first;
    const ObjectContactInfo &objInfo = oc.second;
    
    // std::cout << "Forces count: " << objInfo.appliedForces.size() << std::endl;
    // std::cout << "Angle tolerance: " << this->forcesAngleTolerance << std::endl;
    // std::cout << "Grip count: " << this->gripCounts[objName] << std::endl;
    
    int &counts = this->gripCounts[objName];
    if (CheckGrip(objInfo.appliedForces, this->forcesAngleTolerance, 0.1)) {
      if (counts < this->maxGripCount) counts++;
    } else {
      if (counts > 0) counts--;
    }

    if (counts <= this->gripCountThreshold) continue;

    auto gIt = this->grippers.find(objInfo.maxContactGripper);
    if (gIt == this->grippers.end()) continue;

    GazeboGraspGripper &gripper = gIt->second;
    if (!gripper.isObjectAttached()) {
      std::cout << "[GazeboGraspFix] Attaching " << objName
            << " to gripper " << objInfo.maxContactGripper << std::endl;
      gripper.HandleAttach(objName);
    }
  }

  // detach 처리
  // for (auto &gPair : this->grippers) {
  //   GazeboGraspGripper &gripper = gPair.second;
  //   if (gripper.isObjectAttached()) {
  //     const std::string &objName = gripper.attachedObject();
  //     if (this->gripCounts[objName] <= this->gripCountThreshold) {
  //       std::cout << "[GazeboGraspFix] Detaching " << objName
  //             << " from gripper " << gPair.first << std::endl;
  //       gripper.HandleDetach(objName);
  //       this->gripCounts[objName] = 0;
  //     }
  //   }
  // }

  // detach 처리
  for (auto &gPair : this->grippers) {
    if (this->contact_counts < this->contactCountThreshold){
      GazeboGraspGripper &gripper = gPair.second;
      const std::string &objName = gripper.attachedObject();
      // std::cout << "[GazeboGraspFix] Detaching " << objName
      //           << " from gripper " << gPair.first << std::endl;
      gripper.HandleDetach(objName);
    }
  }

  // for (auto &gPair : this->grippers) {
  //   GazeboGraspGripper &gripper = gPair.second;
  //   std::cout << "Gripper " << gPair.first << " attached: " << gripper.isObjectAttached() << std::endl;
  // }

  this->prevUpdateTime = common::Time::GetWallTime();
}

void GazeboGraspFix::OnContact(const ConstContactsPtr &_msg) {
  for (int i = 0; i < _msg->contact_size(); ++i) {
    std::string name1 = _msg->contact(i).collision1();
    std::string name2 = _msg->contact(i).collision2();

    physics::CollisionPtr coll1 =
        boost::dynamic_pointer_cast<physics::Collision>(
            this->world->EntityByName(name1));
    physics::CollisionPtr coll2 =
        boost::dynamic_pointer_cast<physics::Collision>(
            this->world->EntityByName(name2));

    if (!coll1 || !coll2) continue;

    std::string objName, collLink, gripperName;
    physics::CollisionPtr objColl, linkColl;

    if (this->collisions.find(name1) != this->collisions.end()) {
      objName = name2; collLink = name1;
      gripperName = this->collisions[name1];
      objColl = coll2; linkColl = coll1;
    } else if (this->collisions.find(name2) != this->collisions.end()) {
      objName = name1; collLink = name2;
      gripperName = this->collisions[name2];
      objColl = coll1; linkColl = coll2;
    } else continue;

    ignition::math::Vector3d avgForce;
    for (int k = 0; k < _msg->contact(i).wrench_size(); ++k) {
      auto wrenchMsg = _msg->contact(i).wrench(k);
      
      ignition::math::Vector3d f1(
          wrenchMsg.body_1_wrench().force().x(),
          wrenchMsg.body_1_wrench().force().y(),
          wrenchMsg.body_1_wrench().force().z());

      ignition::math::Vector3d f2(
          wrenchMsg.body_2_wrench().force().x(),
          wrenchMsg.body_2_wrench().force().y(),
          wrenchMsg.body_2_wrench().force().z());

      avgForce += (f1 + f2) * 0.5;  // 양쪽 평균
    }
    if (_msg->contact(i).wrench_size() > 0)
      avgForce /= _msg->contact(i).wrench_size();

    ignition::math::Vector3d avgPos;
    for (int k = 0; k < _msg->contact(i).position_size(); ++k) {
      avgPos += ignition::math::Vector3d(
          _msg->contact(i).position(k).x(),
          _msg->contact(i).position(k).y(),
          _msg->contact(i).position(k).z());
    }
    if (_msg->contact(i).position_size() > 0)
      avgPos /= _msg->contact(i).position_size();

    CollidingPoint cp;
    cp.gripperName = gripperName;
    cp.collLink = linkColl;
    cp.collObj = objColl;
    cp.force += avgForce;
    cp.pos += avgPos;
    cp.sum++;

    std::lock_guard<boost::mutex> lock(this->mutexContacts);
    this->contacts[objName][collLink] = cp;
  }
}
GZ_REGISTER_MODEL_PLUGIN(GazeboGraspFix)


