#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class MimicJointPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      std::cout << "[MimicJointPlugin] Loading plugin..." << std::endl;
      if (!_sdf->HasElement("joint1") || !_sdf->HasElement("joint2"))
      {
        gzerr << "[MimicJointPlugin] Please specify <joint1> and <joint2> in plugin SDF.\n";
        return;
      }

      std::string joint1Name = _sdf->Get<std::string>("joint1");
      std::string joint2Name = _sdf->Get<std::string>("joint2");
      this->multiplier = _sdf->Get<double>("multiplier", 1.0).first;
      this->offset = _sdf->Get<double>("offset", 0.0).first;

      this->joint1 = _model->GetJoint(joint1Name);
      this->joint2 = _model->GetJoint(joint2Name);

      if (!this->joint1 || !this->joint2)
      {
        gzerr << "[MimicJointPlugin] Could not find specified joints.\n";
        return;
      }

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MimicJointPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      if (this->joint1 && this->joint2)
      {
        double target = this->multiplier * this->joint1->Position(0) + this->offset;
        this->joint2->SetPosition(0, target);
      }
    }

  private:
    physics::JointPtr joint1, joint2;
    double multiplier{1.0}, offset{0.0};
    event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin)
}

