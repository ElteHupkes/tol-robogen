/*
 * ServoMotor.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/motors/ServoMotor.h>

namespace gz = gazebo;

namespace tol_robogen {
namespace gazebo {

ServoMotor::ServoMotor(gz::physics::ModelPtr model, gz::physics::JointPtr joint):
	Motor(model, joint)
{}

ServoMotor::~ServoMotor() {}

void ServoMotor::update(float networkOutput) {
	if (!set) {
		std::cout << "Firing first motor." << std::endl;
		auto ctrl = model_->GetJointController();
		std::cout << ctrl->SetVelocityTarget(joint_->GetScopedName(), networkOutput) << std::endl;
		set = true;
	}
}

} /* namespace gazebo */
} /* namespace tol_robogen */
