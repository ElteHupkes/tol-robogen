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

// TODO Do something with the "gain" parameter that's in Robogen;
// probably add it to the joint controller.

ServoMotor::ServoMotor(gz::physics::ModelPtr model, gz::physics::JointPtr joint,
		std::string partId, unsigned int ioId):
	Motor(model, joint, partId, ioId)
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
