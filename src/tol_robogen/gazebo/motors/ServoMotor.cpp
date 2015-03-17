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
		std::string partId, unsigned int ioId, bool velocityDriven):
	Motor(model, joint, partId, ioId),
	velocityDriven_(velocityDriven)
{}

ServoMotor::~ServoMotor() {}

void ServoMotor::update(float networkOutput) {
	std::cout << "Network output: " << networkOutput << std::endl;
//		std::cout << "Firing first motor." << std::endl;
//		auto ctrl = model_->GetJointController();
//		std::cout << ctrl->SetVelocityTarget(joint_->GetScopedName(), networkOutput) << std::endl;
}

} /* namespace gazebo */
} /* namespace tol_robogen */
