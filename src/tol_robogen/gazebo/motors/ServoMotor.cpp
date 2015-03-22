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

ServoMotor::ServoMotor(gz::physics::ModelPtr model, gz::physics::JointPtr joint,
		std::string partId, unsigned int ioId, sdf::ElementPtr motor):
	Motor(model, joint, partId, ioId),
	lowerLimit_(joint->GetLowerLimit(0).Radian()),
	upperLimit_(joint->GetUpperLimit(0).Radian()),
	jointController_(model->GetJointController()),
	jointName_(joint->GetScopedName())
{
	auto veloParam = motor->GetAttribute("velocityDriven");
	if (veloParam) {
		veloParam->Get(velocityDriven_);
	} else {
		velocityDriven_ = false;
	}

	auto gainParam = motor->GetAttribute("gain");
	if (gainParam) {
		// Override PID if gain is given
		double gain;
		gainParam->Get(gain);

		// Create a PID with the correct parameters
		auto pid = gz::common::PID(
			// Proportional gain
			0.0,

			// Integral gain
			0.0,

			// Derivative gain
			gain
		);

		if (velocityDriven_) {
			jointController_->SetVelocityPID(jointName_, pid);
		} else {
			jointController_->SetPositionPID(jointName_, pid);
		}
	}
}

ServoMotor::~ServoMotor() {}

void ServoMotor::update(float networkOutput, unsigned int /*step*/) {
	// TODO Add motor noise
	if (velocityDriven_) {
		double velocity = MIN_VELOCITY + networkOutput * (MAX_VELOCITY - MIN_VELOCITY);
		jointController_->SetVelocityTarget(jointName_, velocity);
	} else {
		double position = lowerLimit_ + networkOutput * (upperLimit_ - lowerLimit_);
		jointController_->SetPositionTarget(jointName_, position);
	}
}

} /* namespace gazebo */
} /* namespace tol_robogen */
