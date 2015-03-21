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
// probably add it to the joint PID controller
const float ServoMotor::DEFAULT_GAIN = 0.5;

// 50 rpm converted to rad/s
// 	note, max velocity should really be 100 rpm, but only with 0 torque
// 	50 rpms is a compromise
const float ServoMotor::MIN_VELOCITY = -(50.0/60.0) * 2 * M_PI;
const float ServoMotor::MAX_VELOCITY = (50.0/60.0) * 2 * M_PI;

ServoMotor::ServoMotor(gz::physics::ModelPtr model, gz::physics::JointPtr joint,
		std::string partId, unsigned int ioId, bool velocityDriven, double gain):
	Motor(model, joint, partId, ioId),
	velocityDriven_(velocityDriven),
	gain_(gain),
	lowerLimit_(joint->GetLowerLimit(0).Radian()),
	upperLimit_(joint->GetUpperLimit(0).Radian()),
	jointController_(model->GetJointController()),
	jointName_(joint->GetScopedName())
{
	// Create a PID with the correct parameters
	auto pid = gz::common::PID(
		// Proportional gain
		0.0,

		// Integral gain
		0.0,

		// Derivative gain
		gain * 5

		// Max/min integral values
		//0.0,
		//0.0,

		// Max / min force; these are already determined
		// by the joint's effort limit.
		//0.0,
		//0.0
	);

	if (velocityDriven) {
		jointController_->SetVelocityPID(jointName_, pid);
	} else {
		jointController_->SetPositionPID(jointName_, pid);
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
