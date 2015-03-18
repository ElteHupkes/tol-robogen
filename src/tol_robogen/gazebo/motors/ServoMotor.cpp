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
		std::string partId, unsigned int ioId):
	Motor(model, joint, partId, ioId),
	velocityDriven_(true),
	gain_(0),
	lowerLimit_(joint->GetLowerLimit(0).Radian()),
	upperLimit_(joint->GetUpperLimit(0).Radian())
{}

ServoMotor::ServoMotor(gz::physics::ModelPtr model, gz::physics::JointPtr joint,
		std::string partId, unsigned int ioId, double gain):
	Motor(model, joint, partId, ioId),
	velocityDriven_(false),
	gain_(gain),
	lowerLimit_(joint->GetLowerLimit(0).Radian()),
	upperLimit_(joint->GetUpperLimit(0).Radian())
{}

ServoMotor::~ServoMotor() {}

void ServoMotor::update(float networkOutput) {
	std::cout << "Before: " << joint_->GetVelocity(0) << std::endl;

	// TODO Add motor noise
	if (velocityDriven_) {
		double velocity = MIN_VELOCITY + networkOutput * (MAX_VELOCITY - MIN_VELOCITY);
		joint_->SetVelocity(0, velocity);
	} else {
//		double position = lowerLimit_ + networkOutput * (upperLimit_ - lowerLimit_);
//		joint_->SetVelocity(0, MIN_VELOCITY);
//		auto ctrl = model_->GetJointController();
//		ctrl->SetVelocityTarget(joint_->GetScopedName(), MAX_VELOCITY);

//		std::cout << "After: " << joint_->GetVelocity(0) << std::endl;
		auto ctrl = model_->GetJointController();
		std::cout << ctrl->SetPositionTarget(joint_->GetScopedName(), networkOutput) << std::endl;

		// TODO set position target instead, can probably use joint PID controller
		//		auto ctrl = model_->GetJointController();
		//		std::cout << ctrl->SetVelocityTarget(joint_->GetScopedName(), networkOutput) << std::endl;
	}
}

} /* namespace gazebo */
} /* namespace tol_robogen */
