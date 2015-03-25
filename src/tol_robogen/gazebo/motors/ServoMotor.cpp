/*
 * ServoMotor.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/motors/ServoMotor.h>

#include <gazebo/math/Rand.hh>

#include <iostream>

namespace gz = gazebo;

namespace tol_robogen {
namespace gazebo {

ServoMotor::ServoMotor(gz::physics::ModelPtr model, gz::physics::JointPtr joint,
		std::string partId, unsigned int ioId, sdf::ElementPtr motor):
	Motor(model, joint, partId, ioId),
	lowerLimit_(joint->GetLowerLimit(0).Radian()),
	upperLimit_(joint->GetUpperLimit(0).Radian()),
	jointController_(model->GetJointController()),
	jointName_(joint->GetScopedName()),
	minVelocity_(0),
	maxVelocity_(0),
	noise_(0),
	velocityDriven_(false)
{
	auto veloParam = motor->GetAttribute("velocityDriven");
	if (veloParam) {
		veloParam->Get(velocityDriven_);
	}

	auto gainParam = motor->GetAttribute("gain");
	if (gainParam) {
		// Override PID if gain is given
		double gain;
		gainParam->Get(gain);

		// Create a PID with the correct parameters
		auto pid = gz::common::PID(
			// Proportional gain
			gain,

			// Integral gain
			0.0,

			// Derivative gain
			0.0
		);

		if (velocityDriven_) {
			jointController_->SetVelocityPID(jointName_, pid);
		} else {
			jointController_->SetPositionPID(jointName_, pid);
		}
	}

	auto noiseParam = motor->GetAttribute("noise");
	if (noiseParam) {
		noiseParam->Get(noise_);
	}

	auto minVParam = motor->GetAttribute("minVelocity");
	auto maxVParam = motor->GetAttribute("maxVelocity");

	if (!minVParam || !maxVParam) {
		std::cerr << "Missing servo min/max velocity parameters, "
				"velocity will be zero." << std::endl;
	} else {
		minVParam->Get(minVelocity_);
		maxVParam->Get(maxVelocity_);
	}
}

ServoMotor::~ServoMotor() {}

void ServoMotor::update(float networkOutput, unsigned int /*step*/) {
	// Motor noise in range +/- noiseLevel * actualValue
	networkOutput += ((2 * gz::math::Rand::GetDblUniform() * noise_) -
				noise_) * networkOutput;

	if (velocityDriven_) {
		double velocity = minVelocity_ + networkOutput * (maxVelocity_ - minVelocity_);
		jointController_->SetVelocityTarget(jointName_, velocity);
	} else {
		double position = lowerLimit_ + networkOutput * (upperLimit_ - lowerLimit_);
		jointController_->SetPositionTarget(jointName_, position);
	}
}

} /* namespace gazebo */
} /* namespace tol_robogen */
