/*
 * Motor.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/motors/Motor.h>

namespace gz = gazebo;

namespace tol_robogen {
namespace gazebo {

Motor::Motor(::gazebo::physics::ModelPtr model, ::gazebo::physics::JointPtr joint,
		std::string partId, unsigned int ioId):
	model_(model),
	joint_(joint),
	ioId_(ioId),
	partId_(partId)
{}

Motor::~Motor() {}

std::string Motor::partId() {
	return partId_;
}

unsigned int Motor::ioId() {
	return ioId_;
}

} /* namespace gazebo */
} /* namespace tol_robogen */
