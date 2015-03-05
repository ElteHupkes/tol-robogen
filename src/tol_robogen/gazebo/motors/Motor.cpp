/*
 * Motor.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/motors/Motor.h>

namespace tol_robogen {
namespace gazebo {

Motor::Motor(::gazebo::physics::ModelPtr model, ::gazebo::physics::JointPtr joint):
	model_(model),
	joint_(joint)
{}

Motor::~Motor() {}

} /* namespace gazebo */
} /* namespace tol_robogen */
