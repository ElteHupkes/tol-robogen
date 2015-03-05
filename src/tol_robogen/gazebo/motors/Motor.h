/*
 * Motor.h
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_MOTORS_MOTOR_H_
#define TOL_ROBOGEN_GAZEBO_MOTORS_MOTOR_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace tol_robogen {
namespace gazebo {

class Motor {
public:
	Motor(::gazebo::physics::ModelPtr model, ::gazebo::physics::JointPtr joint);
	virtual ~Motor();

	/**
	 * Updates the motor based on the attached output
	 * of the neural network.
	 */
	virtual void update(float networkOutput) = 0;
protected:
	/**
	 * The model this motor is part of
	 */
	::gazebo::physics::ModelPtr model_;

	/**
	 * The joint this motor is controlling
	 */
	::gazebo::physics::JointPtr joint_;
};

} /* namespace gazebo */
} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_GAZEBO_MOTORS_MOTOR_H_ */
