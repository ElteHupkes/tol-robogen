/*
 * Motor.h
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_MOTORS_MOTOR_H_
#define TOL_ROBOGEN_GAZEBO_MOTORS_MOTOR_H_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <tol_robogen/gazebo/Types.h>

namespace tol_robogen {
namespace gazebo {

class Motor {
public:
	Motor(::gazebo::physics::ModelPtr model, ::gazebo::physics::JointPtr joint,
			std::string partId, unsigned int ioId);
	virtual ~Motor();

	/**
	 * Updates the motor based on the attached output
	 * of the neural network.
	 *
	 * @param Output of the neural network
	 * @param Actuation time in nanoseconds
	 */
	virtual void update(float networkOutput, unsigned int step) = 0;

	/**
	 * @return The part ID
	 */
	std::string partId();

	/**
	 * @return I/O ID
	 */
	unsigned int ioId();

protected:
	/**
	 * The model this motor is part of
	 */
	::gazebo::physics::ModelPtr model_;

	/**
	 * The joint this motor is controlling
	 */
	::gazebo::physics::JointPtr joint_;

	/**
	 * ID of the body part the motor belongs to
	 */
	std::string partId_;

	/**
	 * I/O ID on the body part
	 */
	unsigned int ioId_;
};

} /* namespace gazebo */
} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_GAZEBO_MOTORS_MOTOR_H_ */
