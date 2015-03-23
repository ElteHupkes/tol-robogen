/*
 * ServoMotor.h
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_MOTORS_SERVOMOTOR_H_
#define TOL_ROBOGEN_GAZEBO_MOTORS_SERVOMOTOR_H_

#include <tol_robogen/gazebo/motors/Motor.h>

#include <gazebo/common/common.hh>

namespace tol_robogen {
namespace gazebo {

class ServoMotor: public Motor {
public:
	/**
	 * @param The model the motor is contained in
	 * @param The joint driven by the motor
	 * @param The part ID the motor belongs to
	 * @param The I/O ID on the part
	 * @param Whether the motor is velocity driven (the alternative is position driven)
	 * @param The derivative gain of the motor's PID controller
	 */
	ServoMotor(::gazebo::physics::ModelPtr model, ::gazebo::physics::JointPtr joint,
			std::string partId, unsigned int ioId, sdf::ElementPtr motor);
	virtual ~ServoMotor();

	virtual void update(float networkOutput, unsigned int step);

protected:
	bool velocityDriven_;

	// Uper and lower position limits
	double lowerLimit_;
	double upperLimit_;

	// Velocity limits
	double minVelocity_;
	double maxVelocity_;

	/**
	 * The joint controller of the attaching model
	 */
	::gazebo::physics::JointControllerPtr jointController_;

	/**
	 * Store string joint name for repeated use
	 */
	std::string jointName_;
};

} /* namespace gazebo */
} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_GAZEBO_MOTORS_SERVOMOTOR_H_ */
