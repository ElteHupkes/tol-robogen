/*
 * ServoMotor.h
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_MOTORS_SERVOMOTOR_H_
#define TOL_ROBOGEN_GAZEBO_MOTORS_SERVOMOTOR_H_

#include <tol_robogen/gazebo/motors/Motor.h>

namespace tol_robogen {
namespace gazebo {

class ServoMotor: public Motor {
public:
	// Physical constants
	static const float MIN_VELOCITY;
	static const float MAX_VELOCITY;
	static const float DEFAULT_GAIN;

	// Constructor without gain - velocity driven
	ServoMotor(::gazebo::physics::ModelPtr model, ::gazebo::physics::JointPtr joint,
			std::string partId, unsigned int ioId);

	// Constructor with gain - position driven
	ServoMotor(::gazebo::physics::ModelPtr model, ::gazebo::physics::JointPtr joint,
			std::string partId, unsigned int ioId, double gain);
	virtual ~ServoMotor();

	virtual void update(float networkOutput);

protected:
	bool velocityDriven_;

	// Uper and lower position limits
	double lowerLimit_;
	double upperLimit_;
	double gain_;
};

} /* namespace gazebo */
} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_GAZEBO_MOTORS_SERVOMOTOR_H_ */
