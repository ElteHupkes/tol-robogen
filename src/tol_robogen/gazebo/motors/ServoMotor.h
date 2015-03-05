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
	ServoMotor(::gazebo::physics::ModelPtr model, ::gazebo::physics::JointPtr joint);
	virtual ~ServoMotor();

	virtual void update(float networkOutput);

	bool set = false;
};

} /* namespace gazebo */
} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_GAZEBO_MOTORS_SERVOMOTOR_H_ */
