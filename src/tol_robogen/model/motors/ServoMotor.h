/*
 * ServoMotor.h
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_MODEL_MOTORS_SERVOMOTOR_H_
#define TOL_ROBOGEN_MODEL_MOTORS_SERVOMOTOR_H_

#include <tol_robogen/model/motors/Motor.h>

namespace sb = sdf_builder;

namespace tol_robogen {

/**
 * Thin wrapper over motor that sets a servo type
 */
class ServoMotor: public Motor {
public:
	ServoMotor(sb::JointPtr joint):
		Motor("servo", joint) {};

	ServoMotor(const ServoMotor & other):
		Motor(other) {};

	virtual ServoMotor * clone() const {
		return new ServoMotor(*this);
	}

	virtual ~ServoMotor() {};

};

} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_MODEL_MOTORS_SERVOMOTOR_H_ */
