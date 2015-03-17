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
	static const float DEFAULT_MAX_FORCE_ROTATIONAL;
	static const float DEFAULT_MAX_FORCE_SERVO;

	ServoMotor(std::string partId, unsigned int ioId,
			sdf_builder::JointPtr joint, double maxForce);
	ServoMotor(const ServoMotor & other):
		Motor(other) {};

	virtual ServoMotor * clone() const;

	virtual ~ServoMotor() {};

};

} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_MODEL_MOTORS_SERVOMOTOR_H_ */
