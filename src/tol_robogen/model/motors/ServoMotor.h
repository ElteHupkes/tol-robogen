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
	// Motor constants
	static const float DEFAULT_MAX_FORCE_ROTATIONAL;
	static const float DEFAULT_MAX_FORCE_SERVO;
	static const float DEFAULT_GAIN;
	static const float MIN_POS_RAD;
	static const float MAX_POS_RAD;
	static const float MIN_VELOCITY;
	static const float MAX_VELOCITY;

	ServoMotor(std::string partId, unsigned int ioId,
			sdf_builder::JointPtr joint, double maxForce,
			bool velocityDriven, double gain);
	ServoMotor(const ServoMotor & other);

	virtual ServoMotor * clone() const;

	virtual ~ServoMotor() {};

	virtual std::string toXML();

	bool velocityDriven;
	double gain;
};

} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_MODEL_MOTORS_SERVOMOTOR_H_ */
