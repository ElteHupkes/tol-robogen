/*
 * Motor.h
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_MODEL_MOTORS_MOTOR_H_
#define TOL_ROBOGEN_MODEL_MOTORS_MOTOR_H_

#include <tol_robogen/tol.h>
#include <sdf_builder/Types.h>
#include <sdf_builder/joint/Joint.h>

namespace tol_robogen {

/**
 * Simple class to represent a motor in the model, we'll pass
 * it in the SDF to the controller classes.
 */
class Motor : public sdf_builder::Element {
public:
	Motor(std::string partId, unsigned int ioId, std::string type, sdf_builder::JointPtr joint);
	Motor(const Motor & other);
	virtual ~Motor();

	virtual Motor * clone() const;

	/**
	 * Returns the XML representation of this motor
	 */
	virtual std::string toXML();

protected:
	/**
	 * ID of the parent part
	 */
	std::string partId_;

	/**
	 * Type of the motor
	 */
	std::string type_;

	/**
	 * I/O ID of the motor on the main part
	 */
	unsigned int ioId_;

	/**
	 * The joint representing this motor.
	 */
	sdf_builder::JointPtr joint_;
};

} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_MODEL_MOTORS_MOTOR_H_ */
