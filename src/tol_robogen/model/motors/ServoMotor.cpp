/**
 *
 */

#include <tol_robogen/model/motors/ServoMotor.h>

#include <iostream>

namespace tol_robogen {

// Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100
const float ServoMotor::DEFAULT_MAX_FORCE_ROTATIONAL = TOL_SCALING * 4 * 9.81 / 100;
const float ServoMotor::DEFAULT_MAX_FORCE_SERVO = TOL_SCALING * 1.8 * 9.81 / 100;

ServoMotor::ServoMotor(std::string partId, unsigned int ioId,
		sdf_builder::JointPtr joint, double maxForce):
	Motor(partId, ioId, "servo", joint)
{
	// TODO Does this do what I intend it to?
	if (!joint->axis->limit) {
		joint->axis->limit.reset(new sdf_builder::Limit());
	}

	joint->axis->limit->effort = maxForce;
}

ServoMotor* ServoMotor::clone() const {
	return new ServoMotor(*this);
}

}
