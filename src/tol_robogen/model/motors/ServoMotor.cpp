/**
 *
 */

#include <tol_robogen/model/motors/ServoMotor.h>

#include <iostream>
#include <sstream>

namespace tol_robogen {

// Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100
const float ServoMotor::DEFAULT_MAX_FORCE_ROTATIONAL = 4 * 9.81 / 100;
const float ServoMotor::DEFAULT_MAX_FORCE_SERVO = 1.8 * 9.81 / 100;

const float ServoMotor::MIN_POS_RAD = -(45 * M_PI / 180);
const float ServoMotor::MAX_POS_RAD = (45 * M_PI / 180);

ServoMotor::ServoMotor(std::string partId, unsigned int ioId,
		sdf_builder::JointPtr joint, double maxForce, bool velocityDriven):
	Motor(partId, ioId, "servo", joint),
	velocityDriven(velocityDriven)
{
	// TODO Does this do what I intend it to?
	if (!joint->axis->limit) {
		joint->axis->limit.reset(new sdf_builder::Limit(0, 0));
	}

	joint->axis->limit->continuous = false;
	joint->axis->limit->lower = MIN_POS_RAD;
	joint->axis->limit->upper = MAX_POS_RAD;

	joint->axis->limit->effort = maxForce;
}

ServoMotor* ServoMotor::clone() const {
	return new ServoMotor(*this);
}

std::string ServoMotor::toXML() {
	std::stringstream attrs;
	attrs << "velocityDriven=\"" << velocityDriven << "\" "
		  << attributes_;

	attributes_ = attrs.str();
	return Motor::toXML();
}

}
