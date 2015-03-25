/**
 *
 */

#include <tol_robogen/model/io/ServoMotor.h>
#include <iostream>
#include <sstream>

namespace tol_robogen {

// Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100
const float ServoMotor::DEFAULT_MAX_FORCE_ROTATIONAL = (4 * 9.81) / 100;
const float ServoMotor::DEFAULT_MAX_FORCE_SERVO = (1.8 * 9.81) / 100;

// Default gain for the Servo's PID controller
// I believe this defines the derivative gain.
const float ServoMotor::DEFAULT_GAIN = 0.5;

const float ServoMotor::MIN_POS_RAD = -(45 * M_PI / 180);
const float ServoMotor::MAX_POS_RAD = (45 * M_PI / 180);

// 50 rpm converted to rad/s
// 	note, max velocity should really be 100 rpm, but only with 0 torque
// 	50 rpms is a compromise
const float ServoMotor::MIN_VELOCITY = -(50.0/60.0) * 2 * M_PI;
const float ServoMotor::MAX_VELOCITY = (50.0/60.0) * 2 * M_PI;

ServoMotor::ServoMotor(std::string partId, unsigned int ioId,
		sdf_builder::JointPtr joint, double maxForce,
		bool velocityDriven, double gain, double noise):
	IO("motor", partId, ioId, "servo", joint->name()),
	gain(gain),
	noise(noise),
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

ServoMotor::ServoMotor(const ServoMotor & other):
	IO(other),
	velocityDriven(other.velocityDriven),
	gain(other.gain),
	noise(other.noise)
{}

ServoMotor* ServoMotor::clone() const {
	return new ServoMotor(*this);
}

std::string ServoMotor::attributes() {
	std::stringstream attrs;
	attrs << "velocityDriven=\"" << velocityDriven << "\" "
		  << "gain=\"" << gain << "\" "
		  << "noise=\"" << noise << "\" "
		  // Rotational velocity we need not scale
		  << "minVelocity=\"" << ServoMotor::MIN_VELOCITY << "\" "
		  << "maxVelocity=\"" << ServoMotor::MAX_VELOCITY << "\" ";

	return attrs.str();
}

}
