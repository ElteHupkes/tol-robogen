/*
 * Motor.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#include <tol_robogen/model/motors/Motor.h>

#include <sstream>

namespace sb = sdf_builder;

namespace tol_robogen {

Motor::Motor(std::string type, sb::JointPtr joint):
	type_(type),
	joint_(joint)
{}

Motor::Motor(const Motor & other):
		type_(other.type_),
		joint_(other.joint_)
{}

Motor * Motor::clone() const {
	return new Motor(*this);
}

Motor::~Motor() {}

std::string Motor::toXML() {
	std::stringstream out;
	out << "<tol:motor type=\"" <<
			type_ << "\" joint=\""
			<< joint_->name() << "\" />"
		<< std::endl;
	return out.str();
}

} /* namespace tol_robogen */
