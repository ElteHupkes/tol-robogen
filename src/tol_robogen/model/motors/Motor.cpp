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

Motor::Motor(std::string partId, unsigned int ioId, std::string type, sdf_builder::JointPtr joint):
	partId_(partId),
	ioId_(ioId),
	type_(type),
	joint_(joint)
{}

Motor::Motor(const Motor & other):
		partId_(other.partId_),
		ioId_(other.ioId_),
		type_(other.type_),
		joint_(other.joint_)
{}

Motor * Motor::clone() const {
	return new Motor(*this);
}

Motor::~Motor() {}

std::string Motor::toXML() {
	std::stringstream out;
	out << "<tol:motor "
		<< "type=\"" << type_ << "\" "
		<< "joint=\"" << joint_->name() << "\" "
		<< "part_id=\"" << partId_ << "\" "
		<< "io_id=\"" << ioId_ << "\" "
		<< " />"
		<< std::endl;
	return out.str();
}

} /* namespace tol_robogen */
