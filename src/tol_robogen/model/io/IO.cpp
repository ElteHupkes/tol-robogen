/*
 * Motor.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#include <tol_robogen/model/io/IO.h>
#include <sstream>

namespace sb = sdf_builder;

namespace tol_robogen {

IO::IO(std::string ioType, std::string partId, unsigned int ioId,
		std::string type, std::string ref):
	ioType_(ioType),
	partId_(partId),
	ioId_(ioId),
	type_(type),
	ref_(ref)
{}

IO::IO(const IO & other):
		partId_(other.partId_),
		ioId_(other.ioId_),
		type_(other.type_),
		ref_(other.ref_),
		ioType_(other.ioType_)
{}

IO * IO::clone() const {
	return new IO(*this);
}

IO::~IO() {}

std::string IO::attributes() {
	return "";
}

std::string IO::toXML() {
	std::stringstream out;
	out << "<tol:" << ioType_ << " "
		<< "type=\"" << type_ << "\" "
		<< "ref=\"" << ref_ << "\" "
		<< "part_id=\"" << partId_ << "\" "
		<< "io_id=\"" << ioId_ << "\" "
		<< this->attributes()
		<< " />"
		<< std::endl;
	return out.str();
}

} /* namespace tol_robogen */
