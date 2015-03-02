/*
 * Robot.cpp
 *
 *  Created on: Feb 26, 2015
 *      Author: elte
 */

#include <stdexcept>

#include <tol_robogen/tol.h>
#include <tol_robogen/model/Robot.h>
#include <tol_robogen/model/Connection.h>

#include <sdf_builder/Model.h>

#include <sstream>

namespace sb = sdf_builder;

namespace tol_robogen {

Robot::Robot() {}

Robot::Robot(PartRepresentationPtr core) {
	init(core);
}

void Robot::init(PartRepresentationPtr core) {
	if (coreComponent_) {
		throw std::runtime_error("This robot has already been initialized "
				"It is currently not possible to reinitialize a robot,"
				"please create a new one instead.");
	}

	coreComponent_ = core->addSubtreeToRobot(this);

	// Now start putting the body parts in position

}

Robot::~Robot() {}

const std::vector< ModelPtr >& Robot::getBodyParts() const {
	return bodyParts_;
}

const std::vector< ConnectionPtr >& Robot::getBodyConnections() const {
	return bodyConnections_;
}

void Robot::addBodyPart(ModelPtr bodyPart) {
	bodyParts_.push_back(bodyPart);
}

void Robot::addBodyConnection(ModelPtr from, ModelPtr to, unsigned int fromSlot, unsigned int toSlot) {
	// Store the connection so we at least may have access to the geometry later
	bodyConnections_.push_back(ConnectionPtr(new Connection(from, to, fromSlot, toSlot)));


	// I don't see the need to setup a breadth first search to create the robot like
	// the original robogen does, when we can just setup the body position here
	// immediately. Model handles this
	// TODO create fixed joint
	from->attachTo(to, fromSlot, toSlot);
}

sb::ModelPtr Robot::toSDFModel(const std::string & name) {
	sb::ModelPtr out(new sb::Model(name));

	std::vector< ModelPtr >::iterator it = bodyParts_.begin();
	for (; it != bodyParts_.end(); ++it) {
		ModelPtr bodyPart = *it;
		out->addPosable(bodyPart->getPosableGroup());
	}

	return out;
}

} /* namespace tol_robogen */
