/*
 * Robot.cpp
 *
 *  Created on: Feb 26, 2015
 *      Author: elte
 */

#include <stdexcept>

#include <tol_robogen/tol.h>
#include <tol_robogen/model/Robot.h>

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

void Robot::addBodyConnection(ConnectionPtr connection) {
	bodyConnections_.push_back(connection);
}

} /* namespace tol_robogen */
