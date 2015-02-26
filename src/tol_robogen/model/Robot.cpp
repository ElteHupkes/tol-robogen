/*
 * Robot.cpp
 *
 *  Created on: Feb 26, 2015
 *      Author: elte
 */

#include <tol_robogen/model/Robot.h>

namespace tol_robogen {

Robot::Robot() {}

Robot::~Robot() {}

const std::vector< ModelPtr >& Robot::getBodyParts() {
	return bodyParts_;
}

const std::vector< ConnectionPtr >& Robot::getBodyConnections() {
	return bodyConnections_;
}

void Robot::addBodyPart(ModelPtr bodyPart) {
	bodyParts_.push_back(bodyPart);
}

void Robot::addBodyConnection(ConnectionPtr connection) {
	bodyConnections_.push_back(connection);
}

} /* namespace tol_robogen */
