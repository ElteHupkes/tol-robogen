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
#include <tol_robogen/model/ActuatedComponent.h>
#include <tol_robogen/model/motors/Motor.h>
#include <sdf_builder/Model.h>

#include <sstream>
#include <iostream>

namespace sb = sdf_builder;

namespace tol_robogen {

Robot::Robot() {}

Robot::Robot(PartRepresentationPtr core, NeuralNetworkRepresentationPtr brain) {
	init(core, brain);
}

void Robot::init(PartRepresentationPtr core, NeuralNetworkRepresentationPtr brain) {
	if (coreComponent_) {
		throw std::runtime_error("This robot has already been initialized "
				"It is currently not possible to reinitialize a robot,"
				"please create a new one instead.");
	}

	coreComponent_ = core->addSubtreeToRobot(this);

	// TODO Is this sufficient?
	brainXML_ = brain->toXML();
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

void Robot::addBodyConnection(ModelPtr from, ModelPtr to, unsigned int fromSlot, unsigned int toSlot, unsigned int orientation) {
	// Store the connection so we at least may have access to the geometry later
	bodyConnections_.push_back(ConnectionPtr(new Connection(from, to, fromSlot, toSlot)));

	// I don't see the need to setup a graph with breadth first search to create
	// the robot like the original robogen does, we can just setup the body position here
	// immediately. Model->attach handles this.
	to->attach(from, fromSlot, toSlot, orientation);
}

sb::ModelPtr Robot::toSDFModel(const std::string & name) {
	sb::ModelPtr out(new sb::Model(name));

	std::stringstream plugin;

	// Attach model controller plugin with parameters
	// TODO sensible namespace
	plugin << "<plugin name=\"control\" filename=\"libtolmodelcontrol.so\">";
	plugin << "<tol:settings xmlns:tol=\"http://elte.me\">";
	for (auto it = bodyParts_.begin(); it != bodyParts_.end(); ++it) {
		ModelPtr bodyPart = *it;
		out->addPosable(bodyPart->getPosableGroup());

		auto joints = bodyPart->joints();
		for (auto itb = joints.begin(); itb != joints.end(); ++itb) {
			out->addJoint(*itb);
		}

		ActuatedComponentPtr actuated = std::dynamic_pointer_cast< ActuatedComponent >(bodyPart);
		if (actuated) {
			// Add motor elements
			auto motors = actuated->getMotors();
			for (auto itb = motors.begin(); itb != motors.end(); ++itb) {
				plugin << (*itb)->toXML();
			}
		}

		// TODO Add sensors
	}

	plugin << brainXML_;
	plugin << "</tol:settings>";
	plugin << "</plugin>\n";
	sb::ElementPtr pluginElem(new sb::StringElement(plugin.str()));
	out->addElement(pluginElem);

	return out;
}

} /* namespace tol_robogen */
