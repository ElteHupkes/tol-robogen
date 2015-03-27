/*
 * Robot.cpp
 *
 *  Created on: Feb 26, 2015
 *      Author: elte
 */

#include <stdexcept>

#include <tol_robogen/model/Robot.h>
#include <tol_robogen/model/Connection.h>
#include <sdf_builder/Model.h>
#include <sdf_builder/collision/Collision.h>
#include <tol_robogen/model/Component.h>
#include <tol_robogen/model/io/IO.h>
#include <sdf_builder/collision/Surface.h>
#include <sdf_builder/collision/Friction.h>

#include <sstream>
#include <iostream>

namespace sb = sdf_builder;

namespace tol_robogen {

Robot::Robot() {}

Robot::Robot(PartRepresentationPtr core, NeuralNetworkRepresentationPtr brain, ConfigurationPtr config) {
	init(core, brain, config);
}

void Robot::init(PartRepresentationPtr core, NeuralNetworkRepresentationPtr brain, ConfigurationPtr config) {
	if (coreComponent_) {
		throw std::runtime_error("This robot has already been initialized "
				"It is currently not possible to reinitialize a robot,"
				"please create a new one instead.");
	}

	coreComponent_ = core->addSubtreeToRobot(this, config);

	// Store brain XML directly without intermediate representation
	brainXML_ = brain->toXML();
}

Robot::~Robot() {}

const std::vector< ComponentPtr >& Robot::getBodyParts() const {
	return bodyParts_;
}

const std::vector< ConnectionPtr >& Robot::getBodyConnections() const {
	return bodyConnections_;
}

void Robot::addBodyPart(ComponentPtr bodyPart) {
	bodyParts_.push_back(bodyPart);
}

void Robot::addBodyConnection(ComponentPtr from, ComponentPtr to, unsigned int fromSlot, unsigned int toSlot, unsigned int orientation) {
	// Store the connection so we at least may have access to the geometry later
	bodyConnections_.push_back(ConnectionPtr(new Connection(from, to, fromSlot, toSlot)));

	// I don't see the need to setup a graph with breadth first search to create
	// the robot like the original robogen does, we can just setup the body position here
	// immediately. Model->attach handles this.
	to->attach(from, fromSlot, toSlot, orientation);
}

// Simple internal helper function
void addSurface(sb::PosableGroupPtr p) {
	auto posables = p->posables();

	for (auto it = posables.begin(); it != posables.end(); ++it) {
		if (!std::dynamic_pointer_cast< sb::Link >(*it)) {
			continue;
		}

		auto link = std::dynamic_pointer_cast< sb::Link >(*it);
		auto lp = link->posables();

		for (auto itb = lp.begin(); itb != lp.end(); ++itb) {
			if (!std::dynamic_pointer_cast< sb::Collision >(*itb)) {
				continue;
			}

			auto col = std::dynamic_pointer_cast< sb::Collision >(*itb);
			if (col->surface) {
				continue;
			}

			// Add our default surface settings
			// These parameters were taken from RobogenCollision.cpp
			// TODO Find out if these parameters make sense and what
			// they change about the simulation.
			col->surface.reset(new sb::Surface());
			auto surf = col->surface;
			surf->friction.reset(new sb::Friction());
			surf->friction->friction1 = 1.0;
			surf->friction->friction2 = 1.0;
			surf->friction->slip1 = 0.1;
			surf->friction->slip2 = 0.1;

			sb::StringElementPtr contact(new sb::StringElement("<contact>"
					"<ode>"
						"<soft_cfm>0.01</soft_cfm>"
						"<soft_erp>0.96</soft_erp>"
					"</ode>"
					"<bullet>"
						"<soft_cfm>0.01</soft_cfm>"
						"<soft_erp>0.96</soft_erp>"
					"</bullet>"
					"</contact>"));
			surf->addElement(contact);
		}
	}
}

sb::ModelPtr Robot::toSDFModel(const std::string & name) {
	sb::ModelPtr out(new sb::Model(name));

	std::stringstream plugin;

	// Attach model controller plugin with parameters
	plugin << "<plugin name=\"control\" filename=\"libtolmodelcontrol.so\">";
	plugin << "<tol:robot_config xmlns:tol=\"https://github.com/ElteHupkes/tol-robogen\">";

	for (auto it = bodyParts_.begin(); it != bodyParts_.end(); ++it) {
		ComponentPtr bodyPart = *it;
		auto group = bodyPart->getPosableGroup();

		// Add collision surface parameters
		addSurface(group);

		// Add body parts
		out->addPosable(group);

		// Add joints
		auto joints = bodyPart->joints();
		for (auto itb = joints.begin(); itb != joints.end(); ++itb) {
			out->addJoint(*itb);
		}

		// Add I/O descriptors
		auto io = bodyPart->getIO();
		for (auto itb = io.begin(); itb != io.end(); ++itb) {
			plugin << (*itb)->toXML();
		}
	}

	plugin << brainXML_;
	plugin << "</tol:robot_config>";
	plugin << "</plugin>\n";
	sb::ElementPtr pluginElem(new sb::StringElement(plugin.str()));
	out->addElement(pluginElem);

	return out;
}

} /* namespace tol_robogen */
