/*
 * ModelController.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/Types.h>
#include <tol_robogen/gazebo/plugin/ModelController.h>
#include <tol_robogen/gazebo/motors/MotorFactory.h>
#include <tol_robogen/gazebo/brain/Brain.h>

#include <boost/bind.hpp>

#include <iostream>
#include <stdexcept>

namespace gz = gazebo;

namespace tol_robogen {
namespace gazebo {

ModelController::ModelController():
	// Default actuation time, this will be overwritten
	// by the plugin config in Load.
	actuationTime_(0),
	lastActuationSec_(0),
	lastActuationNsec_(0)
{}

ModelController::~ModelController()
{}

void ModelController::Load(gz::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  this->model = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = gz::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ModelController::OnUpdate, this, _1));

  // TODO this should be loaded from the world / config
  // For now use 1/25 (i.e. 40ms, i.e. 40,000,000 ns)
  actuationTime_ = 40e6;

  std::cout << "Plugin loaded." << std::endl;
  loadMotors(_sdf);

  // TODO Sensors

  // Load brain, this needs to be done after
  loadBrain(_sdf);
}

// Called by the world update start event
void ModelController::OnUpdate(const gz::common::UpdateInfo & _info) {
	unsigned int nsecPassed = (_info.simTime.sec - lastActuationSec_) * 1e9 +
							(_info.simTime.nsec - lastActuationNsec_);
	if (nsecPassed < actuationTime_) {
		// Not time to actuate yet
		return;
	}

	// Update simulation time
	lastActuationSec_ = _info.simTime.sec;
	lastActuationNsec_ = _info.simTime.nsec;

	// TODO Feed / update neural network
	brain_->step(motors_, _info.simTime.Double());
}


void ModelController::loadMotors(sdf::ElementPtr sdf) {
	// Use HasElement to prevent SDF from doing AddElement when no motor is present.
	auto motor = sdf->HasElement("tol:motor") ? sdf->GetElement("tol:motor") : sdf::ElementPtr();
    while (motor) {
    	auto motorObj = MotorFactory::create(motor, this->model);
    	motors_.push_back(motorObj);
    	std::cout << "Found a motor!" << std::endl;
    	motor = motor->GetNextElement("tol:motor");
    }
}

void ModelController::loadBrain(sdf::ElementPtr sdf) {
	auto brain = sdf->GetElement("tol:brain");

	if (!brain) {
		std::cerr << "No robot brain detected, this is probably an error." << std::endl;
		return;
	}

	brain_.reset(new Brain(brain, motors_));
}

} /* namespace gazebo */
} /* namespace tol_robogen */
