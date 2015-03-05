/*
 * ModelController.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/Types.h>
#include <tol_robogen/gazebo/plugin/ModelController.h>
#include <tol_robogen/gazebo/motors/Motors.h>

#include <iostream>
#include <stdexcept>

namespace gz = gazebo;

namespace tol_robogen {
namespace gazebo {

ModelController::ModelController()
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

  std::cout << "Plugin loaded." << std::endl;
  loadMotors(_sdf);
}

// Called by the world update start event
void ModelController::OnUpdate(const gz::common::UpdateInfo & _info) {
  if (_info.simTime > 10.0) {
	  motors_[0]->update(5.0);
  }
}


void ModelController::loadMotors(sdf::ElementPtr sdf) {
    auto motor = sdf->GetElement("tol:motor");

    while (motor) {
    	auto typeParam = motor->GetAttribute("type");
    	auto jointNameParam = motor->GetAttribute("joint");

    	if (!type || !jointNameParam) {
    		std::cerr << "Motor is missing 'type' or 'joint' attributes." << std::endl;
			throw std::runtime_error("Motor error");
    	}

    	auto jointName = jointNameParam->GetAsString();
    	gz::physics::JointPtr joint = this->model->GetJoint(jointName);

    	if (!joint) {
    		std::cerr << "Could not locate joint  '" << jointName << "'." << std::endl;
    		throw std::runtime_error("Motor error");
    	}

    	MotorPtr motorObj;
    	auto type = typeParam->GetAsString();
    	if ("servo" == type) {
    		motorObj.reset(new ServoMotor(this->model, joint));
    	} else {
    		std::cerr << "Motor type '" << type <<
    				"' is invalid." << std::endl;
    		throw std::runtime_error("Motor error");
    	}

    	motors_.push_back(motorObj);
    	std::cout << "Found a motor!" << std::endl;
    	motor = motor->GetNextElement("tol:motor");
    }
}

} /* namespace gazebo */
} /* namespace tol_robogen */
