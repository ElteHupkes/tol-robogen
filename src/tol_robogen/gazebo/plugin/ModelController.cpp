/*
 * ModelController.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/plugin/ModelController.h>
#include <iostream>

namespace gazebo {

ModelController::ModelController()
{}

ModelController::~ModelController()
{}

void ModelController::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
  // Store the pointer to the model
  this->model = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ModelController::OnUpdate, this, _1));

  std::cout << "Helloo, here I am!" << std::endl;
}

// Called by the world update start event
void ModelController::OnUpdate(const common::UpdateInfo & /*_info*/) {
  // Apply a small linear velocity to the model.
  this->model->SetLinearVel(math::Vector3(.03, 0, 0));
}

} /* namespace gazebo */
