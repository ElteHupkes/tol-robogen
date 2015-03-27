/*
 * ModelController.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/plugin/ModelController.h>
#include <tol_robogen/gazebo/motors/MotorFactory.h>
#include <tol_robogen/gazebo/sensors/SensorFactory.h>
#include <tol_robogen/gazebo/sensors/Sensor.h>
#include <tol_robogen/gazebo/brain/Brain.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>

#include <boost/bind.hpp>
#include <boost/algorithm/string/replace.hpp>

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
	this->world = _parent->GetWorld();

	std::cout << "Plugin loaded." << std::endl;

	if (!_sdf->HasElement("tol:robot_config")) {
		std::cerr << "No `tol:robot_config` element found, controller not initialized."
			  << std::endl;
		return;
	}

	auto settings = _sdf->GetElement("tol:robot_config");

	// Load motors
	loadMotors(settings);

	// Load sensors
	loadSensors(settings);

	// Load brain, this needs to be done after the motors and
	// sensors so they can be reordered.
	loadBrain(settings);

	if (!this->driver) {
		std::cerr << "No driving sensor was found, robot will be still." << std::endl;
		return;
	}

	// Connect to the update event of the core (IMU) sensor
	auto sensor = this->driver->gzSensor();

	this->updateConnection_ = sensor->ConnectUpdated(boost::bind(&ModelController::OnUpdate, this));

	// TODO Pretty sure this just forces the sensor to be always
	// on again; can we make it work without this?
	sensor->SetActive(true);
}

// Called when the driver sensor (i.e. core component) updates
void ModelController::OnUpdate() {
	auto simTime = this->world->GetSimTime();
	unsigned int nsecPassed = (simTime.sec - lastActuationSec_) * 1e9 +
								(simTime.nsec - lastActuationNsec_);

	lastActuationSec_ = simTime.sec;
	lastActuationNsec_ = simTime.nsec;
	brain_->update(motors_, sensors_, simTime.Double(), nsecPassed);
}

void ModelController::loadMotors(sdf::ElementPtr sdf) {
	if (!sdf->HasElement("tol:motor")) {
		return;
	}

	auto motor = sdf->GetElement("tol:motor");
    while (motor) {
    	auto motorObj = MotorFactory::create(motor, this->model, actuationTime_);
    	motors_.push_back(motorObj);
    	motor = motor->GetNextElement("tol:motor");
    }
}

void ModelController::loadSensors(sdf::ElementPtr sdf) {

	if (!sdf->HasElement("tol:sensor")) {
		return;
	}

	auto sensor = sdf->GetElement("tol:sensor");
	while (sensor) {
		auto sensorObj = SensorFactory::create(sensor, this->model);
		sensors_.push_back(sensorObj);

		if (sensor->HasAttribute("driver")) {
			bool isDriver;
			sensor->GetAttribute("driver")->Get(isDriver);

			if (isDriver) {
				this->driver = sensorObj;
			}
		}

		sensor = sensor->GetNextElement("tol:sensor");
	}
}

void ModelController::loadBrain(sdf::ElementPtr sdf) {
	auto brain = sdf->GetElement("tol:brain");

	if (!brain) {
		std::cerr << "No robot brain detected, this is probably an error." << std::endl;
		return;
	}

	brain_.reset(new Brain(brain, motors_, sensors_));
}

} /* namespace gazebo */
} /* namespace tol_robogen */
