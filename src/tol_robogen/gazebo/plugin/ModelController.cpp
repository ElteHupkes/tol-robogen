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
	auto driver = loadSensors(settings);

	// Load brain, this needs to be done after the motors and
	// sensors so they can be reordered.
	loadBrain(settings);

	if (!driver) {
		std::cerr << "No driving sensor was found, robot will be still." << std::endl;
		return;
	}

	// Connect to the update event of the core (IMU) sensor
	auto sensor = driver->gzSensor();
	sensor->ConnectUpdated(boost::bind(&ModelController::OnUpdate, this));
}

// Called by the world update start event
//void ModelController::OnUpdate(const gz::common::UpdateInfo & _info) {
//	unsigned int nsecPassed = (_info.simTime.sec - lastActuationSec_) * 1e9 +
//							(_info.simTime.nsec - lastActuationNsec_);
//
//	if (nsecPassed < actuationTime_) {
//		// Not time to actuate yet
//		return;
//	}
//
//	// Update simulation time
//	lastActuationSec_ = _info.simTime.sec;
//	lastActuationNsec_ = _info.simTime.nsec;
//
//	// TODO Sensors
//	brain_->update(motors_, sensors_, _info.simTime.Double(), actuationTime_);
//}

void ModelController::OnUpdate() {
	std::cout << "Update!" << std::endl;
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

SensorPtr ModelController::loadSensors(sdf::ElementPtr sdf) {
	SensorPtr driver;

	if (!sdf->HasElement("tol:sensor")) {
		return driver;
	}

	auto sensor = sdf->GetElement("tol:sensor");
	while (sensor) {
		auto sensorObj = SensorFactory::create(sensor, this->model);
		sensors_.push_back(sensorObj);

		if (sensor->HasAttribute("driver")) {
			bool isDriver;
			sensor->GetAttribute("driver")->Get(isDriver);

			if (isDriver) {
				driver = sensorObj;
			}
		}

		sensor = sensor->GetNextElement("tol:sensor");
	}

	return driver;
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
