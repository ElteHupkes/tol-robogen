/*
 * ImuSensor.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/sensors/ImuSensor.h>

#include <iostream>
#include <stdexcept>

namespace gz = gazebo;

namespace tol_robogen {
namespace gazebo {

ImuSensor::ImuSensor(::gazebo::physics::ModelPtr model, ::gazebo::sensors::SensorPtr sensor,
		std::string partId, unsigned int ioId):
		Sensor(model, sensor, partId, ioId)
{
	this->castSensor_ = boost::dynamic_pointer_cast<gz::sensors::ImuSensor>(sensor);

	if (!this->castSensor_) {
		throw std::runtime_error("Creating an IMU sensor with a non-IMU sensor object.");
	}
}

ImuSensor::~ImuSensor() {}

float ImuSensor::read() {
	if (ioId_ < 3) {
		return this->castSensor_->GetLinearAcceleration()[ioId_];
	} else if(ioId_ < 6) {
		return this->castSensor_->GetAngularVelocity()[ioId_ - 3];
	} else {
		std::cerr << "Invalid I/O ID " << ioId_ << " for IMU sensor, result will be zero.";
		return 0;
	}
}

} /* namespace gazebo */
} /* namespace tol_robogen */
