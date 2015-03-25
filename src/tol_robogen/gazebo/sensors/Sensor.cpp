/*
 * Sensor.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/sensors/Sensor.h>

namespace tol_robogen {
namespace gazebo {

Sensor::Sensor(::gazebo::physics::ModelPtr model, ::gazebo::sensors::SensorPtr sensor,
		std::string partId, unsigned int ioId):
	model_(model),
	sensor_(sensor),
	partId_(partId),
	ioId_(ioId)
{}

Sensor::~Sensor()
{}

std::string Sensor::partId() {
	return partId_;
}

unsigned int Sensor::ioId() {
	return ioId_;
}

::gazebo::sensors::SensorPtr Sensor::gzSensor() {
	return sensor_;
}

} /* namespace gazebo */
} /* namespace tol_robogen */
