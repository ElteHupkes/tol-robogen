/*
 * LightSensor.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/sensors/LightSensor.h>

namespace gz = gazebo;

namespace tol_robogen {
namespace gazebo {

LightSensor::LightSensor(::gazebo::physics::ModelPtr model, ::gazebo::sensors::SensorPtr sensor,
		std::string partId, unsigned int ioId):
	Sensor(model, sensor, partId, ioId)
{
	this->castSensor_ = boost::dynamic_pointer_cast<gz::sensors::CameraSensor>(sensor);

	if (!this->castSensor_) {
		std::cerr << "Creating a light sensor with a non-camera sensor object." << std::endl;
		throw std::runtime_error("Sensor error");
	}
}

LightSensor::~LightSensor()
{}

float LightSensor::read() {
//	std::cout << "Read light sensor" << std::endl;
	return 0;
}

} /* namespace gazebo */
} /* namespace tol_robogen */
