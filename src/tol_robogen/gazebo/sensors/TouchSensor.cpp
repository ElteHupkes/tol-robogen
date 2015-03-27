/*
 * TouchSensor.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/sensors/TouchSensor.h>

#include <boost/bind.hpp>

namespace gz = gazebo;

namespace tol_robogen {
namespace gazebo {

TouchSensor::TouchSensor(::gazebo::physics::ModelPtr model, ::gazebo::sensors::SensorPtr sensor,
		std::string partId, unsigned int ioId):
		Sensor(model, sensor, partId, ioId),
		lastValue_(false)
{
	this->castSensor_ = boost::dynamic_pointer_cast<gz::sensors::ContactSensor>(sensor);

	if (!this->castSensor_) {
		std::cerr << "Creating a light sensor with a non-camera sensor object." << std::endl;
		throw std::runtime_error("Sensor error");
	}

	// Sensor must always update
	this->castSensor_->SetActive(true);

	// Add update connection that will produce new value
	this->updateConnection_ = sensor->ConnectUpdated(boost::bind(&TouchSensor::OnUpdate, this));
}

TouchSensor::~TouchSensor()
{}

void TouchSensor::OnUpdate() {
	auto contacts = this->castSensor_->GetContacts();
	this->lastValue_ = contacts.contact_size() > 0;
}

float TouchSensor::read() {
	return lastValue_ ? 1 : 0;
}

} /* namespace gazebo */
} /* namespace tol_robogen */
