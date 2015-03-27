/*
 * LightSensor.h
 *
 *  Created on: Mar 26, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_SENSORS_LIGHTSENSOR_H_
#define TOL_ROBOGEN_GAZEBO_SENSORS_LIGHTSENSOR_H_

#include <tol_robogen/gazebo/sensors/Sensor.h>

namespace tol_robogen {
namespace gazebo {

class LightSensor: public Sensor {
public:
	LightSensor(::gazebo::physics::ModelPtr model, ::gazebo::sensors::SensorPtr sensor,
			std::string partId, unsigned int ioId);
	virtual ~LightSensor();

	/**
	 * Returns a float intensity between 0 and 1
	 */
	virtual float read();

	/**
	 * Called when the camera sensor is updated
	 */
	void OnUpdate();

private:
	/**
	 * Sensor dynamically casted to correct type,
	 * so it needs to happen only once.
	 */
	::gazebo::sensors::CameraSensorPtr castSensor_;

	/**
	 * Size of the data which we determine once
	 */
	unsigned int dataSize_;

	/**
	 * Last calculated average
	 */
	float lastValue_;

private:
    // Pointer to the update connection
    ::gazebo::event::ConnectionPtr updateConnection_;
};

} /* namespace gazebo */
} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_GAZEBO_SENSORS_LIGHTSENSOR_H_ */
