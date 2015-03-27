/*
 * TouchSensor.h
 *
 *  Created on: Mar 27, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_SENSORS_TOUCHSENSOR_H_
#define TOL_ROBOGEN_GAZEBO_SENSORS_TOUCHSENSOR_H_

#include <tol_robogen/gazebo/sensors/Sensor.h>

namespace tol_robogen {
namespace gazebo {

class TouchSensor: public Sensor {
public:
	TouchSensor(::gazebo::physics::ModelPtr model, ::gazebo::sensors::SensorPtr sensor,
			std::string partId, unsigned int ioId);
	virtual ~TouchSensor();

	/**
	 * The touch sensor is boolean; it is either
	 * touching something or it is not. Since
	 * the NN works with floats, we return 0.0
	 * or 1.0.
	 */
	virtual float read();

	/**
	 * Called when the camera sensor is updated
	 */
	void OnUpdate();

private:
	/**
	 * The contact state at the last update
	 */
	bool lastValue_;

	/**
	 * Sensor dynamically casted to correct type,
	 * so it needs to happen only once.
	 */
	::gazebo::sensors::ContactSensorPtr castSensor_;

    // Pointer to the update connection
    ::gazebo::event::ConnectionPtr updateConnection_;
};

} /* namespace gazebo */
} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_GAZEBO_SENSORS_TOUCHSENSOR_H_ */
