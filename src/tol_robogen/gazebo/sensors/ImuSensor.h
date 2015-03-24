/*
 * ImuSensor.h
 *
 *  Created on: Mar 24, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_SENSORS_IMUSENSOR_H_
#define TOL_ROBOGEN_GAZEBO_SENSORS_IMUSENSOR_H_

#include <tol_robogen/gazebo/sensors/Sensor.h>

namespace tol_robogen {
namespace gazebo {

class ImuSensor: public Sensor {
public:
	ImuSensor(::gazebo::physics::ModelPtr model, ::gazebo::sensors::SensorPtr sensor,
			std::string partId, unsigned int ioId);
	virtual ~ImuSensor();

	/**
	 * Read the value of this IMU sensor corresponding
	 * to the set IO ID
	 */
	virtual float read();

private:
	/**
	 * Sensor dynamically casted to correct type,
	 * so it needs to happen only once.
	 */
	::gazebo::sensors::ImuSensorPtr castSensor_;
};

} /* namespace gazebo */
} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_GAZEBO_SENSORS_IMUSENSOR_H_ */
