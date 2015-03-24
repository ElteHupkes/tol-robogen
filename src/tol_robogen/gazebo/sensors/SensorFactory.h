/*
 * SensorFactory.h
 *
 *  Created on: Mar 24, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_SENSORS_SENSORFACTORY_H_
#define TOL_ROBOGEN_GAZEBO_SENSORS_SENSORFACTORY_H_

#include <tol_robogen/gazebo/Types.h>
#include <gazebo/common/common.hh>

namespace tol_robogen {
namespace gazebo {

class SensorFactory {
private:
	// Singleton
	SensorFactory();
public:
	virtual ~SensorFactory();

	/**
	 * Creates a new sensor in the given model, from the
	 * given SDF element pointer.
	 */
	static SensorPtr create(sdf::ElementPtr sensor, ::gazebo::physics::ModelPtr model);
};

} /* namespace gazebo */
} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_GAZEBO_SENSORS_SENSORFACTORY_H_ */
