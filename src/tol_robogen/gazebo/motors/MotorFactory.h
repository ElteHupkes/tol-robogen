/*
 * MotorFactory.h
 *
 *  Created on: Mar 16, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_MOTORS_MOTORFACTORY_H_
#define TOL_ROBOGEN_GAZEBO_MOTORS_MOTORFACTORY_H_

#include <tol_robogen/gazebo/Types.h>

#include <gazebo/common/common.hh>

namespace tol_robogen {
namespace gazebo {

class MotorFactory {
private:
	MotorFactory();
public:
	virtual ~MotorFactory();

	/**
	 * Creates a motor for the given model for the given SDF element.
	 */
	static MotorPtr create(sdf::ElementPtr motor, ::gazebo::physics::ModelPtr model);
};

} /* namespace gazebo */
} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_GAZEBO_MOTORS_MOTORFACTORY_H_ */
