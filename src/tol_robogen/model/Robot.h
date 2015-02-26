/*
 * Robot.h
 *
 *  Created on: Feb 26, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_ROBOT_H_
#define TOL_ROBOGEN_ROBOT_H_

#include <sdf_builder/Parts.h>

//#include "robogen.pb.h"

namespace tol_robogen {

class Robot {
public:
	Robot();
	virtual ~Robot();

	/**
	 * Initializes the robot
	 */
//	bool Robot::init(const robogenMessage::Robot& robotSpec);

protected:
	/**
	 *
	 */
//	bool decodeBody(const robogenMessage::Body& robotBody);
};

} /* namespace sdf_builder */

#endif /* TOL_ROBOGEN_ROBOT_H_ */
