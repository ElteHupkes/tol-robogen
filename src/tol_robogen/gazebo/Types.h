#ifndef TOL_ROBOGEN_GZ_MODEL_TYPES_H_
#define TOL_ROBOGEN_GZ_MODEL_TYPES_H_

#include <memory>

namespace tol_robogen {
namespace gazebo {
	class Motor;
	class Sensor;
	class Brain;

	typedef std::shared_ptr< Brain > BrainPtr;
	typedef std::shared_ptr< Motor > MotorPtr;
	typedef std::shared_ptr< Sensor > SensorPtr;
}
}

#endif
