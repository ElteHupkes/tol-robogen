#ifndef TOL_ROBOGEN_GZ_MODEL_TYPES_H_
#define TOL_ROBOGEN_GZ_MODEL_TYPES_H_

#include <memory>

namespace tol_robogen {
namespace gazebo {
	class Motor;

	typedef std::shared_ptr< Motor > MotorPtr;
}
}

#endif
