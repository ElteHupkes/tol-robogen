#ifndef SDF_BUILDER_MODEL_TYPES_H_
#define SDF_BUILDER_MODEL_TYPES_H_
/**
 * Some typedefs for the model subdirectory
 */
#include <memory>

namespace sdf_builder {
	class Robot;
	class Model;

	typedef std::shared_ptr< Robot > RobotPtr;
	typedef std::shared_ptr< Model > ModelPtr;
}

#endif
