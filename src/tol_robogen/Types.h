#ifndef SDF_BUILDER_MODEL_TYPES_H_
#define SDF_BUILDER_MODEL_TYPES_H_
/**
 * Some typedefs for the model subdirectory
 */
#include <memory>

namespace tol_robogen {
	class Robot;
	class Model;
	class Connection;
	class PartRepresentation;

	typedef std::shared_ptr< Robot > RobotPtr;
	typedef std::shared_ptr< Model > ModelPtr;
	typedef std::shared_ptr< Connection > ConnectionPtr;
	typedef std::shared_ptr< PartRepresentation > PartRepresentationPtr;
}

#endif
