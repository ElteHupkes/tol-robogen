#ifndef TOL_ROBOGEN_MODEL_TYPES_H_
#define TOL_ROBOGEN_MODEL_TYPES_H_
/**
 * Some typedefs for the model subdirectory
 */
#include <memory>

namespace tol_robogen {
	class Robot;
	class Model;
	class Motor;
	class Connection;
	class RobotRepresentation;
	class PartRepresentation;
	class NeuralNetworkRepresentation;
	class NeuronRepresentation;
	class ActuatedComponent;
	class PerceptiveComponent;

	typedef std::shared_ptr< Robot > RobotPtr;
	typedef std::shared_ptr< Model > ModelPtr;
	typedef std::shared_ptr< Motor > MotorPtr;
	typedef std::shared_ptr< ActuatedComponent > ActuatedComponentPtr;
	typedef std::shared_ptr< PerceptiveComponent > PerceptiveComponentPtr;
	typedef std::shared_ptr< Connection > ConnectionPtr;
	typedef std::shared_ptr< PartRepresentation > PartRepresentationPtr;
	typedef std::shared_ptr< RobotRepresentation > RobotRepresentationPtr;
	typedef std::weak_ptr< PartRepresentation > WeakPartRepresentationPtr;
	typedef std::shared_ptr< NeuralNetworkRepresentation > NeuralNetworkRepresentationPtr;
	typedef std::shared_ptr< NeuronRepresentation > NeuronRepresentationPtr;
	typedef std::weak_ptr< NeuronRepresentation > WeakNeuronRepresentationPtr;
}

#endif
