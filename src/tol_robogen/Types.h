#ifndef TOL_ROBOGEN_MODEL_TYPES_H_
#define TOL_ROBOGEN_MODEL_TYPES_H_
/**
 * Some typedefs for the model subdirectory
 */
#include <memory>

namespace tol_robogen {
	class Robot;
	class Component;
	class IO;
	class Sensor;
	class Connection;
	class RobotRepresentation;
	class PartRepresentation;
	class NeuralNetworkRepresentation;
	class NeuronRepresentation;

	typedef std::shared_ptr< Robot > RobotPtr;
	typedef std::shared_ptr< Component > ComponentPtr;
	typedef std::shared_ptr< IO > IOPtr;
	typedef std::shared_ptr< Connection > ConnectionPtr;
	typedef std::shared_ptr< PartRepresentation > PartRepresentationPtr;
	typedef std::shared_ptr< RobotRepresentation > RobotRepresentationPtr;
	typedef std::weak_ptr< PartRepresentation > WeakPartRepresentationPtr;
	typedef std::shared_ptr< NeuralNetworkRepresentation > NeuralNetworkRepresentationPtr;
	typedef std::shared_ptr< NeuronRepresentation > NeuronRepresentationPtr;
	typedef std::weak_ptr< NeuronRepresentation > WeakNeuronRepresentationPtr;
}

#endif
