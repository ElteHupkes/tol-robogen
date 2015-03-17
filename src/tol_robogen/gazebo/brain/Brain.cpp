/*
 * NeuralNetwork.cpp
 *
 *  Created on: Mar 16, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/brain/Brain.h>
#include <tol_robogen/gazebo/motors/Motor.h>

#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <cstdlib>
#include <map>
#include <string>

namespace tol_robogen {
namespace gazebo {

Brain::Brain(sdf::ElementPtr node, std::vector< MotorPtr > & motors) {
	// The neural network is initialized with
	// the following parameters (which are to be determined):
	// Number of inputs
	unsigned int nInputs = 0;

	// Number of outputs
	unsigned int nOutputs = 0;

	// Number of hidden neurons
	unsigned int nHidden = 0;

	// Connection weights; we can have connections from
	// every input / output / hidden neuron to every
	// output / hidden neuron. We fill weight with zeros immediately
	float weights[(MAX_INPUT_NEURONS + MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)
	             * (MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];
	memset(weights, 0, sizeof(weights));

	// Neuron parameters, this is a maximum of 3
	// per neuron depending on the type.
	float params[MAX_PARAMS * (MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];

	// Neuron types, input neurons are fixed
	unsigned int types[(MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];

	// We now setup the neural network and its parameters. Furthermore,
	// we rearrange the motor and sensor arrays such that the index
	// of each sensor matches that of its input neuron, and the
	// index of each motor matches the output neuron.

	// Stores the position of each neuron ID, relative to its type
	std::map<std::string, unsigned int> positionMap;

	// Stores the type of each neuron ID
	std::map<std::string, std::string> layerMap;

	// Fetch the first neuron; note the HasElement call is necessary to prevent SDF from complaining
	// if no neurons are present.
	auto neuron = node->HasElement("tol:neuron") ? node->GetElement("tol:neuron") : sdf::ElementPtr();
	while (neuron) {
		auto layerParam = neuron->GetAttribute("layer");
		auto typeParam = neuron->GetAttribute("type");
		auto idParam = neuron->GetAttribute("id");
		auto ioPair = neuron->GetElement("tol:io_pair");

		// TODO error handling if the parameters are missing
		auto neuronId = idParam->GetAsString();
		auto layer = layerParam->GetAsString();
		auto type = typeParam ? typeParam->GetAsString() : "simple";
		auto partId = ioPair->GetAttribute("part_id")->GetAsString();

		unsigned int ioId;
		ioPair->GetAttribute("io_id")->Get(ioId);

		if (layerMap.count(neuronId)) {
			std::cerr << "Duplicate neuron ID '"
					<< neuronId << "'" << std::endl;
			throw std::runtime_error("Robot brain error");
		}

		layerMap[neuronId] = layer;

		if ("input" == layer) {
			if (nInputs >= MAX_INPUT_NEURONS) {
				std::cerr << "The number of input neurons(" << (nInputs + 1)
						<< ") is greater than the maximum allowed one ("
						<< MAX_INPUT_NEURONS << ")" << std::endl;
				throw std::runtime_error("Robot brain error");
			}

			// TODO match with sensors
			positionMap[neuronId] = nInputs;
			nInputs++;
		} else if ("output" == layer) {
			if (nOutputs >= MAX_OUTPUT_NEURONS) {
				std::cerr << "The number of output neurons(" << (nOutputs + 1)
						<< ") is greater than the maximum allowed one ("
						<< MAX_OUTPUT_NEURONS << ")" << std::endl;
				throw std::runtime_error("Robot brain error");
			}

			// Find the index of the corresponding motor in the motor list
			unsigned int motorIndex;
			for (motorIndex = 0; motorIndex < motors.size(); ++motorIndex) {
				auto motor = motors[motorIndex];
				if (motor->partId() == partId && motor->ioId() == ioId) {
					break;
				}
			}

			if (motorIndex >= motors.size()) {
				std::cerr << "Motor for part " << partId <<
						"with IO ID " << ioId << " could not be located"
						<< std::endl;
				throw std::runtime_error("Robot brain error");
			}

			// If the motor is not at the correct index in the motor
			// array, swap with whatever's there. In the end this must
			// mean that every motor is at its correct position.
			if (nOutputs != motorIndex) {
				std::iter_swap(motors.begin() + nOutputs,
						motors.begin() + motorIndex);
			}

			// Parameter starting index
			unsigned int idx = nOutputs * MAX_PARAMS;
			neuronHelper(&params[0], &types[0], idx, nOutputs, type, neuron);

			positionMap[neuronId] = nOutputs;
			nOutputs++;
		} else if ("hidden" == layer) {
			if (nHidden >= MAX_HIDDEN_NEURONS) {
				std::cerr << "The number of hidden neurons(" << (nHidden + 1)
						<< ") is greater than the maximum allowed one ("
						<< MAX_HIDDEN_NEURONS << ")" << std::endl;
				throw std::runtime_error("Robot brain error");
			}

			// Parameter starting index. We don't actually *know*
			// this yet, since more output neurons might arrive,
			// but we'll just shift the items back later if required
			int idx = (MAX_OUTPUT_NEURONS * MAX_PARAMS) + nHidden * MAX_PARAMS;
			int typeIdx = MAX_OUTPUT_NEURONS + nHidden;
			neuronHelper(&params[0], &types[0], idx, typeIdx, type, neuron);

			positionMap[neuronId] = nHidden;
			nHidden++;
		} else {
			std::cerr << "Unknown neuron layer '" << layer << "'." << std::endl;
			throw std::runtime_error("Robot brain error");
		}

		// Load the next neuron
		neuron = neuron->GetNextElement("tol:neuron");
	}

	// If we have fewer output neurons than allowed, the hidden neuron
	// settings are shifted to the right. Shift them back here.
	if (nOutputs < MAX_OUTPUT_NEURONS) {
		for (unsigned int i = 0; i < nHidden; ++i) {
			// Shift types backwards
			types[nOutputs + i] = types[MAX_OUTPUT_NEURONS + i];

			// Shift all params backwards
			unsigned int paramFrom = (MAX_OUTPUT_NEURONS * MAX_PARAMS) + i * MAX_PARAMS;
			unsigned int paramTo = (nOutputs * MAX_PARAMS) + i * MAX_PARAMS;
			for (unsigned int j = 0; j < MAX_PARAMS; ++j) {
				params[paramTo + j] = params[paramFrom + j];
			}
		}

		// No need to zero out the end of the array; this is treated as garbage.
	}

	if (nOutputs != motors.size()) {
		std::cerr << "The number of output neurons (" << nOutputs
				<< ") differs from the number of motors (" << motors.size()
				<< ")" << std::endl;
		throw std::runtime_error("Robot brain error");
	}

	// Decode connections
	unsigned int nNonInputs = nOutputs + nHidden;
	auto connection = node->HasElement("tol:connection") ? node->GetElement("tol:connection") : sdf::ElementPtr();
	while (connection) {
		auto srcParam = connection->GetAttribute("src");
		auto dstParam = connection->GetAttribute("dest");
		auto weightParam = connection->GetAttribute("weight");

		if (!srcParam || !dstParam || !weightParam) {
			std::cerr << "Missing required connection attributes." << std::endl;
			throw std::runtime_error("Robot brain error");
		}

		auto src = srcParam->GetAsString();
		auto dst = srcParam->GetAsString();

		if (!layerMap.count(src)) {
			std::cerr << "Source neuron '" << src << "' is unknown.";
			throw std::runtime_error("Robot brain error");
		}

		if (!layerMap.count(dst)) {
			std::cerr << "Destination neuron '" << dst << "' is unknown.";
			throw std::runtime_error("Robot brain error");
		}

		auto srcLayer = layerMap[src];
		auto dstLayer = layerMap[dst];

		int srcNeuronPos;
		int dstNeuronPos;

		srcNeuronPos = positionMap[src];
		dstNeuronPos = positionMap[dst];

		if ("hidden" == srcLayer) {
			// Offset by outputs if hidden neuron; nothing
			// needs to happen for output or input neurons.
			srcNeuronPos += nOutputs;
		}

		if ("input" == dstLayer) {
			std::cerr << "Destination neuron '" << dst << "' is an input neuron.";
			throw std::runtime_error("Robot brain error");
		} else if ("hidden" == dstLayer) {
			// Offset by outputs if hidden neuron
			dstNeuronPos += nOutputs;
		}

		// Determine the index of the weight.
		// Each output / hidden neuron can be used as an output, input
		// neurons can only be used as an input. By default, we offset
		// the index by the position of the neuron, which is the
		// correct position for an input neuron:
		unsigned int idx = (srcNeuronPos * nNonInputs) + dstNeuronPos;

		if ("input" != srcLayer) {
			// The output neuron list starts after all input neuron
			// connections, so we need to offset it from all
			// nInputs * nNonInputs of such connections:
			idx += (nInputs * nNonInputs);
		}

		// Get has a return argument here
		weightParam->Get(weights[idx]);

		// Load the next connection
		connection = connection->GetNextElement("tol:connection");
	}

	// Create the actual neural network
	neuralNetwork_.reset(new NeuralNetwork);
}

Brain::~Brain() {}

// TODO Check for erroneous / missing parameters
void Brain::neuronHelper(float* params, unsigned int* types, unsigned int paramIdx,
		unsigned int typeIdx, const std::string& type, sdf::ElementPtr neuron) {
	if ("sigmoid" == type) {
		types[typeIdx] = SIGMOID;

		// Set bias and gain parameters
		params[paramIdx] = neuron->GetElement("tol:bias")->Get< float >();
		params[paramIdx + 1] = neuron->GetElement("tol:gain")->Get< float >();
	} else if ("oscillator" == type) {
		types[typeIdx] = OSCILLATOR;

		// Set period, phase offset and gain
		params[paramIdx] = neuron->GetElement("tol:period")->Get< float >();
		params[paramIdx + 1] = neuron->GetElement("tol:phase_offset")->Get< float >();
		params[paramIdx + 2] = neuron->GetElement("tol:gain")->Get< float >();
	} else {
		std::cout << "only sigmoid and oscillator neurons supported currently" << std::endl;
		throw std::runtime_error("Robot brain error");
	}
}

} /* namespace gazebo */
} /* namespace tol_robogen */
