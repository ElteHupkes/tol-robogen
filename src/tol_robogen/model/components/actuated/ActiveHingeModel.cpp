/*
 * TODO License
 */
#include <tol_robogen/model/components/actuated/ActiveHingeModel.h>
#include <tol_robogen/model/io/ServoMotor.h>

namespace sb = sdf_builder;

namespace tol_robogen {

// These are all in grams and millimeter
const float ActiveHingeModel::MASS_SERVO = 9;
const float ActiveHingeModel::MASS_SLOT = 7;
const float ActiveHingeModel::MASS_FRAME = 1.2;

const float ActiveHingeModel::SLOT_WIDTH = 34;
const float ActiveHingeModel::SLOT_THICKNESS = 1.5;

const float ActiveHingeModel::FRAME_LENGTH = 18;
const float ActiveHingeModel::FRAME_HEIGHT = 10;
const float ActiveHingeModel::FRAME_ROTATION_OFFSET = 14; // Left to right

const float ActiveHingeModel::SERVO_LENGTH = 24.5;
const float ActiveHingeModel::SERVO_HEIGHT = 10;
const float ActiveHingeModel::SERVO_ROTATION_OFFSET = 20.5; // Right to left

ActiveHingeModel::ActiveHingeModel(std::string id, const Configuration & conf) :
		Component(id, conf)
{}

ActiveHingeModel::~ActiveHingeModel() {

}

bool ActiveHingeModel::initModel() {
	// Create the 4 components of the hinge
	hingeRoot_ = this->createLink(B_SLOT_A_ID);
	auto frame = this->createLink(B_FRAME_ID);
	auto servo = this->createLink(B_SERVO_ID);
	hingeTail_ = this->createLink(B_SLOT_B_ID);

	// Set the masses, geometries and positions for the various boxes
	float separation = inMm(0.1);


	double thickness = inMm(SLOT_THICKNESS);
	double width = inMm(SLOT_WIDTH);
	double frameLength = inMm(FRAME_LENGTH);
	double servoLength = inMm(SERVO_LENGTH);
	double slotMass = inGrams(MASS_SLOT);

	// Create box for root
	hingeRoot_->makeBox(slotMass, thickness, width, width);

	// Box and position for frame
	double xFrame = thickness / 2 + separation + frameLength / 2 -
			thickness;
	frame->makeBox(inGrams(MASS_FRAME), frameLength, width, inMm(FRAME_HEIGHT));
	frame->position(sb::Vector3(xFrame, 0, 0));

	// Box and position for servo
	double xServo = xFrame + (inMm(FRAME_ROTATION_OFFSET) - (frameLength / 2))
				+ (servoLength / 2 - (servoLength - inMm(SERVO_ROTATION_OFFSET)));
	servo->makeBox(inGrams(MASS_SERVO), servoLength, width, inMm(SERVO_HEIGHT));
	servo->position(sb::Vector3(xServo, 0, 0));

	// Color the servo black so we can easily recognize it
	sb::ElementPtr color(new sb::StringElement("<material>"
          "<script>"
            "<uri>file://media/materials/scripts/gazebo.material</uri>"
            "<name>Gazebo/Black</name>"
          "</script>"
        "</material>"));
	auto vis = std::dynamic_pointer_cast< sb::Visual >(servo->posables()[1]);
	vis->addElement(color);

	// Box and position for tail
	double xTail = xServo + servoLength / 2 + separation + thickness / 2;
	hingeTail_->makeBox(slotMass, thickness, width, width);
	hingeTail_->position(sb::Vector3(xTail, 0, 0));

	// Create joints to hold pieces in position
	// root <-> frame
	this->fixLinks(hingeRoot_, frame, sb::Vector3(-frameLength / 2, 0, 0));

	// Frame <-> servo (hinge)
	// For a parent/child joint in Gazebo, it calls dJointAttach(child, parent);
	// for equivalency with Robogen frame must thus be child
	sb::JointPtr revolve(new sb::RevoluteJoint(servo, frame));
	// Joint position/axis are in child reference frame, so "frame" in this case
	revolve->axis->xyz(sb::Vector3(0, 1, 0));
	revolve->position(sb::Vector3((-frameLength / 2) + inMm(FRAME_ROTATION_OFFSET), 0, 0));
	this->addJoint(revolve);

	// Servo <-> tail
	this->fixLinks(servo, hingeTail_, sb::Vector3(-thickness / 2, 0, 0));

	// Register the revolve hinge as a motor
	// TODO Need to figure out the correct force / gain scalings
	double torque = inNm(ServoMotor::DEFAULT_MAX_FORCE_SERVO);
	double maxError = ServoMotor::MAX_POS_RAD - ServoMotor::MIN_POS_RAD;

	// This is the proportional gain of the motor's PID controller.
	// The output of Gazebo's PID controllor is applied
	// as a force directly in the JointController. The result of
	// the statement below is that the force will be maximal (times
	// a DEFAULT_GAIN) if the error is maximal, whereas it will of
	// course be zero if the error is zero. We probably need something
	// more clever than this; for instance maximum force cannot be applied
	// at this point. We'll have to use derivative / integral gain for
	// that, but for testing now this is fine.
	// TODO Tune this
	double gain = torque / maxError;
	IOPtr motor(new ServoMotor(id_, 0, revolve,
			torque, false, gain));
	this->addIO(motor);

	return true;
}

sb::LinkPtr ActiveHingeModel::getRoot() {
	return hingeRoot_;
}

sb::LinkPtr ActiveHingeModel::getSlot(unsigned int i) {
	if (i == SLOT_A) {
		return hingeRoot_;
	} else {
		return hingeTail_;
	}
}

sb::Vector3 ActiveHingeModel::getSlotPosition(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	sb::Vector3 slotAxis = this->getSlotAxis(i);
	sb::Vector3 curPos = (i == SLOT_A) ?
			hingeRoot_->position() : hingeTail_->position();

	return curPos + slotAxis * (inMm(SLOT_THICKNESS) / 2);
}

sb::Vector3 ActiveHingeModel::getSlotAxis(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	sb::Quaternion quat;
	sb::Vector3 axis;

	if (i == SLOT_A) {

		quat = hingeRoot_->rotation();
		axis[0] = -1; axis[1] = 0; axis[2] = 0;

	} else if (i == SLOT_B) {

		quat = hingeTail_->rotation();
		axis[0] = 1; axis[1] = 0; axis[2] = 0;

	}

	return quat * axis;

}

sb::Vector3 ActiveHingeModel::getSlotOrientation(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	sb::Quaternion quat;
	sb::Vector3 axis;

	if (i == SLOT_A) {

		quat = hingeRoot_->rotation();
		axis[0] = 0; axis[1] = 1; axis[2] = 0;

	} else if (i == SLOT_B) {

		quat = hingeTail_->rotation();
		axis[0] = 0; axis[1] = 1; axis[2] = 0;

	}

	return quat * axis;

}

//void ActiveHingeModel::getMotors(
//		std::vector<boost::shared_ptr<Motor> >& motors) {
//	motors.resize(1);
//	motors[0] = this->motor_;
//}

}
