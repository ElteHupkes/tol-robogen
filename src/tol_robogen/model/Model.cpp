/*
 *
 */
#include <stdexcept>
#include <sstream>
#include <string>

#include <tol_robogen/model/Model.h>

namespace tol_robogen {

namespace sb = sdf_builder;

Model::Model(std::string id) :
		orientationToParentSlot_(0), // To get rid of the warning
		id_(id) {
}

Model::~Model() {

}

const std::string &Model::getId(){
	return id_;
}

sb::Vector3 Model::getRootPosition() {
	return this->getPosition(this->getRoot());
}

sb::Quaternion Model::getRootAttitude() {
	return this->getAttitude(this->getRoot());
}

void Model::setRootPosition(const sb::Vector3& pos) {

	sb::Vector3 curPosition = this->getRootPosition();
	sb::Vector3 translation = pos - curPosition;

	std::map<int, sb::LinkPtr>::iterator it = this->links_.begin();
	for (; it != this->links_.end(); ++it) {
		sb::Vector3 curBodyPos = this->getPosition(it->second);
		curBodyPos += translation;
		it->second->pose()->setPosition(curBodyPos);
	}
}

void Model::translateRootPosition(const sb::Vector3& translation) {

	sb::Vector3 newPosition = this->getRootPosition() + translation;
	this->setRootPosition(newPosition);

}

void Model::setRootAttitude(const sb::Quaternion& quat) {

	sb::Vector3 rootPosition = this->getRootPosition();

	std::map<int, sb::LinkPtr>::iterator it = this->links_.begin();
	for (; it != this->links_.end(); ++it) {
		sb::Vector3 curPosition = this->getPosition(it->second);
		sb::Vector3 relPosition = curPosition - rootPosition;

		// Rotate relPosition
		sb::Vector3 newPosition = quat * relPosition;
		it->second->setPosition(newPosition);

		sb::Quaternion curBodyAttitude = this->getAttitude(it->second);
		curBodyAttitude *= quat;
		it->second->setRotation(curBodyAttitude);
	}

	this->setRootPosition(rootPosition);

}

sb::Vector3 Model::getPosition(sb::LinkPtr link) {
	return link->pose()->position();
}

sb::Quaternion Model::getAttitude(sb::LinkPtr link) {
	return link->pose()->rotation();
}

sb::Vector3 Model::getBodyPosition(int id) {
	return this->getPosition(this->getLink(id));
}

sb::Quaternion Model::getBodyAttitude(int id) {
	return this->getAttitude(this->getLink(id));
}

sb::LinkPtr Model::getLink(int id) {
	std::map<int, sb::LinkPtr>::iterator it = this->links_.find(id);
	if (it == this->links_.end()) {
		std::cout
				<< "[Model] Error: The specified body does not exists in this model"
				<< std::endl;
		assert(it != this->links_.end());

		// Return empty link pointer
		return sb::LinkPtr();
	}
	return links_[id];
}

std::vector<sb::LinkPtr> Model::getLinks() {

	std::vector<sb::LinkPtr> bodies;
	std::map<int, sb::LinkPtr>::iterator it = this->links_.begin();
	for (; it != this->links_.end(); ++it) {
		bodies.push_back(it->second);
	}
	return bodies;

}

void Model::addLink(sb::LinkPtr body, int id) {
	this->links_.insert(std::pair<int, sb::LinkPtr>(id, body));
}

sb::LinkPtr Model::createLink(int label) {
	sb::LinkPtr b(new sb::Link("link_"+id_+std::to_string(label)));
	if (label >= 0) {
		this->addLink(b, label);
	}

	// Add the model to the posables anyway
	posables_.push_back(b);
	return b;
}

sb::LinkPtr Model::createLink() {
	return this->createLink(-1);
}

const std::vector< sb::JointPtr > & Model::getJoints() {
	return joints_;
}

const std::vector< sb::PosablePtr > & Model::getPosables() {
	return posables_;
}

void Model::addJoint(sb::JointPtr joint) {
	joints_.push_back(joint);
	posables_.push_back(joint);
}

sb::JointPtr Model::fixLinks(sb::LinkPtr parent, sb::LinkPtr child,
		const sb::Vector3& axis, const sb::Vector3& anchor) {
	sb::JointPtr joint(new sb::FixedJoint(parent, child));
	joint->setPosition(anchor);
	joint->axis->xyz(axis);
	this->addJoint(joint);
	return joint;
}

//dxGeom* Model::createCylinderGeom(dBodyID body, float mass,
//		const osg::Vec3& pos, int direction, float radius, float height) {
//
//	dMass massOde;
//	dMassSetCylinderTotal(&massOde, mass, direction, radius, height);
//	dBodySetMass(body, &massOde);
//	dxGeom* g = dCreateCylinder(this->getCollisionSpace(), radius, height);
//	dBodySetPosition(body, pos.x(), pos.y(), pos.z());
//	dGeomSetPosition(g, pos.x(), pos.y(), pos.z());
//
//	if (direction == 1) {
//
//		osg::Quat rotateCylinder;
//		rotateCylinder.makeRotate(osg::inDegrees(90.0), osg::Vec3(0, 1, 0));
//		dsb::Quaternion quatOde;
//		quatOde[0] = rotateCylinder.w();
//		quatOde[1] = rotateCylinder.x();
//		quatOde[2] = rotateCylinder.y();
//		quatOde[3] = rotateCylinder.z();
//		dBodySetsb::Quaternion(body, quatOde);
//
//	} else if (direction == 2) {
//
//		osg::Quat rotateCylinder;
//		rotateCylinder.makeRotate(osg::inDegrees(90.0), osg::Vec3(1, 0, 0));
//		dsb::Quaternion quatOde;
//		quatOde[0] = rotateCylinder.w();
//		quatOde[1] = rotateCylinder.x();
//		quatOde[2] = rotateCylinder.y();
//		quatOde[3] = rotateCylinder.z();
//		dBodySetsb::Quaternion(body, quatOde);
//
//	}
//
//	dGeomSetBody(g, body);
//
//	return g;
//
//}

//dxGeom* Model::createCapsuleGeom(dBodyID body, float mass, const osg::Vec3& pos,
//		int direction, float radius, float height) {
//
//	dMass massOde;
//	dMassSetCapsuleTotal(&massOde, mass, direction, radius, height);
//	dBodySetMass(body, &massOde);
//	dxGeom* g = dCreateCapsule(this->getCollisionSpace(), radius, height);
//	dBodySetPosition(body, pos.x(), pos.y(), pos.z());
//	dGeomSetPosition(g, pos.x(), pos.y(), pos.z());
//
//	if (direction == 1) {
//
//		osg::Quat rotateCapsule;
//		rotateCapsule.makeRotate(osg::inDegrees(90.0), osg::Vec3(0, 1, 0));
//		dsb::Quaternion quatOde;
//		quatOde[0] = rotateCapsule.w();
//		quatOde[1] = rotateCapsule.x();
//		quatOde[2] = rotateCapsule.y();
//		quatOde[3] = rotateCapsule.z();
//		dBodySetsb::Quaternion(body, quatOde);
//
//	} else if (direction == 2) {
//
//		osg::Quat rotateCapsule;
//		rotateCapsule.makeRotate(osg::inDegrees(90.0), osg::Vec3(1, 0, 0));
//		dQuaternion quatOde;
//		quatOde[0] = rotateCapsule.w();
//		quatOde[1] = rotateCapsule.x();
//		quatOde[2] = rotateCapsule.y();
//		quatOde[3] = rotateCapsule.z();
//		dBodySetQuaternion(body, quatOde);
//
//	}
//
//	dGeomSetBody(g, body);
//
//	return g;
//
//}

//dJointID Model::fixBodies(dBodyID b1, dBodyID b2, const osg::Vec3& /*axis*/) {
//	dJointID joint = dJointCreateFixed(this->getPhysicsWorld(), 0);
//	dJointAttach(joint, b1, b2);
//	dJointSetFixed(joint);
//	return joint;
//
//}

bool Model::setOrientationToParentSlot(int orientation){
	if (orientation < 0 || orientation > 3){
		std::cout << "Specified orientation to parent slot is not"\
				" between 0 and 3." << std::endl;
		return false;
	}
	this->orientationToParentSlot_ = orientation;
	return true;
}

int Model::getOrientationToParentSlot(){
	return this->orientationToParentSlot_;
}

}
