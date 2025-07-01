#include "arm_controller/KinovaArmControllerTest.h"

// Constructor: initialize with home pose and dummy data
KinovaArmControllerTest::KinovaArmControllerTest()
    : positions_{185.0, 180.0, 180.0, 180.0, 180.0, 40.0},
      velocities_(6, 0.0f),
      torques_(6, 1.0),
      currents_(6, 0.5),
      temperatures_(6, 25.0f)
{
}

// Destructor
KinovaArmControllerTest::~KinovaArmControllerTest() = default;

bool KinovaArmControllerTest::sendJointsVelocity(const std::vector<float>& velocities) {
    if (velocities.size() != 6)
        throw std::invalid_argument("Expected 6 joint velocities");
    velocities_ = velocities;
    return true;
}

bool KinovaArmControllerTest::sendJointsPosition(const std::vector<double>& positions) {
    if (positions.size() != 6)
        throw std::invalid_argument("Expected 6 joint positions");
    positions_ = positions;
    return true;
}

bool KinovaArmControllerTest::sendToHome() {
    return sendJointsPosition({185.0, 180.0, 180.0, 180.0, 180.0, 40.0});
}

bool KinovaArmControllerTest::sendJointVelocity(size_t idx, float velocity) {
    if (idx >= velocities_.size())
        throw std::out_of_range("Invalid joint index");
    velocities_[idx] = velocity;
    return true;
}

bool KinovaArmControllerTest::sendJointPosition(size_t idx, double position) {
    if (idx >= positions_.size())
        throw std::out_of_range("Invalid joint index");
    positions_[idx] = position;
    return true;
}

std::vector<double> KinovaArmControllerTest::getJointsPositions() const {
    return positions_;
}

std::vector<float> KinovaArmControllerTest::getJointsVelocities() const {
    return velocities_;
}

std::vector<double> KinovaArmControllerTest::getJointsTorques() const {
    return torques_;
}

std::vector<double> KinovaArmControllerTest::getJointsCurrents() const {
    return currents_;
}

std::vector<float> KinovaArmControllerTest::getJointsTemperatures() const {
    return temperatures_;
}

double KinovaArmControllerTest::getJointPosition(size_t idx) const {
    if (idx >= positions_.size())
        throw std::out_of_range("Invalid joint index");
    return positions_[idx];
}

float KinovaArmControllerTest::getJointVelocity(size_t idx) const {
    if (idx >= velocities_.size())
        throw std::out_of_range("Invalid joint index");
    return velocities_[idx];
}

double KinovaArmControllerTest::getJointTorque(size_t idx) const {
    if (idx >= torques_.size())
        throw std::out_of_range("Invalid joint index");
    return torques_[idx];
}

double KinovaArmControllerTest::getJointCurrent(size_t idx) const {
    if (idx >= currents_.size())
        throw std::out_of_range("Invalid joint index");
    return currents_[idx];
}

float KinovaArmControllerTest::getJointTemperature(size_t idx) const {
    if (idx >= temperatures_.size())
        throw std::out_of_range("Invalid joint index");
    return temperatures_[idx];
}

std::vector<std::string> KinovaArmControllerTest::getJointNames() const {
    return {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
}
