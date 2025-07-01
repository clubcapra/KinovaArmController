// KinovaArmControllerTest.h
#ifndef KINOVA_ARM_CONTROLLER_TEST_H
#define KINOVA_ARM_CONTROLLER_TEST_H

#include <vector>
#include <string>
#include <stdexcept>

// A stub controller for testing when the real Kinova arm is not connected.
class KinovaArmControllerTest {
public:
    KinovaArmControllerTest();
    ~KinovaArmControllerTest();

    // Motion commands
    bool sendJointsVelocity(const std::vector<float>& velocities);
    bool sendJointsPosition(const std::vector<double>& positions);
    bool sendToHome();

    bool sendJointVelocity(size_t idx, float velocity);
    bool sendJointPosition(size_t idx, double position);

    // State queries
    std::vector<double> getJointsPositions() const;
    std::vector<float>  getJointsVelocities() const;
    std::vector<double> getJointsTorques() const;
    std::vector<double> getJointsCurrents() const;
    std::vector<float>  getJointsTemperatures() const;

    // Single joint getters
    double getJointPosition(size_t idx) const;
    float  getJointVelocity(size_t idx) const;
    double getJointTorque(size_t idx) const;
    double getJointCurrent(size_t idx) const;
    float  getJointTemperature(size_t idx) const;

    // Joint names
    std::vector<std::string> getJointNames() const;

private:
    std::vector<double> positions_;
    std::vector<float>  velocities_;
    std::vector<double> torques_;
    std::vector<double> currents_;
    std::vector<float>  temperatures_;
};

#endif // KINOVA_ARM_CONTROLLER_TEST_H
