#pragma once

#include "KinovaTypes.h"
#include <iostream>
#include <dlfcn.h>
#include <vector>
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include <stdio.h>
#include <unistd.h>

class KinovaArmController {
public:
    explicit KinovaArmController(const std::string& library_path = "Kinova.API.UsbCommandLayerUbuntu.so");
    ~KinovaArmController();

    // Zero in position
    const double JOINT_0_ZERO = 185.00;
    const double JOINT_1_ZERO = 180.00;
    const double JOINT_2_ZERO = 180.00;
    const double JOINT_3_ZERO = 180.00;
    const double JOINT_4_ZERO = 180.00;
    const double JOINT_5_ZERO = 40.00;


    // Core motion commands
    bool sendJointsVelocity(const std::vector<float>& velocities);
    bool sendJointsPosition(const std::vector<double>& positions);
    
    bool sendJointVelocity(size_t idx, float velocity);
    bool sendJointPosition(size_t idx, double position);

    bool sendToHome();

    // Query state (all joints)
    std::vector<double> getJointsPositions();
    std::vector<float> getJointsVelocities();
    std::vector<double> getJointsTorques();
    std::vector<double> getJointsCurrents();

    std::vector<float> getJointsTemperatures();

    // Query state (single joint, by index)
    double getJointPosition(size_t idx);
    float getJointVelocity(size_t idx);
    double getJointTorque(size_t idx);
    double getJointCurrent(size_t idx);
    double getJointTemperature(size_t idx);

    // Names
    std::vector<std::string> getJointNames() const;

    // Control modes
    bool setForceControlMode(bool active);

    // API Health
    bool isInitialized() const;

private:
    int loadSymbols();
    void initialize(int result);
    void closeAPI();
    void ensureInitialized() const;

    AngularPosition dataCommand;

    AngularPosition dataPosition;
    AngularPosition dataAngularPosition;
    GeneralInformations dataGeneral;

    void * commandLayer_handle;
    bool initialized_;

    // Function pointers (loaded via dlsym)
    int (*MyInitAPI)();
    int (*MyCloseAPI)();
    int (*MySendBasicTrajectory)(TrajectoryPoint command);
    int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
    int (*MySetActiveDevice)(KinovaDevice device);

    int (*MyStartForceControl)();
    int (*MyStopForceControl)();

    int (*MyInitFingers)();
    int (*MyGetAngularCommand)(AngularPosition &);
    int (*MyGetAngularPosition)(AngularPosition &);
    int (*MyGetGeneralInformations)(GeneralInformations &);
    int (*MyGetActuatorAcceleration)(AngularAcceleration &);
    int (*MyGetAngularVelocity)(AngularPosition &);
    int (*MyGetAngularCurrent)(AngularPosition &Response);
    int (*MyGetAngularForce)(AngularPosition &);
    int (*MyGetAngularForceGravityFree)(AngularPosition &);
    int (*MyGetCartesianCommand)(CartesianPosition &);
    int (*MyGetCartesianPosition)(CartesianPosition &);
};
