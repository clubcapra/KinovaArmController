// KinovaArmController.cpp

#include "arm_controller/KinovaArmController.h"
#include <iostream>
#include <dlfcn.h>
#include <unistd.h>
#include <string.h>
#include <stdexcept>

// Hardware zero offsets and per-joint sign flips
static const double ZERO_OFFSETS[6] = {
    185.00,  // joint 1 hardware zero
    180.00,  // joint 2
    180.00,  // joint 3
    180.00,  // joint 4
    180.00,  // joint 5
     40.00   // joint 6
};
static const int SIGN[6] = {
    1,  // joint 1
   -1,  // joint 2 (flip)
   -1,  // joint 3 (flip)
    1,  // joint 4
    1,  // joint 5
    1   // joint 6
};

KinovaArmController::KinovaArmController(const std::string &library_path)
    : initialized_(false) {
    initialize(loadSymbols());
}

KinovaArmController::~KinovaArmController() {
    if (initialized_) {
        closeAPI();
    }
    if (commandLayer_handle) {
        dlclose(commandLayer_handle);
    }
}

// ------- Motion commands -------
bool KinovaArmController::sendJointsVelocity(const std::vector<float>& velocities) {
    ensureInitialized();
    if (velocities.size() != 6)
        throw std::invalid_argument("Expected 6 joint velocities");

    TrajectoryPoint pointToSend;
    pointToSend.InitStruct();
    pointToSend.Position.Type = ANGULAR_VELOCITY;

    // Apply sign flips
    pointToSend.Position.Actuators.Actuator1 = SIGN[0] * velocities[0];
    pointToSend.Position.Actuators.Actuator2 = SIGN[1] * velocities[1];
    pointToSend.Position.Actuators.Actuator3 = SIGN[2] * velocities[2];
    pointToSend.Position.Actuators.Actuator4 = SIGN[3] * velocities[3];
    pointToSend.Position.Actuators.Actuator5 = SIGN[4] * velocities[4];
    pointToSend.Position.Actuators.Actuator6 = SIGN[5] * velocities[5];

    for (int i = 0; i < 100; i++) {
        MySendBasicTrajectory(pointToSend);
        usleep(5000);
    }
    return true;
}

bool KinovaArmController::sendJointsPosition(const std::vector<double>& positions) {
    ensureInitialized();
    if (positions.size() != 6)
        throw std::invalid_argument("Expected 6 joint positions");

    TrajectoryPoint point;
    point.InitStruct();
    point.Position.Type = ANGULAR_POSITION;

    // Map user-space to hardware: hardware = SIGN * user + ZERO_OFFSET
    point.Position.Actuators.Actuator1 = SIGN[0] * positions[0] + ZERO_OFFSETS[0];
    point.Position.Actuators.Actuator2 = SIGN[1] * positions[1] + ZERO_OFFSETS[1];
    point.Position.Actuators.Actuator3 = SIGN[2] * positions[2] + ZERO_OFFSETS[2];
    point.Position.Actuators.Actuator4 = SIGN[3] * positions[3] + ZERO_OFFSETS[3];
    point.Position.Actuators.Actuator5 = SIGN[4] * positions[4] + ZERO_OFFSETS[4];
    point.Position.Actuators.Actuator6 = SIGN[5] * positions[5] + ZERO_OFFSETS[5];

    MySendBasicTrajectory(point);
    
    return true;
}

bool KinovaArmController::sendToHome() {
    // Send zeros in user-space; hardware offsets drive to true home
    sendJointsPosition({0, 0, 0, 0, 0, 0});
    return true;
}

// Single joint setters
bool KinovaArmController::sendJointVelocity(size_t idx, float velocity) {
    if (idx >= 6) throw std::out_of_range("Invalid joint index");
    auto current = getJointsVelocities();
    current[idx] = velocity;
    return sendJointsVelocity(current);
}

bool KinovaArmController::sendJointPosition(size_t idx, double position) {
    if (idx >= 6) throw std::out_of_range("Invalid joint index");
    auto current = getJointsPositions();
    current[idx] = position;
    return sendJointsPosition(current);
}

bool KinovaArmController::sendJointsTrajectory(
    const std::vector<std::vector<double>>& trajectory)
{
    ensureInitialized();
    if (trajectory.empty()) {
        throw std::invalid_argument("Trajectory must contain at least one waypoint");
    }

    // prepare two kinds of points: one for position, one for delay
    TrajectoryPoint posPoint, delayPoint;
    posPoint.InitStruct();
    posPoint.Position.Type        = ANGULAR_POSITION;
    posPoint.LimitationsActive    = 0;        // no extra limits
    posPoint.SynchroType          = 1;        // synchronize joints

    delayPoint.InitStruct();
    delayPoint.Position.Type      = TIME_DELAY;
    delayPoint.Position.Delay     = 0.05f;
    delayPoint.LimitationsActive  = 0;
    delayPoint.SynchroType        = 0;

    // queue each waypoint + delay
    for (const auto& wp : trajectory) {
        // map from user space → hardware
        posPoint.Position.Actuators.Actuator1 = SIGN[0] * wp[0] + ZERO_OFFSETS[0];
        posPoint.Position.Actuators.Actuator2 = SIGN[1] * wp[1] + ZERO_OFFSETS[1];
        posPoint.Position.Actuators.Actuator3 = SIGN[2] * wp[2] + ZERO_OFFSETS[2];
        posPoint.Position.Actuators.Actuator4 = SIGN[3] * wp[3] + ZERO_OFFSETS[3];
        posPoint.Position.Actuators.Actuator5 = SIGN[4] * wp[4] + ZERO_OFFSETS[4];
        posPoint.Position.Actuators.Actuator6 = SIGN[5] * wp[5] + ZERO_OFFSETS[5];

        // enqueue position
        MySendAdvanceTrajectory(posPoint);
        // enqueue a small delay before the next waypoint
        MySendAdvanceTrajectory(delayPoint);
    }

    return true;
}

// ------- State queries -------
std::vector<double> KinovaArmController::getJointsPositions() {
    ensureInitialized();
    (*MyGetAngularCommand)(dataCommand);
    (*MyGetAngularPosition)(dataPosition);

    // Map hardware to user-space: user = SIGN * (hardware - ZERO_OFFSET)
    return {
        SIGN[0] * (dataPosition.Actuators.Actuator1 - ZERO_OFFSETS[0]),
        SIGN[1] * (dataPosition.Actuators.Actuator2 - ZERO_OFFSETS[1]),
        SIGN[2] * (dataPosition.Actuators.Actuator3 - ZERO_OFFSETS[2]),
        SIGN[3] * (dataPosition.Actuators.Actuator4 - ZERO_OFFSETS[3]),
        SIGN[4] * (dataPosition.Actuators.Actuator5 - ZERO_OFFSETS[4]),
        SIGN[5] * (dataPosition.Actuators.Actuator6 - ZERO_OFFSETS[5])
    };
}

std::vector<float> KinovaArmController::getJointsVelocities() {
    ensureInitialized();
    MyGetAngularVelocity(dataAngularPosition);

    // Flip signs back to user-space
    return {
        SIGN[0] * dataAngularPosition.Actuators.Actuator1,
        SIGN[1] * dataAngularPosition.Actuators.Actuator2,
        SIGN[2] * dataAngularPosition.Actuators.Actuator3,
        SIGN[3] * dataAngularPosition.Actuators.Actuator4,
        SIGN[4] * dataAngularPosition.Actuators.Actuator5,
        SIGN[5] * dataAngularPosition.Actuators.Actuator6
    };
}

std::vector<double> KinovaArmController::getJointsTorques() {
    ensureInitialized();
    MyGetAngularForce(dataAngularPosition);
    return {
        dataAngularPosition.Actuators.Actuator1,
        dataAngularPosition.Actuators.Actuator2,
        dataAngularPosition.Actuators.Actuator3,
        dataAngularPosition.Actuators.Actuator4,
        dataAngularPosition.Actuators.Actuator5,
        dataAngularPosition.Actuators.Actuator6
    };
}

std::vector<double> KinovaArmController::getJointsCurrents() {
    ensureInitialized();
    MyGetAngularCurrent(dataAngularPosition);
    return {
        dataAngularPosition.Actuators.Actuator1,
        dataAngularPosition.Actuators.Actuator2,
        dataAngularPosition.Actuators.Actuator3,
        dataAngularPosition.Actuators.Actuator4,
        dataAngularPosition.Actuators.Actuator5,
        dataAngularPosition.Actuators.Actuator6
    };
}

std::vector<float> KinovaArmController::getJointsTemperatures() {
    ensureInitialized();
    MyGetGeneralInformations(dataGeneral);
    return {
        dataGeneral.ActuatorsTemperatures[0],
        dataGeneral.ActuatorsTemperatures[1],
        dataGeneral.ActuatorsTemperatures[2],
        dataGeneral.ActuatorsTemperatures[3],
        dataGeneral.ActuatorsTemperatures[4],
        dataGeneral.ActuatorsTemperatures[5]
    };
}

// ------- Single joint getters -------
double KinovaArmController::getJointPosition(size_t idx) {
    auto pos = getJointsPositions();
    if (idx >= pos.size()) throw std::out_of_range("Invalid joint index");
    return pos[idx];
}

float KinovaArmController::getJointVelocity(size_t idx) {
    auto vels = getJointsVelocities();
    if (idx >= vels.size()) throw std::out_of_range("Invalid joint index");
    return vels[idx];
}

double KinovaArmController::getJointTorque(size_t idx) {
    auto torques = getJointsTorques();
    if (idx >= torques.size()) throw std::out_of_range("Invalid joint index");
    return torques[idx];
}

double KinovaArmController::getJointCurrent(size_t idx) {
    auto currents = getJointsCurrents();
    if (idx >= currents.size()) throw std::out_of_range("Invalid joint index");
    return currents[idx];
}

float KinovaArmController::getJointTemperature(size_t idx) {
    auto temps = getJointsTemperatures();
    if (idx >= temps.size()) throw std::out_of_range("Invalid joint index");
    return temps[idx];
}

// ------- Joint Names -------
std::vector<std::string> KinovaArmController::getJointNames() const {
    return {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
}

// ------- Control Modes -------
bool KinovaArmController::setForceControlMode(bool active) {
    ensureInitialized();
    if (active) {
        MyStartForceControl();
        return true;
    } else {
        MyStopForceControl();
        return false;
    }
}

// ------- API Health -------
bool KinovaArmController::isInitialized() const {
    return initialized_;
}

// ------- Internal API/Helpers -------
int KinovaArmController::loadSymbols() {
    std::cout << "KinovaArmController initializing.\n";
    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
    MyInitAPI                 = (int (*)()) dlsym(commandLayer_handle, "InitAPI");
    MyCloseAPI                = (int (*)()) dlsym(commandLayer_handle, "CloseAPI");
    MyGetDevices              = (int (*)(KinovaDevice[], int &)) dlsym(commandLayer_handle, "GetDevices");
    MySetActiveDevice         = (int (*)(KinovaDevice)) dlsym(commandLayer_handle, "SetActiveDevice");
    MySendBasicTrajectory     = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "SendBasicTrajectory");
    MyStartForceControl       = (int (*)()) dlsym(commandLayer_handle, "StartForceControl");
    MyStopForceControl        = (int (*)()) dlsym(commandLayer_handle, "StopForceControl");
    MyGetGeneralInformations  = (int (*)(GeneralInformations &)) dlsym(commandLayer_handle, "GetGeneralInformations");
    MyGetAngularCommand       = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
    MyGetAngularPosition      = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
    MyGetAngularVelocity      = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularVelocity");
    MyGetAngularForce         = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularForce");
    MySendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "SendAdvanceTrajectory");
    MyGetAngularForceGravityFree =
        (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularForceGravityFree");
    MyGetAngularCurrent       = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCurrent");
    MyGetCartesianCommand     = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianCommand");
    MyGetCartesianPosition    = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianPosition");
    MyGetActuatorAcceleration = (int (*)(AngularAcceleration &)) dlsym(commandLayer_handle, "GetActuatorAcceleration");

    std::cout << "Library Finished initializing.\n";

    if (!MyInitAPI || !MyCloseAPI || !MySendBasicTrajectory ||
        !MyGetDevices || !MySetActiveDevice || !MyGetAngularCommand) {
        throw std::runtime_error("* * *  ERROR DURING INITIALIZATION  * * *");
    }
    return MyInitAPI();
}

void KinovaArmController::initialize(int result) {
    KinovaDevice list[MAX_KINOVA_DEVICE];
    int devicesCount = MyGetDevices(list, result);

    for (int i = 0; i < devicesCount; i++) {
        MySetActiveDevice(list[i]);
    }

    std::cout << devicesCount << " devices found\n";

    if (devicesCount < 1) {
        throw std::runtime_error("No Kinova devices found");
    } else {
        for (int i = 0; i < devicesCount; i++) {
            MySetActiveDevice(list[i]);
            std::cout << list[i].DeviceID << " " << list[i].Model << " utilised\n";
        }
    }
    initialized_ = true;
}

void KinovaArmController::closeAPI() {
    if (MyCloseAPI() != 0) {
        std::cerr << "Warning: Kinova API CloseAPI failed" << std::endl;
    }
    initialized_ = false;
}

void KinovaArmController::ensureInitialized() const {
    if (!initialized_) {
        throw std::runtime_error("KinovaArmController not initialized");
    }
}
