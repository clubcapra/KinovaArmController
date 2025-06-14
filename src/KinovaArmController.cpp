#include "arm_controller/KinovaArmController.h"
#include <iostream>
#include <dlfcn.h>
#include <unistd.h>
#include <string.h>
#include <stdexcept>

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
    pointToSend.Position.Actuators.Actuator1 = velocities[0];
    pointToSend.Position.Actuators.Actuator2 = velocities[1];
    pointToSend.Position.Actuators.Actuator3 = velocities[2];
    pointToSend.Position.Actuators.Actuator4 = velocities[3];
    pointToSend.Position.Actuators.Actuator5 = velocities[4];
    pointToSend.Position.Actuators.Actuator6 = velocities[5]; 

    for (int i = 0; i < 100; i++)
    {
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
    point.Position.Actuators.Actuator1 = positions[0];
    point.Position.Actuators.Actuator2 = positions[1];
    point.Position.Actuators.Actuator3 = positions[2];
    point.Position.Actuators.Actuator4 = positions[3];
    point.Position.Actuators.Actuator5 = positions[4];
    point.Position.Actuators.Actuator6 = positions[5];
    for (int i = 0; i < 100; i++)
    {
        MySendBasicTrajectory(point);
        usleep(5000);
    }

    return true;
}

bool KinovaArmController::sendToHome() {
    sendJointsPosition({JOINT_0_ZERO,JOINT_1_ZERO, JOINT_2_ZERO, JOINT_3_ZERO, JOINT_4_ZERO, JOINT_5_ZERO});
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

// ------- State queries -------
std::vector<double> KinovaArmController::getJointsPositions() {
    ensureInitialized();
    (*MyGetAngularCommand)(dataCommand);
    (*MyGetAngularPosition)(dataPosition);
    return {
        dataPosition.Actuators.Actuator1,
        dataPosition.Actuators.Actuator2,
        dataPosition.Actuators.Actuator3,
        dataPosition.Actuators.Actuator4,
        dataPosition.Actuators.Actuator5,
        dataPosition.Actuators.Actuator6
    };
}

std::vector<float> KinovaArmController::getJointsVelocities() {
    ensureInitialized();
    MyGetAngularVelocity(dataAngularPosition);
    return {
        dataAngularPosition.Actuators.Actuator1,
        dataAngularPosition.Actuators.Actuator2,
        dataAngularPosition.Actuators.Actuator3,
        dataAngularPosition.Actuators.Actuator4,
        dataAngularPosition.Actuators.Actuator5,
        dataAngularPosition.Actuators.Actuator6
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
        dataAngularPosition.Actuators.Actuator6,

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
    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
    MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
    MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");

    MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
    MySetActiveDevice = (int (*)(KinovaDevice device)) dlsym(commandLayer_handle,"SetActiveDevice");
    MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
    MyStartForceControl = (int (*)()) dlsym(commandLayer_handle,"StartForceControl");
    MyGetGeneralInformations = (int (*)(GeneralInformations &info)) dlsym(commandLayer_handle,"GetGeneralInformations");
    MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");
    MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
    MyGetAngularVelocity = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularVelocity");
    MyGetAngularForce = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularForce");
    MyGetAngularForceGravityFree = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularForceGravityFree");
    MyGetAngularCurrent = (int(*)(AngularPosition &Response)) dlsym(commandLayer_handle, "GetAngularCurrent");
    MyGetCartesianCommand = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianCommand");
    MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianPosition");
    MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) dlsym(commandLayer_handle, "GetActuatorAcceleration");

    std::cout << "Library Finished initializing.\n";

    if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
         (MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetAngularCommand == NULL))

    {
        throw std::runtime_error("* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *");
    }
    else {
        return (*MyInitAPI)();
    }
}

void KinovaArmController::initialize(int result) {
    KinovaDevice list[MAX_KINOVA_DEVICE];
    int devicesCount = MyGetDevices(list, result);

    for (int i = 0; i < devicesCount; i++)
    {
        MySetActiveDevice(list[i]);
    }

    std::cout << devicesCount << + " devices found\n";

    if (devicesCount < 1) {
        throw std::runtime_error("No Kinova devices found");
    }else{
        for (int i = 0; i < devicesCount; i++)
        {
            MySetActiveDevice(list[i]);
            std::cout << list[i].DeviceID << + " " << list[i].Model << " utilised\n";
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
