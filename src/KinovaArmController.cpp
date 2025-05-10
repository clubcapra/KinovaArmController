#include "arm_controller/KinovaArmController.h"
#include <iostream>
#include <dlfcn.h>
#include <unistd.h>
#include <string.h>

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


bool KinovaArmController::sendJointVelocity(const std::vector<double>& velocities) {
    bool success = true;
    ensureInitialized();
    if (velocities.size() != 6) {
        success = false;
        throw std::invalid_argument("Expected 6 joint velocities");
    }

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

    return success;
}


void KinovaArmController::sendJointPosition(const std::vector<double> &positions) {
    ensureInitialized();
    if (positions.size() != 6) {
        throw std::invalid_argument("Expected 6 joint positions");
    }

    AngularPosition current;
    if (MyGetAngularCommand(current) != 0) {
        throw std::runtime_error("Failed to get current joint positions");
    }
    TrajectoryPoint point;
    point.InitStruct();
    point.Position.Type = ANGULAR_POSITION;

    point.Position.Actuators.Actuator1 = positions[0];
    point.Position.Actuators.Actuator2 = positions[1];
    point.Position.Actuators.Actuator3 = positions[2];
    point.Position.Actuators.Actuator4 = positions[3];
    point.Position.Actuators.Actuator5 = positions[4];
    point.Position.Actuators.Actuator6 = positions[5];
    if (MySendBasicTrajectory(point) != 0) {
        throw std::runtime_error("Failed to send joint position command");
    }
}

std::vector<double> KinovaArmController::getCurrentJointPositions() {
    ensureInitialized();
    AngularPosition current;

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

int KinovaArmController::loadSymbols() {
    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");

	MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
	MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
	MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");

	MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");
    MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
	MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) dlsym(commandLayer_handle, "GetActuatorAcceleration");
	MyGetAngularVelocity = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularVelocity");

	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetAngularCommand == NULL))

	{
        throw std::runtime_error("* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *");
	}
	else
	{
		return (*MyInitAPI)();
    }
}

void KinovaArmController::initialize(int result) {

    AngularPosition currentCommand;
    KinovaDevice list[MAX_KINOVA_DEVICE];
    int devicesCount = MyGetDevices(list, result);

    for (int i = 0; i < devicesCount; i++)
	{
        MySetActiveDevice(list[i]);
    }

    if (devicesCount < 1) {
        throw std::runtime_error("No Kinova devices found");
    }else{
        for (int i = 0; i < devicesCount; i++)
        {
            MySetActiveDevice(list[i]);
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
