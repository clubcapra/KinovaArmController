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
    bool sendJointVelocity(const std::vector<double>& velocities);
    void sendJointPosition(const std::vector<double>& positions);
    std::vector<double> getCurrentJointPositions();

private:
    int loadSymbols();
    void initialize(int result);
    void closeAPI();
    void ensureInitialized() const;

    AngularPosition dataCommand;
	AngularPosition dataPosition;
    void * commandLayer_handle;
    bool initialized_;

    int(*MyInitAPI)();
    int(*MyCloseAPI)();
    int(*MySendBasicTrajectory)(TrajectoryPoint command);
    int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
    int(*MySetActiveDevice)(KinovaDevice device);
    int(*MyInitFingers)();
    int(*MyGetAngularCommand)(AngularPosition &);
    int(*MyGetAngularPosition)(AngularPosition &);
    int (*MyGetActuatorAcceleration)(AngularAcceleration &Response);
    int (*MyGetAngularVelocity)(AngularPosition &Response);
};
