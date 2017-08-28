#ifndef KINOVA_METHOD_H_
#define KINOVA_METHOD_H_

#include <iostream>
#include <dlfcn.h>
#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"
#include <stdio.h>

using namespace std;

void* commandLayer_handle;

int (*MyInitAPI)();
int (*MyCloseAPI)();
int (*MyMoveHome)();
int (*MyInitFingers)();
int (*MyGetDevices)(KinovaDevice[MAX_KINOVA_DEVICE], int &);
int (*MySetActiveDevice)(KinovaDevice);
int (*MySendBasicTrajectory)(TrajectoryPoint);
int (*MyGetCartesianCommand)(CartesianPosition &);
int (*MyGetCartesianPosition)(CartesianPosition &);

int LoadAPI(){
  cout << "Loading API" << endl;
  // Load the lib
  commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW|RTLD_GLOBAL);

  // Load the func
  MyInitAPI = (int (*)()) dlsym(commandLayer_handle, "InitAPI");
  MyCloseAPI = (int (*)()) dlsym(commandLayer_handle, "CloseAPI");
  MyMoveHome = (int (*)()) dlsym(commandLayer_handle, "MoveHome");
  MyInitFingers = (int (*)()) dlsym(commandLayer_handle, "InitFingers");
  MyGetDevices = (int (*)(KinovaDevice[MAX_KINOVA_DEVICE], int &)) dlsym(commandLayer_handle, "GetDevices");
  MySetActiveDevice = (int (*)(KinovaDevice)) dlsym(commandLayer_handle, "SetActiveDevice");
  MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "SendBasicTrajectory");
  MyGetCartesianCommand = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianCommand");
  MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianPosition");

  if (MyInitAPI==NULL || MyCloseAPI==NULL || MyMoveHome==NULL
      || MyInitFingers==NULL
      || MyGetDevices==NULL || MySetActiveDevice==NULL
      || MySendBasicTrajectory==NULL
      || MyGetCartesianCommand==NULL || MyGetCartesianPosition==NULL){
    cout << " *** ERROR During Loading *** " << endl;
    return 0;
  }
  else{
    cout << "Loading Complete" << endl;
    return 1;
  }
}

#endif
