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
int (*MyGetAngularPosition)(AngularPosition &);
int (*MyGetAngularCommand)(AngularPosition &);
int (*MyGetAngularVelocity)(AngularPosition &);
int (*MySetAngularControl)();
int (*MySetCartesianControl)();
int (*MyGetControlType)(int &);

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
  MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
  MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
  MyGetAngularVelocity = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularVelocity");
  MySetAngularControl = (int (*)()) dlsym(commandLayer_handle, "SetAngularControl");
  MySetCartesianControl = (int (*)()) dlsym(commandLayer_handle, "SetCartesianControl");
  MyGetControlType = (int (*)(int &)) dlsym(commandLayer_handle, "GetControlType");
  
  if (MyInitAPI==NULL || MyCloseAPI==NULL || MyMoveHome==NULL
      || MyInitFingers==NULL
      || MyGetDevices==NULL || MySetActiveDevice==NULL
      || MySendBasicTrajectory==NULL
      || MyGetCartesianCommand==NULL || MyGetCartesianPosition==NULL
      || MyGetAngularPosition==NULL || MyGetAngularCommand==NULL
      || MySetAngularControl==NULL || MySetCartesianControl==NULL
      || MyGetControlType==NULL){
    cout << " *** ERROR During Loading *** " << endl;
    return 0;
  }
  else{
    // cout << "Loading Complete" << endl;
    return 1;
  }
}

void SetAngularPoint(AngularInfo &pA,
		     double a1, double a2, double a3,
		     double a4, double a5, double a6){
  pA.Actuator1 = a1;
  pA.Actuator2 = a2;
  pA.Actuator3 = a3;
  pA.Actuator4 = a4;
  pA.Actuator5 = a5;
  pA.Actuator6 = a6;
}

void SetCartesianPoint(CartesianInfo &pC,
		       double x, double y, double z,
		       double tx, double ty, double tz){
  pC.X = x;
  pC.Y = y;
  pC.Z = z;
  pC.ThetaX = tx;
  pC.ThetaY = ty;
  pC.ThetaZ = tz;
}

AngularInfo AngularPlus(AngularInfo pA1, AngularInfo pA2){
  AngularInfo res;
  res.InitStruct();
  res.Actuator1 = pA1.Actuator1 + pA2.Actuator1;
  res.Actuator2 = pA1.Actuator2 + pA2.Actuator2;
  res.Actuator3 = pA1.Actuator3 + pA2.Actuator3;
  res.Actuator4 = pA1.Actuator4 + pA2.Actuator4;
  res.Actuator5 = pA1.Actuator5 + pA2.Actuator5;
  res.Actuator6 = pA1.Actuator6 + pA2.Actuator6;
  return res;
}

AngularInfo AngularMinus(AngularInfo pA1, AngularInfo pA2){
  AngularInfo res;
  res.InitStruct();
  res.Actuator1 = pA1.Actuator1 - pA2.Actuator1;
  res.Actuator2 = pA1.Actuator2 - pA2.Actuator2;
  res.Actuator3 = pA1.Actuator3 - pA2.Actuator3;
  res.Actuator4 = pA1.Actuator4 - pA2.Actuator4;
  res.Actuator5 = pA1.Actuator5 - pA2.Actuator5;
  res.Actuator6 = pA1.Actuator6 - pA2.Actuator6;
  return res;
}

#endif
