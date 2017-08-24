#include <iostream>
#include <dlfcn.h>
#include <vector>
#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"
#include <stdio.h>
#include <unistd.h>

using namespace std;

int main(){
  int result;

  CartesianPosition currentCommand;

  // Handle for the lib's command layer.
  void* commandLayer_handle;

  // Function pointers
  int (*MyInitAPI)();
  int (*MyCloseAPI)();
  int (*MySendBasicTrajectory)(TrajectoryPoint command);
  int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
  int (*MySetActiveDevice)(KinovaDevice device);
  int (*MyMoveHome)();
  int (*MyInitFingers)();
  int (*MyGetQuickStatus)(QuiskStatus &);
  int (*MyGetCartesianCommand)(CartesianPosition &);

  // Load the lib
  commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW|RTLD_GLOBAL);

  // Load the func
  MyInitAPI = (int (*)()) dlsym(commandLayer_handle, "InitAPI");
  MyCLoseAPI = ()()
}
