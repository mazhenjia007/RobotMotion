#include <iostream>
#include <dlfcn.h>
#include <vector>
#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#define PI (3.14159265357)
#define sqr(x) ((x)*(x))

using namespace std;

int Display(CartesianPosition currentCommand){
  float X = currentCommand.Coordinates.X;
  float Y = currentCommand.Coordinates.Y;
  float Z = currentCommand.Coordinates.Z;
  float ThetaX = currentCommand.Coordinates.ThetaX;
  float ThetaY = currentCommand.Coordinates.ThetaY;
  float ThetaZ = currentCommand.Coordinates.ThetaZ;
  printf("X=%7.4f, Y=%7.4f, Z=%7.4f, ThetaX=%7.2f, ThetaY=%7.2f, ThetaZ=%7.2f",
	 X, Y, Z, ThetaX/PI*180, ThetaY/PI*180, ThetaZ/PI*180);
}

double FitAngle(double angle){
  double res = angle;
  if (res > PI){
    res = res - 2*PI;
  }
  if (res < -PI){
    res = res + 2*PI;
  }
  return res;
}

CartesianPosition Diff(CartesianPosition p1, CartesianPosition p2){
  CartesianPosition res;
  res.InitStruct();
  res.Coordinates.X = p1.Coordinates.X - p2.Coordinates.X;
  res.Coordinates.Y = p1.Coordinates.Y - p2.Coordinates.Y;
  res.Coordinates.Z = p1.Coordinates.Z - p2.Coordinates.Z;
  res.Coordinates.ThetaX = p1.Coordinates.ThetaX - p2.Coordinates.ThetaX;
  res.Coordinates.ThetaY = p1.Coordinates.ThetaY - p2.Coordinates.ThetaY;
  res.Coordinates.ThetaZ = p1.Coordinates.ThetaZ - p2.Coordinates.ThetaZ;
  res.Coordinates.ThetaX = FitAngle(res.Coordinates.ThetaX);
  res.Coordinates.ThetaY = FitAngle(res.Coordinates.ThetaY);
  res.Coordinates.ThetaZ = FitAngle(res.Coordinates.ThetaZ);
  return res;
}

int Judge(CartesianPosition diff){
  CartesianInfo dp = diff.Coordinates;
  double dis = sqrt(sqr(dp.X) + sqr(dp.Y) + sqr(dp.Z));
  if (dis > 1e-2) return 0;

  if (abs(dp.ThetaX) > PI/180.0*(1.0)) return 0;
  if (abs(dp.ThetaY) > PI/180.0*(1.0)) return 0;
  if (abs(dp.ThetaZ) > PI/180.0*(1.0)) return 0;
  return 1;
}

void FixVelocity(CartesianPosition v, TrajectoryPoint &pointToSend){
  CartesianInfo vv = v.Coordinates;

  if (abs(vv.X) < 5e-3) vv.X = 0;
  if (abs(vv.Y) < 5e-3) vv.Y = 0;
  if (abs(vv.Z) < 5e-3) vv.Z = 0;
  if (abs(vv.ThetaX) < PI/180.0*(1.0)) vv.ThetaX = 0;
  if (abs(vv.ThetaY) < PI/180.0*(1.0)) vv.ThetaY = 0;
  if (abs(vv.ThetaZ) < PI/180.0*(1.0)) vv.ThetaZ = 0;
  
  double vlen = sqrt(sqr(vv.X)+sqr(vv.Y)+sqr(vv.Z));
  double v_len = 0.1;

  if (vlen > 0){
    vv.X = vv.X / vlen * v_len;
    vv.Y = vv.Y / vlen * v_len;
    vv.Z = vv.Z / vlen * v_len;
  }

  double alen = sqrt(sqr(vv.ThetaX)+sqr(vv.ThetaY)+sqr(vv.ThetaZ));
  double a_len = PI/180.0*(120.0);

  vv.ThetaX = vv.ThetaX / alen * a_len;
  vv.ThetaY = vv.ThetaY / alen * a_len;
  vv.ThetaZ = vv.ThetaZ / alen * a_len;

  pointToSend.Position.CartesianPosition = vv;
}

int main(){
  int result;

  CartesianPosition currentCommand;
  TrajectoryPoint pointToSend;
  pointToSend.InitStruct();

  // Handle for the lib's command layer.
  void* commandLayer_handle;

  // Function pointers
  int (*MyInitAPI)();
  int (*MyCloseAPI)();
  int (*MyMoveHome)();
  int (*MyInitFingers)();
  int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
  int (*MySetActiveDevice)(KinovaDevice device);
  int (*MySendBasicTrajectory)(TrajectoryPoint command);
  int (*MyGetCartesianCommand)(CartesianPosition &);
  int (*MyGetCartesianPosition)(CartesianPosition &);
  
  cout << "Loading API" << endl;
  
  // Load the lib
  commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW|RTLD_GLOBAL);

  // Load the func
  MyInitAPI = (int (*)()) dlsym(commandLayer_handle, "InitAPI");
  MyCloseAPI = (int (*)()) dlsym(commandLayer_handle, "CloseAPI");
  MyMoveHome = (int (*)()) dlsym(commandLayer_handle, "MoveHome");
  MyInitFingers = (int (*)()) dlsym(commandLayer_handle, "InitFingers");
  MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
  MySetActiveDevice = (int (*)(KinovaDevice device)) dlsym(commandLayer_handle, "SetActiveDevice");
  MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "SendBasicTrajectory");
  MyGetCartesianCommand = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianCommand");
  MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianPosition");
  
  if (MyInitAPI==NULL || MyCloseAPI==NULL || MyMoveHome==NULL || MyInitFingers==NULL) {
    cout << " *** ERROR During Initialization *** " << endl;
  }
  else{
    cout << "Initialization Completed" << endl;

    result = (*MyInitAPI)();

    KinovaDevice list[MAX_KINOVA_DEVICE];
    int deviceCount = MyGetDevices(list, result);

    if (deviceCount==0){
      cout << " *** No Availiable Device Found *** " << endl;
    }
    else{
      cout << "Device on USB (" << list[0].SerialNumber << ") Now Active" << endl;
      
      // Always use the 1st device
      MySetActiveDevice(list[0]);

      cout << "Move HOME" << endl;
      MyMoveHome();

      cout << "Initializing the fingers" << endl;
      MyInitFingers();

      cout << "Command : ";
      MyGetCartesianCommand(currentCommand);
      Display(currentCommand);
      cout << endl;
      
      cout << "Position: ";
      MyGetCartesianPosition(currentCommand);
      Display(currentCommand);
      cout << endl;

      cout << "Move to Point #1" << endl;

      
      pointToSend.Position.Type = CARTESIAN_POSITION;
      // MyGetCartesianCommand(currentCommand);
      // pointToSend.Position.CartesianPosition = currentCommand.Coordinates;
      pointToSend.Position.CartesianPosition.X = 0.0;
      pointToSend.Position.CartesianPosition.Y = -0.3;
      pointToSend.Position.CartesianPosition.Z = 0.15;
      pointToSend.Position.CartesianPosition.ThetaX = PI / 180.0 * (180.0);
      pointToSend.Position.CartesianPosition.ThetaY = PI / 180.0 * (0.0);
      pointToSend.Position.CartesianPosition.ThetaZ = PI / 180.0 * (180.0);
      
      MySendBasicTrajectory(pointToSend);
      for (int i=0; i<50; i++){
	MyGetCartesianPosition(currentCommand);
	Display(currentCommand);
	cout << endl;
	usleep(100000);
      }
            
      CartesianPosition p0, p1, p2, p_now, p_diff;
      MyGetCartesianPosition(p0);
      p1.Coordinates.X = 0.0;
      p1.Coordinates.Y = -0.3;
      p1.Coordinates.Z = 0.15;
      p1.Coordinates.ThetaX = PI / 180.0 * (0.0);
      p1.Coordinates.ThetaY = PI / 180.0 * (60.0);
      p1.Coordinates.ThetaZ = PI / 180.0 * (0.0);
            
      cout << "Move to Point #2" << endl;

      int nDisp = 20;
      int iDisp = 0;
      int flag = 1;

      pointToSend.Position.Type = CARTESIAN_VELOCITY;
      while (flag){
	if (iDisp==nDisp){
	  MyGetCartesianPosition(currentCommand);
	  Display(currentCommand);
	  cout << endl;
	  iDisp = 0;
	}
	else{
	  iDisp++;
	}

	MyGetCartesianPosition(p_now);

	p_diff = Diff(p1, p_now);

	if (Judge(p_diff)){
	  flag = 0;
	  Display(p_now);
	  cout << endl;
	}
	else{
	  FixVelocity(p_now, pointToSend);
	  MySendBasicTrajectory(pointToSend);
	}
	
	usleep(5000);
      }
      
      /*
      pointToSend.Position.Type = CARTESIAN_VELOCITY;
      pointToSend.Position.CartesianPosition.X = 0;
      pointToSend.Position.CartesianPosition.Y = 0;
      pointToSend.Position.CartesianPosition.Z = 0;
      pointToSend.Position.CartesianPosition.ThetaX = PI / 180.0 * (0.0);
      pointToSend.Position.CartesianPosition.ThetaY = PI / 180.0 * (60.0);
      pointToSend.Position.CartesianPosition.ThetaZ = PI / 180.0 * (0.0);

      int nDisp = 20;
      int iDisp = 0;
      for (int i=0; i<200; i++){
	if (iDisp==nDisp){
	  MyGetCartesianPosition(currentCommand);
	  Display(currentCommand);
	  cout << endl;
	  iDisp = 0;
	}
	else{
	  iDisp++;
	}
	
	MySendBasicTrajectory(pointToSend);
	usleep(5000);
      }
      */
      
      /*
      int flag = 1;
      int nDisp = 50;
      int iDisp = 0;
      float v = 0.15;
      float w = 1.5;
      for (int i=0; i<2500 && flag; i++){
	MyGetCartesianCommand(p_now);
      }
      */
      cout << "Finished" << endl;
    }

    result = (*MyCloseAPI)();
    cout << "Close API" << endl;
  }
  dlclose(commandLayer_handle);
}
