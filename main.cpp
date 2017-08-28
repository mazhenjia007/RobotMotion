#include <iostream>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>

#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"
#include "KinovaMethod.h"

#define PI (3.14159265357)
#define sqr(x) ((x)*(x))

using namespace std;

int Display(CartesianPosition pNow){
  float X = pNow.Coordinates.X;
  float Y = pNow.Coordinates.Y;
  float Z = pNow.Coordinates.Z;
  float ThetaX = pNow.Coordinates.ThetaX;
  float ThetaY = pNow.Coordinates.ThetaY;
  float ThetaZ = pNow.Coordinates.ThetaZ;
  printf("X=%7.4f, Y=%7.4f, Z=%7.4f, ThetaX=%7.2f, ThetaY=%7.2f, ThetaZ=%7.2f",
	 X, Y, Z, ThetaX/PI*180, ThetaY/PI*180, ThetaZ/PI*180);
}

int main(){
  int result;

  TrajectoryPoint pSend;
  pSend.InitStruct();

  CartesianPosition pNow;
  
  if (LoadAPI()==1){
    result = MyInitAPI();
    KinovaDevice list[MAX_KINOVA_DEVICE];
    int deviceCount = MyGetDevices(list, result);

    if (deviceCount==0){
      cout << " *** No Availiable Device Found *** " << endl;
      return 0;
    }

    cout << "Device on USB (" << list[0].SerialNumber << ") Now Active" << endl;

    MySetActiveDevice(list[0]);

    cout << "Move HOME" << endl;
    MyMoveHome();

    cout << "Move to Point #1" << endl;
    pSend.Position.Type = CARTESIAN_POSITION;
    pSend.Position.CartesianPosition.X = 0.0;
    pSend.Position.CartesianPosition.Y = -0.3;
    pSend.Position.CartesianPosition.Z = 0.15;
    pSend.Position.CartesianPosition.ThetaX = PI/180.0*(180.0);
    pSend.Position.CartesianPosition.ThetaY = PI/180.0*(0.0);
    pSend.Position.CartesianPosition.ThetaZ = PI/180.0*(180.0);

    MySendBasicTrajectory(pSend);
    for (int i=0; i<40; i++){
      MyGetCartesianPosition(pNow);
      Display(pNow);
      cout << endl;
      usleep(100000);
    }

    cout << "Move to Point #2" << endl;
    pSend.Position.Type = CARTESIAN_POSITION;
    pSend.Position.CartesianPosition.X = 
  }

  return 0;
}
