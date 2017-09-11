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

int Display_A(AngularPosition pA){
  float A1 = pA.Actuators.Actuator1;
  float A2 = pA.Actuators.Actuator2;
  float A3 = pA.Actuators.Actuator3;
  float A4 = pA.Actuators.Actuator4;
  float A5 = pA.Actuators.Actuator5;
  float A6 = pA.Actuators.Actuator6;

  printf("A1=%7.2f, A2=%7.2f, A3=%7.2f, A4=%7.2f, A5=%7.2f, A6=%7.2f",
	 A1, A2, A3, A4, A5, A6);
}

double abs(double x){
  return x>0.0 ? x : -x;
}

double MAXANGLE(AngularInfo pA){
  double maxa = 0;
  maxa = max(maxa, abs(pA.Actuator1));
  maxa = max(maxa, abs(pA.Actuator2));
  maxa = max(maxa, abs(pA.Actuator3));
  maxa = max(maxa, abs(pA.Actuator4));
  maxa = max(maxa, abs(pA.Actuator5));
  maxa = max(maxa, abs(pA.Actuator6));
  return maxa;
}

int main(){
  int result;

  TrajectoryPoint pSend;
  pSend.InitStruct();

  CartesianPosition pNow;
  pNow.InitStruct();
  
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
    MyInitFingers();
    
    MyGetCartesianPosition(pNow);
    pSend.Position.Type = CARTESIAN_POSITION;
    pSend.Position.CartesianPosition = pNow.Coordinates;
    pSend.Position.Fingers.Finger1 = 6000.0;
    pSend.Position.Fingers.Finger2 = 6000.0;
    MySendBasicTrajectory(pSend);
    usleep(500000);

    AngularPosition pA;
    pA.InitStruct();
    MyGetAngularPosition(pA);
    Display_A(pA);
    cout << endl;

    AngularInfo p1, p2;
    SetAngularPoint(p1, 350.0, 223.0, 73.0, 350.0, 35.0, 160.0);
    SetAngularPoint(p2, 383.0, 230.0, 94.0, 346.0, 52.5, 189.0);
    
    cout << "Move to Point #1" << endl;

    // Set the system to Angular control
    result = MySetAngularControl();

    int cType;
    MyGetControlType(cType);
    if (cType == CONTROL_TYPE_CARTESIAN){
      cout << "Control Type : CARTESIAN" << endl;
    }
    else if (cType == CONTROL_TYPE_ANGULAR){
      cout << "Control Type : ANGULAR" << endl;
    }
    else{
      cout << " *** ERROR in Control Type *** " << endl;
    }
    
    // cout << result << endl;
    
    pSend.Position.Type = ANGULAR_POSITION;
    pSend.Position.Actuators = p1;
    pSend.Position.Fingers.Finger1 = 0.0;
    pSend.Position.Fingers.Finger2 = 0.0;
    
    result = MySendBasicTrajectory(pSend);

    for (int i=0; i<30; i++){
      MyGetAngularPosition(pA);
      Display_A(pA);
      cout << endl;
      usleep(100000);
    }

    MyGetCartesianPosition(pNow);
    Display(pNow);
    cout << endl;
    
    cout << "Move to Point #2" << endl;

    // Set the system to Cartesian control
    result = MySetCartesianControl();

    // cout << result << endl;
    pSend.InitStruct();

    AngularInfo v1;
    v1 = AngularMinus(p2, p1);
    double maxa, coe;
    double maxw = 20.0;
    maxa = MAXANGLE(v1);
    coe = maxa/maxw;
    v1.Actuator1 /= coe;
    v1.Actuator2 /= coe;
    v1.Actuator3 /= coe;
    v1.Actuator4 /= coe;
    v1.Actuator5 /= coe;
    v1.Actuator6 /= coe;
    
    pSend.Position.Type = ANGULAR_VELOCITY;   
    pSend.Position.Actuators = v1;
    pSend.Position.Fingers.Finger1 = 0.0;
    pSend.Position.Fingers.Finger2 = 0.0;

    int nDis = 20;
    int iDis = 0;
    bool flag = true;

    while (flag){
      MyGetAngularPosition(pA);
      if (++iDis==nDis){
	Display_A(pA);
	cout << endl;
	iDis = 0;
      }

      AngularInfo dp;
      dp = AngularMinus(p2, pA.Actuators);
      maxa = MAXANGLE(dp);
      if (maxa < 0.3){
	flag = false;
      }
      else{
	coe = maxa/maxw;
	dp.Actuator1 /= coe;
	dp.Actuator2 /= coe;
	dp.Actuator3 /= coe;
	dp.Actuator4 /= coe;
	dp.Actuator5 /= coe;
	dp.Actuator6 /= coe;

	pSend.InitStruct();
	pSend.Position.Type = ANGULAR_VELOCITY;
	pSend.Position.Actuators = dp;
	pSend.Position.Fingers.Finger1 = 0.0;
	pSend.Position.Fingers.Finger2 = 0.0;
	
	result = MySendBasicTrajectory(pSend);
      }
      usleep(5000);
    }
    
    /*
    for (int i=0; i<20; i++){
      MyGetAngularPosition(pA);
      Display_A(pA);
      cout << endl;

      for (int j=0; j<100000/5000; j++){
	result = MySendBasicTrajectory(pSend);
	usleep(5000);
      }
    }
    */
    
    /*
    result = MySendBasicTrajectory(pSend);

    for (int i=0; i<20; i++){
      MyGetAngularPosition(pA);
      Display_A(pA);
      cout << endl;
      usleep(100000);
    }
    */

    MyGetAngularPosition(pA);
    Display_A(pA);
    cout << endl;

    MyGetCartesianPosition(pNow);
    Display(pNow);
    cout << endl;

  }

  return 0;
}
