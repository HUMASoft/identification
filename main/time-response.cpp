#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "SocketCanPort.h"

#include "math.h"

#include "SerialArduino.h"

#include "fcontrol.h"
#include "IPlot.h"
#include "OnlineSystemIdentification.h"

#include "Kinematics.h"


// Demo Close loop with Inclination Sensor, steps 20º incl - 0..45º orientation.
// It requires: -Platform inclination=0
//              -Reset IMU sensor


int main ()
{

  ofstream data("../ids.csv",std::ofstream::out);

  //Samplinfg time
  double dts=0.001;
  SamplingTime Ts(dts);


  //m1 setup
  SocketCanPort pm31("can1");
  CiA402SetupData sd31(2048,24,0.001, 0.144);
  CiA402Device m1 (31, &pm31, &sd31);
  m1.Reset();
  m1.SwitchOn();
//    m1.SetupPositionMode(10,10);
  m1.Setup_Velocity_Mode(10);
//  m1.Setup_Torque_Mode();

  //m2
  SocketCanPort pm2("can1");
  CiA402SetupData sd32(2048,24,0.001, 0.144);
  CiA402Device m2 (32, &pm2, &sd32);
  m2.Reset();
  m2.SwitchOn();
//    m2.SetupPositionMode(10,10);
  m2.Setup_Velocity_Mode(10);
//  m2.Setup_Torque_Mode();


  //m3
  SocketCanPort pm3("can1");
  CiA402SetupData sd33(2048,24,0.001, 0.144);
  CiA402Device m3 (33, &pm3, &sd33);
  m3.Reset();
  m3.SwitchOn();
//    m3.SetupPositionMode(10,10);
  m3.Setup_Velocity_Mode(10);
//  m3.Setup_Torque_Mode();




  IPlot id;

  double vel, curvel;

  vel=-5;
  m1.SetVelocity(vel);

  //Initialize identification
  double interval=1; //in seconds
  for (double t=0;t<interval; t+=dts)
  {

      curvel = m1.GetVelocity();


      cout << "curvel " << curvel ;
      data << t << "," << vel << "," << curvel << endl;


      Ts.WaitSamplingTime();
  }

  m1.SetVelocity(0);
  m2.SetVelocity(0);
  m3.SetVelocity(0);
//  sleep (4);
  data.close();



return 0;

}

