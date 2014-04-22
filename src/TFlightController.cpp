#include <math.h>
#include "TFlightController.h"
//*************************************************************
//  PID regulator class
//*************************************************************

TPIDRegulator::TPIDRegulator()
{
    P_GAIN=0;
    I_GAIN=0;
    D_GAIN=0;
    GAIN=0;
    I_LIM=0;
    setp=0;
    E0=0;
    I=0;
    D=0;
}
float TPIDRegulator::update(float v)
{
    float error=(v-setp)*GAIN;
    D=D*0.5+(error-E0)*0.5;
//    D=D*0.8+(error-E0)*0.2;
    E0=error;
    I+=error*I_GAIN;

    if(I>I_LIM)
        I=I_LIM;
    if(I<-I_LIM)
        I=-I_LIM;

    float r=error*P_GAIN+I+D*D_GAIN;

    return r;
}
void TPIDRegulator::setpoint(float s)
{
    setp=s;
}
void TPIDRegulator::setup(float p,float i,float d,float ilim,float g)
{
    P_GAIN=p;
    I_GAIN=i;
    D_GAIN=d;
    I_LIM=ilim;
    GAIN=g;
}
void TPIDRegulator::setP(float p){P_GAIN=p;};
void TPIDRegulator::setI(float i){I_GAIN=i;};
void TPIDRegulator::setD(float d){D_GAIN=d;};
void TPIDRegulator::setGain(float g){GAIN=g;};
void TPIDRegulator::setIlim(float i){I_LIM=i;};
void TPIDRegulator::reset(float i)
{
    I=i;
}

//*************************************************************
//  Flight controller class
//*************************************************************

 TFlightController::TFlightController()
{
    cmdX=0;
    cmdY=0;
    cmdZ=0;
    cmdR=0;

    mode=PILOT_PITCH|PILOT_ROLL|PILOT_THROTTLE|PILOT_YAW;

    throttlePID.setup(   0.0570,   0.0002, 1.455,255.0, 1.0);
    pitchPID.setup   (   0.0570,  0.0002, 1.455,255.0, 1.0);
    rollPID.setup    (   0.0570,  0.0002, 1.455,255.0, 1.0);
    yawPID.setup     (   0.0570,  0.0002, 1.455,255, 200.0);

/*    throttlePID.setup(   0.0165,   0.0002,   2.0,255.0, 1.0);
    pitchPID.setup   (   0.0570,  0.00051, 1.260,255.0, 1.0);
    rollPID.setup    (   0.0570,  0.00051, 1.260,255.0, 1.0);
    yawPID.setup     (   0.1,  0.003, 10.0,  200, 1.0);
*/
/*    throttlePID.setup(0.0165, 0.0002,  2.0,200.0, 1.0);
    pitchPID.setup   (   0.0885,  0.00122, 2.677,  200, 1.0);
    rollPID.setup    (   0.0885,  0.00122, 2.677,  200, 1.0);
    yawPID.setup     (   0.1,  0.003, 10.0,  200, 1.0);
*/
/*    throttlePID.setup(0.05, 0.0015,  4.0,200.0, 1.0);
    pitchPID.setup   ( 0.2,  0.003, 15.0,  200, 0.25);
    rollPID.setup    ( 0.2,  0.003, 15.0,  200, 0.25);
    yawPID.setup     ( 0.1,  0.003, 10.0,  200, 1.0);
*/
/*
    throttlePID.setup(0.05, 0.0015,  4.0,200.0, 1.0);
    pitchPID.setup   ( 0.1,  0.003, 10.0,  200, 0.8);
    rollPID.setup    ( 0.1,  0.003, 10.0,  200, 0.8);
    yawPID.setup     ( 0.1,  0.003, 10.0,  200, 1.0);
*/

    pitchOut=128;
    rollOut=128;
    yawOut=128;
    throttleOut=0;

    droneData[0]=0;
    droneData[1]=128;
    droneData[2]=64;
    droneData[3]=128;
    droneData[4]=128;
    droneData[5]=64;
    droneData[6]=64;
    droneData[7]=0;
}

void TFlightController::flyByWire(float p,float r, float y,float t)
{
    if(mode&PILOT_PITCH)
        pitchOut=p;
    if(mode&PILOT_ROLL)
        rollOut=r;
    if(mode&PILOT_YAW)
        yawOut=y;
    if(mode&PILOT_THROTTLE)
        throttleOut=t;
    prepareData();
}
void TFlightController::update(float x,float y,float z, float heading)
{
 //   x-=cmdX;
//    y-=cmdY;

    float tx=cos(heading)*x-sin(heading)*y;
    float ty=sin(heading)*x+cos(heading)*y;

    heading-=cmdR;
    if(heading<-M_PI)
        heading+=M_PI;
    if(heading>M_PI)
        heading-=M_PI;

    yawPID.setpoint(0);

    if(mode&PILOT_PITCH)
        pitchPID.update(ty);
    else
        pitchOut=pitchPID.update(ty);

    if(mode&PILOT_ROLL)
        rollPID.update(tx);
    else
        rollOut=rollPID.update(tx);

    if(mode&PILOT_YAW)
        yawPID.update(heading);
    else
        yawOut=yawPID.update(-heading);

    if(mode&PILOT_THROTTLE)
        throttlePID.update(z);
    else
        throttleOut=throttlePID.update(z);

    prepareData();
}

void TFlightController::setMode(int m)
{
    if(mode&PILOT_THROTTLE)
        if(!(m&PILOT_THROTTLE))
            throttlePID.reset(throttleOut);

    if(mode&PILOT_PITCH)
        if(!(m&PILOT_PITCH))
            pitchPID.reset(128);
//            pitchPID.reset(pitchOut);

    if(mode&PILOT_ROLL)
        if(!(m&PILOT_ROLL))
            rollPID.reset(128);
//            rollPID.reset(rollOut);

    if(mode&PILOT_YAW)
        if(!(m&PILOT_YAW))
            yawPID.reset(128);
//            yawPID.reset(yawOut);


    mode=m;
}

void TFlightController::command(float x, float y,float z, float heading)
{
    cmdX=x;
    cmdY=y;
    throttlePID.setpoint(z);
    cmdZ=z;
    cmdR=heading;

/*    pitchPID.setpoint(y);
    rollPID.setpoint(x);
    yawPID.setpoint(heading);
    throttlePID.setpoint(z);
*/
}

void TFlightController::prepareData()
{
    droneData[0]=dlimit(throttleOut);
    droneData[1]=dlimit(yawOut);
    droneData[3]=dlimit(pitchOut);
    droneData[4]=dlimit(rollOut);
}

unsigned char TFlightController::dlimit(float f)
{
    if(f>254)
        return 254;
    if(f<0)
        return 0;
    return f;
}

