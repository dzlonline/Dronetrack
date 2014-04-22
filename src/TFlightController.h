#ifndef TFLIGHTCONTROLLER_H
#define TFLIGHTCONTROLLER_H

//*************************************************************
//  PID regulator class
//*************************************************************

class TPIDRegulator
{
public:
    float P_GAIN;
    float I_GAIN;
    float D_GAIN;
    float I_LIM;
    float GAIN;
    float setp;
    TPIDRegulator();
    float update(float v);
    void setpoint(float);
    void setup(float,float,float,float,float);
    void setP(float);
    void setI(float);
    void setD(float);
    void setIlim(float);
    void setGain(float);
    void reset(float);
private:
    float I;
    float D;
    float E0;

 };

//*************************************************************
//  Flight controller class
//*************************************************************

#define PILOT_ALL_MANUAL 15
#define PILOT_ALL_AUTO 0
#define PILOT_PITCH 1
#define PILOT_ROLL 2
#define PILOT_YAW 4
#define PILOT_THROTTLE 8

class TFlightController
{
    public:
        float cmdX;
        float cmdY;
        float cmdZ;
        float cmdR;

        TPIDRegulator throttlePID;
        TPIDRegulator pitchPID;
        TPIDRegulator rollPID;
        TPIDRegulator yawPID;
        unsigned char droneData[8];
        int mode;

        float pitchOut;
        float rollOut;
        float yawOut;
        float throttleOut;

        TFlightController();
        void flyByWire(float p,float r, float y,float t);
        void update(float tx,float ty,float tz,float heading);
        void setMode(int m);
        void command(float x, float y,float z, float heading);


    protected:
    private:
    void prepareData();
    unsigned char dlimit(float f);

};

#endif // TFLIGHTCONTROLLER_H
