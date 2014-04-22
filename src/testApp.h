#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

#include "TColorTracker.h"
#include "TFlightController.h"

#include "ofxBiquadFilter.h"


// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class serialHandler
{
    public:
    ofSerial port;
    unsigned char inptr;
    unsigned char indata[256];
    unsigned char outdata[256];
    int counter;

    bool active;

    serialHandler()
    {
        inptr=0;
        counter=0;
        active=false;
    }
    void open(string s)
    {
        active=port.setup(s,19200);
    }

    int update()
    {
        if(!active)
            return 0;
        int r=0;
        while(port.available())
        {
            unsigned char c=port.readByte();
            if(c==255)
            {
                inptr=0;
                counter++;
                r=1;
            }
            else
                indata[inptr++]=c;
        }
        return r;
    }
    void send(unsigned char n)
    {
        if(!active)
            return;

        for(int i=0;i<n;i++)
        {
            if(outdata[i]>254)
                port.writeByte(254);
            else
                port.writeByte(outdata[i]);
        }
        port.writeByte(255);
    }
};




class testApp : public ofBaseApp {
public:

	void setup();
	void update();
	void draw();
	void exit();

	void drawPointCloud();
	void viewScene();

	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

	ofxKinect kinect;
    ofLight pointLight;
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif

	ofxCvColorImage colorImg;

	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image

	ofxCvContourFinder contourFinder;

	bool bThreshWithOpenCV;
	bool bDrawPointCloud;

	int nearThreshold;
	int farThreshold;

	int angle;

    int camWidth;
    int camHeight;

	TColorTracker *redDot;
	TColorTracker *greenDot;

	// used for viewing the point cloud
	ofEasyCam easyCam;

    serialHandler controller;
    serialHandler mixer;

//	ofSerial handset;
//	ofSerial mixer;

/*
    unsigned char outbuf[16];
    unsigned char inbuf[16];
    unsigned char inptr;
*/

//    unsigned char manual;

    TFlightController drone0;


};


