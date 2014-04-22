#ifndef TCOLORTRACKER_H
#define TCOLORTRACKER_H

#include "ofMain.h"
#include "ofxKinect.h"

#include "ofxBiquadFilter.h"

class TColorTracker
{
    public:

        ofPoint position;
        ofPoint anchor;
        float window;
        ofColor trackColor;
        ofxKinect *kinect;
        TColorTracker(ofPoint p);
        void bindKinect(ofxKinect*);
        int track();
        void setColor(ofColor c);
        void setPosition(ofPoint p);

        ofxBiquadFilterInstance filterZ;
        ofxBiquadFilterInstance filterX;
        ofxBiquadFilterInstance filterY;


    protected:
    private:
        int clientWidth;
        int clientHeight;
        int active;
        float max(float,float,float);

};

#endif // TCOLORTRACKER_H
