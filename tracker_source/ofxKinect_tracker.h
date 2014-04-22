#ifndef _OPENNI_TRACKER_H
#define _OPENNI_TRACKER_H

#include "ofxFBOTexture.h"
//#include "ofxKinect.h"
#include "ofxOpenNI.h"

#define v_scale 0.25
//#define v_scale 1.0

extern float TRotX;
extern float TRotY;
extern float TDist;
extern float THeight;
extern float TSize;
extern float TLeft;
extern float TUp;

//***************************************************************************************
//
//  Zone object
//
//***************************************************************************************

#define ZONE_OUTPUT 1 //Output zone data for debugging

class TZone
{
private:
    //Size of sensor depth map
    int clientWidth;
    int clientHeight;
public:

    enum ZONESHAPES
    {
        RECTANGULAR,
        CIRCULAR
    };



    //Size of region being tracked
    int height;
    int width;
    //Position of region being tracked
    float x;
    float y;
    float signal;
    //Centroid of pixels if any
    float vx;
    float vy;
    float apex;
    //Top and bottom boundaries
    float top;
    float bottom;

    int shape;

    //Number of pixels in zone

    ofxDepthGenerator *depth;

    TZone(ofxDepthGenerator *d,int s)
    {
        depth=d;
        clientWidth=d->getWidth();
        clientHeight=d->getHeight();
        height=40;
        width=40;
        x=clientWidth/2;
        y=clientHeight/2;
        vx=x;
        vy=y;
        top=500;
        bottom=3800;

        shape=s;
        apex=bottom;
    }

    unsigned long update()
    {
        //-Local tracking variables
        float apx=0;
        float x0=x-(width/2);
        float y0=y-(height/2);

        vx=0;
        vy=0;
        signal=0;


#ifdef ZONE_OUTPUT

        ofSetColor(255,255,0);
        glBegin(GL_POINTS);
#endif

        long n=0;

        for(int py=y0;py<(y0+height)-1;py++)
        {
            for(int px=x0;px<(x0+width)-1;px++)
            {
                ofPoint cur;
                cur.x=px;
                cur.y=py;

                //-Check boundaries
                if((px>=0)&&(px<clientWidth)&&(py>=0)&&py<clientHeight)
                    cur.z = depth->getPixelDepth(px, py);
                else
                    cur.z=0;

				float g=0;

                if((cur.z>top)&&(cur.z<bottom))
                {

                    bool match=false;
                    switch(shape)
                    {
                        case TZone::CIRCULAR:
                        {
                            if(ofDist(x,y,px,py)<width/2)
                                match=true;
                        };break;
                        case TZone::RECTANGULAR:
                        default:
                        {
                            match=true;
                        };
                    }

                    if(match)
                    {
                        apx+=cur.z;
                        g=255.0;
                        n++;
#ifdef ZONE_OUTPUT
                        glVertex3f(cur.x-clientWidth/2, cur.y-clientHeight/2,cur.z*v_scale);
#endif
                    }
                    vx+=(float)px*g;
                    vy+=(float)py*g;
                    signal+=g;

				}

            }

        }

        if(signal>0.0)
        {
            vx/=signal;
            vy/=signal;
        }
        else
        {
            vx=x;
            vy=y;
        }

        if(n>0)
            apx/=(float)n;
        else
            apx=top;

        apex=apex*0.9+apx*0.1;

#ifdef ZONE_OUTPUT
        glEnd();

    ofSetColor(255,255,255);

    ofPushMatrix();
    ofTranslate(0,0,apex*v_scale);
    ofEllipse(x-clientWidth/2,y-clientHeight/2,width,height);
    ofPopMatrix();


    if(signal>100)
        ofSetColor(0,255,0);
    else
        ofSetColor(255,0,0);
       ofPushMatrix();
    ofTranslate(0,0,bottom*v_scale);
    ofEllipse(x-clientWidth/2,y-clientHeight/2,width,height);
    ofPopMatrix();






#endif
        return n;
    }
};

#define N_ZONES 8

class TZoneGesture
{
private:
    int size;
public:
    int signals[N_ZONES];
    bool active[N_ZONES];
    float angles[N_ZONES];
    float elevations[N_ZONES];

    TZoneGesture()
    {
        size=N_ZONES;
        for(int i=0;i<N_ZONES;i++)
        {
            signals[i]=0;
            active[i]=false;
            angles[i]=0;
            elevations[i]=0.5;
        }
    }
    int Size()
    {
        return size;
    }
};


//***************************************************************************************
//
//  Zone based tracker
//
//***************************************************************************************

class TOpenNITracker
{
private:

    ofxUserGenerator *user;
    ofxDepthGenerator *depth;

	int minsize;                    //-Track region boundries
	int maxsize;
	float target_filter;            //-Tracker parameters
	float gain;
	int clientWidth;                //-Size of raw tracker field
    int clientHeight;

    TZone *trackZone;               //-Zone used for position tracking

    float rooflevel;
    float floorlevel;

#define GESTURE_RADIUS 60
#define GESTURE_SIZE 60

    TZone *gestureZones[N_ZONES];

public:

    ofxFBOTexture *dataView;
	float x;                        //-Real world tracker position center 0,0 all in mm.
    float y;
	float apex;


	int width;                      //-Track region
    int height;


    TZoneGesture gesture;
//	int gesture[N_ZONES];                    //-Target values
	float elevation;
    float azimuth;
	float radius;
	float offset;
    unsigned char active;

    ~TOpenNITracker()
    {
        delete trackZone;
        for(int i=0;i<N_ZONES;i++)
            delete gestureZones[i];
    }
    TOpenNITracker(ofxUserGenerator *u,ofxDepthGenerator *d)
    {
        user=u;
        depth=d;

        trackZone=new TZone(d,TZone::CIRCULAR);
//        trackZone=new TZone(d,TZone::RECTANGULAR);

        x=trackZone->x;
        y=trackZone->y;
        trackZone->width=100;
        trackZone->height=100;
//        width=100;
//        height=100;
     trackZone->top=rooflevel=500;
     trackZone->bottom=floorlevel=2600;
//     trackZone->bottom=floorlevel=3000;

        for(int i=0;i<N_ZONES;i++)
        {
            gestureZones[i]=new TZone(d,TZone::CIRCULAR);
            gestureZones[i]->top=rooflevel;
            gestureZones[i]->bottom=floorlevel;
            gestureZones[i]->width=GESTURE_SIZE;
            gestureZones[i]->height=GESTURE_SIZE;
        }


    apex=floorlevel;

//		gesture=0;
		elevation=0.4;
		azimuth=0;
		radius=0;
		offset=0;
        active=0;

		maxsize=300;
		minsize=30;
		target_filter=width;
		gain=0.00001;

    clientWidth=user->getWidth();
    clientHeight=user->getHeight();

//        x=clientWidth/2;
//        y=clientHeight/2;

    }

    void setRoof(float r)
    {
        rooflevel=trackZone->top=r;
    }
    void setFloor(float f)
    {
        floorlevel=trackZone->bottom=f;
    }


    void track()
    {

        //************************************
        // Data Visualization
        //************************************
        dataView->clear();
        dataView->begin();
        ofPushStyle();
        ofSetColor(255,255,255);
        ofNoFill();
        ofPushMatrix();
        ofTranslate(dataView->getWidth()/2,dataView->getHeight()/2,TDist);
        ofRotateX(TRotY);
        ofRotateZ(TRotX);
        ofTranslate(0,0,THeight*v_scale);
        ofSetColor(0,0,255);
        ofRect(-clientWidth/2,-clientHeight/2,clientWidth,clientHeight);  //-Zero plane
        ofPushMatrix();
        ofScale(10,10);
        ofDrawBitmapString("Ceiling",-25,0);
        ofPopMatrix();
        ofSetColor(255,255,255);

        ofSetSphereResolution(100);

//        ofScale(1,1,0.25);
        //************************************


        trackZone->update();

		if(trackZone->signal>1000)
        {
            x=x*0.75+( trackZone->x=trackZone->vx)*0.25;
            y=y*0.75+( trackZone->y=trackZone->vy)*0.25;
			active=1;
        }
        else
        {
            y =  trackZone->y=trackZone->y*.95+(clientHeight/2)*0.05;
            x =  trackZone->x=trackZone->x*.95+(clientWidth/2)*0.05;
            active=0;
        }

        apex=trackZone->apex;


		target_filter+=( trackZone->signal-(float)(height*width*128))*gain;
//		target_filter+=( trackZone->signal-(float)(height*width*50))*gain;
		if(target_filter>maxsize)
			target_filter=maxsize;
		if(target_filter<minsize)
			target_filter=minsize;

        trackZone->height=trackZone->width=target_filter;

		width=height=target_filter;

        float xp=x-clientWidth/2.0;
        float yp=y-clientHeight/2.0;


        float ang=ofRadToDeg(atan2(xp,-yp));
        azimuth=ang+offset;
        float ra=sqrt(xp*xp+yp*yp);
        radius=ra;
        if(radius>(clientHeight/2.0)-width/2.0)
            radius=(clientHeight/2.0)-width/2.0;
        radius=radius/((clientHeight/2.0)-width/2.0);



        //------------------------------------
        //  Gesture tracking
        //------------------------------------

        int N=gesture.Size();
        for(int i=0;i<N;i++)
        {
            float w=(360.0*i)/(float)N;
            gesture.angles[i]=w;
            gestureZones[i]->x=x+(width/2+GESTURE_RADIUS)*cos(ofDegToRad(w+azimuth));
            gestureZones[i]->y=y+(height/2+GESTURE_RADIUS)*sin(ofDegToRad(w+azimuth));

            gestureZones[i]->bottom=apex+100;
            gestureZones[i]->update();

            if(gestureZones[i]->signal>100)
            {
                gesture.active[i]=true;
            }
            else
            {
                gesture.active[i]=false;
                gestureZones[i]->apex=gestureZones[i]->bottom;
            }
            gesture.signals[i]=gestureZones[i]->signal;
            gesture.elevations[i]=(apex-gestureZones[i]->apex)*0.1;
        }


        //************************************
        // Data Visualization
        //************************************



        ofPushStyle();
        ofNoFill();
        ofSetColor(255,255,255,80);
        ofSphere(0,0,0,floorlevel*v_scale);
        ofPopStyle();







        ofPopMatrix();




        ofPushStyle();
        for(int i=0;i<gesture.Size();i++)
        {
            ofSetColor(255,255,255);
            ofDrawBitmapString("Zone "+ofToString(i),i*75+5,10);

            if(gesture.active[i])
            {
                ofDrawBitmapString(ofToString(gesture.elevations[i]),i*75+5,25);
                ofFill();
                ofSetColor(0,255,0);
                ofRect(i*75+1,30,75,30);

            }
            else
            {
                ofNoFill();
                ofSetColor(255,0,0);
                ofRect(i*75+1,30,75,30);
            }
            ofNoFill();
            ofSetColor(255,255,0);
            ofRect(i*75+1,0,75,30);

        }

        ofPopStyle();
/*
        if(n>50)
        {
            ofRect(0,0,50,50);
        }
*/

        ofPopStyle();
        dataView->end();
        //************************************


    }



};







#endif
