#include "TColorTracker.h"
#include "ofMain.h"
#include <math.h>

TColorTracker::TColorTracker(ofPoint p)
{
    kinect=NULL;
    position=p;
    window=50;
    active=0;
    filterX.setBiquad(OFX_BIQUAD_TYPE_LOWPASS, 0.5, 0.7, 0.0);
    filterY.setBiquad(OFX_BIQUAD_TYPE_LOWPASS, 0.5, 0.7, 0.0);
    filterZ.setBiquad(OFX_BIQUAD_TYPE_LOWPASS, 0.1, 0.7, 0.0);
 //   filter = ofxBiquadFilter1f(OFX_BIQUAD_TYPE_LOWPASS, 0.01, 0.7, 0.0);
}

void TColorTracker::bindKinect(ofxKinect *k)
{
    kinect=k;
    active=1;
    clientWidth=kinect->getWidth();
    clientHeight=kinect->getHeight();
}

void TColorTracker::setColor(ofColor c)
{
    float a=max(c.r,c.g,c.b)/255.0;
    if(a>0)
    {
        trackColor.r=c.r/a;
        trackColor.g=c.g/a;
        trackColor.b=c.b/a;
    }
}
void TColorTracker::setPosition(ofPoint p)
{
    position=p;
    anchor=p;
}

int TColorTracker::track()
{
    if(!active)
        return 0;

    float w=1;              //-Number of matching pixels in sample
    float t=0;              //-Number of "black" pixels in sample
    ofPoint tp=position;    //-Current position "1.st pixel"
    tp.z=8000;
    float x,y;              //-Current sample position

//    float zcnt=0;


//    float height=0;
//    int heightcount=0;


    for (int py=0;py<window;py++)
    {
      for (int px=0;px<window;px++)
      {
        x=position.x-(window/2)+px;     //-Calculate current sample pixel position in video
        y= position.y-(window/2)+py;    //+

        if(ofDist(x,y,position.x,position.y)<(window/2))    //-Make circular sample
        {
            //--Normalize sampled color--
            ofColor c=kinect->getColorAt(x,y);
            float a=max(c.r,c.g,c.b)/255.0;
            ofColor n=ofColor(0,0,0);
            if(a>0)
            {
                n.r=c.r/a;
                n.g=c.g/a;
                n.b=c.b/a;
            }
            //--Calculate match--
            float d = sqrt((n.r-trackColor.r)*(n.r-trackColor.r) +(n.g-trackColor.g)*(n.g-trackColor.g) +(n.b-trackColor.b)*(n.b-trackColor.b) )/250.0;

            if ((d<0.25)&&(a>0.3))
            {
                //--Accumolate position--
                w++;
                tp.x+=x;
                tp.y+=y;

//                ofSetColor(n);
//                ofRect(position.x-window/2+px,position.y-window/2+py,1,1);
            }
            else
            {
//                t++;
//                ofSetColor(ofColor(0,0,0));
//                ofRect(position.x-window/2+px,position.y-window/2+py,1,1);

            }
            t++;

            //--Calculate distance to Kinect--
//            float dc=kinect->getWorldCoordinateAt(x,y).z;


//            if(ofDist(x,y,position.x,position.y)<(window/2))    //-Make circular sample
            {
                float dc=kinect->getDistanceAt(x,y);

/*                if(dc>500&&dc<2300)
                {
                    zcnt++;
                    tp.z+=dc;
                }
*/
                if(dc>500)
                {
                    if(dc<tp.z)
                        tp.z=dc;
                }

            }
        }
      }
    }

//    tp.z=height/heightcount;

    //--Calculate centroid--
    tp.x/=w;
    tp.y/=w;
/*    if(zcnt>0)
    {
        tp.z/=zcnt;
    }
*/
    //--Average distance--
//    distance/=dw;

    //--Calculate signal ratio--
    float sr;
//    if(t>0)
        sr=((float)w/(float)t);
//    else
//        sr=1.0;


    if(sr>0.01) //-Is 10% pixels in sample a match?
    {

        //tp.x= position.x-(window/2.0)+tp.x-2.0;
       // tp.y= position.y-(window/2.0)+tp.y-2.0;

        position.x=tp.x;
        position.y=tp.y;


//        position.x=filterX.process(tp.x);
//        position.y=filterY.process(tp.y);
//        position.z=position.z*0.7+tp.z*0.3;


//        if(zcnt>10)
//            position.z=tp.z;
/*        if(tp.z<8000)
            position.z=position.z*0.5+tp.z*0.5;
*/

        if(tp.z<8000)
        {
//            position.z=filterZ.process(tp.z);
            position.z=position.z*0.5+tp.z*0.5;
        }



//        position=tp;

        if(t>0)
        {
            if(sr>0.18)
                window+=0.7;
            else
                window-=0.7;

            if(window<10)
                window=10;
            if(window>200)
                window=200;
        }
    }
    else
    {

        position.x=position.x*0.95+anchor.x*0.05;
        position.y=position.y*0.95+anchor.y*0.05;
        filterX.process(position.x);
        filterY.process(position.y);


        window=window*0.99+50.0*0.01;

    }

 /*       ofSetColor(255);
        ofLine(tp.x-100,tp.y,tp.x+100,tp.y);
        ofLine(tp.x,tp.y-100,tp.x,tp.y+100);
*/


    if(position.x-(window/2)<1)
        position.x=window/2+1;

    if((position.x+(window/2)+1) > clientWidth )
        position.x=clientWidth-(window/2)-1;

    if(position.y-(window/2)<1)
        position.y=window/2+1;

    if((position.y+(window/2)+1) > clientHeight )
        position.y=clientHeight-(window/2)-1;


/*        ofSetColor(255);
        ofLine(tp.x-100,tp.y,tp.x+100,tp.y);
        ofLine(tp.x,tp.y-100,tp.x,tp.y+100);
*/

//    position.z=kinect->getDistanceAt(position.x,position.y);



}

float TColorTracker::max(float a,float b,float c)
{
    float d;
    if(a>b)
        d=a;
    else
        d=b;
    if(c>d)
        return c;
    else
        return d;
}
