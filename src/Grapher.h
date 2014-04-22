#ifndef GRAPHER_H
#define GRAPHER_H
#include "ofMain.h"

class Grapher
{
    public:
        int x,y,w,h;
        float f0=0;
        ofFbo chart;

        Grapher(int a, int b,int c ,int d)
        {
            x=a;
            y=b;
            w=c;
            h=d;
            chart.allocate(w,h,GL_RGB);
        }
        void update(float f)
        {

            ofFbo temp=chart;
            chart.begin();
                temp.draw(1,0);

//                ofBackground(0);
                ofNoFill();
                ofSetColor(0,0,0);
                ofRect(0,0,1,h);
                ofSetColor(255,255,255);

//                ofLine(0,f,0,f0);
                ofRect(0,0,1,f);
//                temp.draw(1,0);
            chart.end();

            f0=f;
        }
        void paint()
        {
            chart.draw(x,y);
        }

    protected:
    private:
};

#endif // GRAPHER_H
