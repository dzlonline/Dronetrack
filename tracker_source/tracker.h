#ifndef _TRACKER
#define _TRACKER

class TTracker
{
public:
    float x;
    float y;

    float azimuth;
    float radius;

    int width;
    int height;
    int clientWidth;
    int clientHeight;
    unsigned char* video;
    unsigned char active;
    float signal;

    TTracker()
    {
        clientWidth=0;
        clientHeight=0;
        x=100;
        y=100;
        width=30;
        height=30;
        active=0;
        signal=0;
    }

    void track()
    {
        if(x<width/2)
            x=width/2;
        if(x>clientWidth-width/2)
            x=clientWidth-width/2;
        if(y<height/2)
            y=height/2;
        if(y>clientHeight-height/2)
            y=clientHeight-height/2;

        float vx=0;
        float vy=0;
        float z=0;
        int x0=x-(width/2);
        int y0=y-(height/2);

        for(int ty=y0;ty<(y0+height);ty++)
        {
            for(int tx=x0;tx<(x0+width);tx++)
            {
                int a=(tx+ty*clientWidth)*3;
                float g=video[a];
                video[a]=128;

                vx+=(float)tx*g;
                vy+=(float)ty*g;
                z+=g;
            }
        }

        if(z>1000)
        {
            vx/=z;
            vy/=z;
            x=x*.5+vx*.5;
            y=y*.5+vy*.5;
            active=1;
        }
        else
        {
            x=x*.95+(clientWidth/2)*0.05;
            y=y*.95+(clientHeight/2)*0.05;
            active=0;
        }

        if(x<width/2)
            x=width/2;
        if(x>clientWidth-width/2)
            x=clientWidth-width/2;
        if(y<height/2)
            y=height/2;
        if(y>clientHeight-height/2)
            y=clientHeight-height/2;


        float xp=x-clientWidth/2.0;
        float yp=y-clientHeight/2.0;

        azimuth=ofRadToDeg(atan2(xp,yp));
        radius=sqrt(xp*xp+yp*yp);

        if(radius>(clientHeight/2.0)-width/2.0)
            radius=(clientHeight/2.0)-width/2.0;

        radius=radius/((clientHeight/2.0)-width/2.0);
    }
};

#endif
