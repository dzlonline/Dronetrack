#include <math.h>
#include "target.h"

VDtarget::VDtarget(int initialX , int initialY)
{
    x=initialX;
    y=initialY;
    track_r=1.0;
    track_g=0.0;
    track_b=0.0;

    sample_h=32;
    sample_w=32;
    wf=sample_w;

    size=32;

    video_width=640;
    video_height=480;
}

void VDtarget::setcolor(unsigned char r,unsigned char g,unsigned char b)
{
    float a=(float)max(r,g,b);
    track_r=(float)r/a;
    track_g=(float)g/a;
    track_b=(float)b/a;
}

void VDtarget::setpos(int px,int py)
{
    size=20;
    sample_h=sample_w=((int)size)*2;
    x=px;
    y=py;


    if(x<sample_w/2)
    {
        x=sample_w;
    }
    if(x>video_width-sample_w/2)
    {
        x=video_width-sample_w/2;
    }
    if(y<sample_h/2)
    {
        y=sample_h/2;
    }
    if(y>video_height-sample_h/2)
    {
        y=video_height-sample_h/2;
    }

}

void VDtarget::track(unsigned char *video)
{
    unsigned long w=0;
    unsigned long t=0;
    tx=x;
    ty=y;


/*    float r2 = red(trackColor);
    float g2 = green(trackColor);
    float b2 = blue(trackColor);


    float a2=max(r2, g2, b2);
    r2=r2/a2;
    g2=g2/a2;
    b2=b2/a2;
*/



    for (int py=0;py<sample_w;py++)
    {
      for (int px=0;px<sample_w;px++)
      {
//        color currentColor=p.get(x-(sample_w/2)+px, y-(sample_w/2)+py);


        int loc=(tx+(int)x)*3+((ty+(int)y)*video_width*3);
        float r1=video[loc];
        float g1=video[loc+1];
        float b1=video[loc+2];
        float a1=max(r1,g1,b1);

        r1/=a1;
        g1/=a1;
        b1/=a1;

//        float r1 = red(currentColor);
//        float g1 = green(currentColor);
//        float b1 = blue(currentColor);
//        float a1=max(r1, g1, b1);

//        float d = dist(r1/a1, g1/a1, b1/a1, track_r, track_g, track_b);
        float d=sqrt((r1-track_r)*(r1-track_r)+(g1-track_g)*(g1-track_g)+(b1-track_b)*(b1-track_b));



        if (d<1)
//        if (d<.2)
        {
          w++;
          tx+=px;
          ty+=py;
//          set(px, py, color(255, 0, 0));
        }
        else
        {
           t++;
//          set(px, py, currentColor);
        }
      }
    }

    if (w>0)
    {
      tx/=w;
      ty/=w;
      tx+=(x-sample_w/2);
      ty+=(y-sample_w/2);


      if(t>0)
      {
        if((w/t)>0.7)
          wf+=0.7;
        else
          wf-=0.7;
        if(wf<5)
          wf=5;
        if(wf>200)
          wf=200;
      }
      sample_w=sample_h=(int)wf;

    }
/*    else
    {
      return false;
    }
  */









/*

    sample_h=sample_w=((int)size)*2;

    float r1,g1,b1;
    float a1;
    int p=0;

    for(int sy=(-sample_h/2);sy<(sample_h/2);sy++)
    {
        for(int sx=(-sample_w/2);sx<(sample_w/2);sx++)
        {
            int loc=(sx+(int)x)*3+((sy+(int)y)*video_width*3);
//            int loc=(sx+x)*3+((sy+y)*video_width*3);
            r1=video[loc];
            g1=video[loc+1];
            b1=video[loc+2];
            a1=max(r1,g1,b1);

            r1/=a1;
            g1/=a1;
            b1/=a1;

//            sample[p++]=video[loc];

            float d=sqrt((r1-track_r)*(r1-track_r)+(g1-track_g)*(g1-track_g)+(b1-track_b)*(b1-track_b));
//            sample[p++]=d*255;
            if(d<.2)
            {
                sample[p++]=255-d*255;
            }
            else
            {
                sample[p++]=0;
            }
        }
    }

//        float tx=0;
//        float ty=0;
    float g;
    float z=0;
    int n=0;
    for(int sx=(-sample_w/2);sx<(sample_w/2);sx++)
    {
        for(int sy=-sample_h/2;sy<sample_h/2;sy++)
        {
            n++;
            g=(float)sample[(sx+sample_w/2)+((sy+sample_h/2)*sample_w)]/255.0;
            z+=g;
            tx+=g*(float)sx;
            ty+=g*(float)sy;
        }
    }
//    if(z>10.0)
    if(z>n/100.0)
    {
        tx/=z;
        ty/=z;
        if(z>n/3)
        {
            size+=1.3;
        }
//        else
        if(z<n/4)
        {
            size-=1.3;
        }

            if(size>120)
            {
                size=120.0;
            }

            if(size<5)
            {
                size=5.0;
            }
        }
        else
        {
            tx=0;
            ty=0;
        }


        x=x*.1+(x+tx)*0.9;
        y=y*.1+(y+ty)*0.9;


        if(x<sample_w/2)
        {
            x=sample_w;
        }
        if(x>video_width-sample_w/2)
        {
            x=video_width-sample_w/2;
        }
        if(y<sample_h/2)
        {
            y=sample_h/2;
        }
        if(y>video_height-sample_h/2)
        {
            y=video_height-sample_h/2;
        }
*/
}

float VDtarget::max(float a,float b,float c)
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
