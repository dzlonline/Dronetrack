#ifndef _TARGET
#define _TARGET


class VDtarget
{
public:
    float x;
    float y;
    float tx;
    float ty;
    int sample_h;
    int sample_w;
    float wf;
    float size;
    unsigned char sample[255*255];
    float track_r;
    float track_g;
    float track_b;
    unsigned char *video;
    int video_width;
    int video_height;
	VDtarget(int initialX, int initialY);
    void setcolor(unsigned char r,unsigned char g,unsigned char b);
	void setpos(int x,int y);
	void track(unsigned char *video);
private:
    float max(float a,float b,float c);
};

#endif
