#include "testApp.h"
#include "Grapher.h"

#include <math.h>

#include "TFlightController.h"

ofTrueTypeFont font;
ofTrueTypeFont font1;

Grapher *chart1;

float rot;


float posX=0;
float posY=0;

//--------------------------------------------------------------
void testApp::setup() {

    drone0.command(0,0,1500,0);
//    drone0.command(320,240,900,0);


    font.loadFont("arial.ttf",12);
    font1.loadFont("arial.ttf",200);

    controller.open("COM12");
    mixer.open("COM50");


//    handset.setup("COM12",19200);


 //   inptr=0;

	kinect.setRegistration(true);
	kinect.init();
	kinect.open();		// opens first available kinect

    redDot=new TColorTracker(ofPoint(320,250));
    greenDot=new TColorTracker(ofPoint(320,250));

	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";

        camWidth=640;
        camHeight=320;

        redDot->bindKinect(&kinect);
        greenDot->bindKinect(&kinect);
	}

    chart1=new Grapher(0,kinect.getHeight(),kinect.getWidth(),255);


#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif

	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;

	ofSetFrameRate(25);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);

	// start from the front
	bDrawPointCloud = false;
}

//--------------------------------------------------------------
void testApp::update() {

//    cout << ofGetFrameRate() << "\r\n";

    mixer.update();

    float p=(float)mixer.indata[0]*(0.1/254.0);
    float i=(float)mixer.indata[1]*(0.001/254.0);
    float d=(float)mixer.indata[2]*(10.0/254.0);

    float v=((float)mixer.indata[0]-128.0) *(M_PI/128.0);

    float fc=(float)mixer.indata[0]*(0.5/254.0);
 //   cout << fc << "\t" << "\r\n";

 //   redDot->filter.setBiquad(OFX_BIQUAD_TYPE_LOWPASS, 0.001, 0.7, 0.0);

//    redDot->filterZ.setFc(fc);
 //   redDot->filterX.setFc(fc);
  //  redDot->filterY.setFc(fc);
//    greenDot->filterZ.setFc(fc);
    greenDot->filterX.setFc(fc);
    greenDot->filterY.setFc(fc);

  //  cout << v << "\r\n";

    float alt=500.0+((float)mixer.indata[3]*(4000.0/254.0));

    float g=(float)mixer.indata[5]*(3.0/254.0);
    float g2=(float)mixer.indata[4]*(3.0/254.0);




    drone0.pitchPID.setGain(g);
    drone0.rollPID.setGain(g);

//    drone0.throttlePID.setup(p,i,d,200,g2);


//    drone0.pitchPID.setup(p,i,d,200,g);
//    drone0.rollPID.setup(p,i,d,200,g);
    drone0.throttlePID.setGain(g2);

    drone0.command(0,0,alt,v);
/*    drone0.throttlePID.setup(p,i,d,200,g)2;
    drone0.pitchPID.setGain(g2);
    drone0.rollPID.setGain(g2);
*/

    cout << p << "\t" << i << "\t" << d << "\t" << g << "\t"<< alt << "\t" << "\r\n";






    controller.update();




    if(controller.indata[0]<5)
        drone0.setMode(PILOT_ALL_MANUAL);

    drone0.flyByWire(controller.indata[2],controller.indata[3],controller.indata[1],controller.indata[0]);

    //-Send flight data via this particular TX system

    controller.outdata[0]=drone0.droneData[0];
    controller.outdata[1]=drone0.droneData[1];
    controller.outdata[2]=drone0.droneData[3];
    controller.outdata[3]=drone0.droneData[4];

    controller.send(5);

    controller.outdata[4]=0;    //-Reset bind byte

    float y0=redDot->position.x-greenDot->position.x;
    float x0=redDot->position.y-greenDot->position.y;
    float ang=atan2( x0 , y0 );

    rot=ang;

    float xd=(greenDot->position.x+redDot->position.x )/2;
    float yd=(greenDot->position.y+redDot->position.y )/2;


    ofPoint w=kinect.getWorldCoordinateAt(xd,yd);

    xd=w.x;
    yd=w.y;

//    cout << w.x << "\t," << w.y << "\r\n";

//    xd-=320;
//    yd-=240;


//    float tx=cos(ang)*xd-sin(ang)*yd;
//    float ty=sin(ang)*xd+cos(ang)*yd;

    drone0.update(xd,-yd,(greenDot->position.z+redDot->position.z )/2,ang);

/*
    while(handset.available())
    {
        unsigned char c=handset.readByte();
        if(c==255)
        {
            inptr=0;

            if(inbuf[0]<5)EAN nr. 579 800 0422 308. Angiv følgende: Gunhild Borggreen, Praksisgruppen
                drone0.setMode(PILOT_ALL_MANUAL);

            drone0.flyByWire(inbuf[2],inbuf[3],inbuf[1],inbuf[0]);


            //-Send flight data via this particular TX system

            outbuf[0]=drone0.droneData[0];
            outbuf[1]=drone0.droneData[1];
            outbuf[2]=drone0.droneData[3];
            outbuf[3]=drone0.droneData[4];
            for(int i=0;i<5;i++)
            {
                handset.writeByte(outbuf[i]);
            }
            handset.writeByte(255);
            outbuf[4]=0;    //-Reset bind byte

            float y0=redDot->position.x-greenDot->position.x;
            float x0=redDot->position.y-greenDot->position.y;
            float ang=atan2( x0 , y0 );


            float xd=(greenDot->position.x+redDot->position.x )/2-320;
            float yd=kinect.getHeight()-(greenDot->position.y+redDot->position.y )/2-240;

            float tx=cos(ang)*xd-sin(ang)*yd;
            float ty=sin(ang)*xd+cos(ang)*yd;

            drone0.update(tx,ty,(greenDot->position.z+redDot->position.z )/2,0);


        }
        else
        {
            if(inptr<5)
                inbuf[inptr++]=c;
        }
    }
*/

//    chart1->update(drone0.droneData[4]);

//	ofBackground(100, 100, 100);

	kinect.update();
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

//        redDot->track();
//        greenDot->track();


		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {

			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();

			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}

		// update the cv images
		grayImage.flagImageChanged();

		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);


	}

#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void testApp::draw() {

	ofSetColor(255, 255, 255);
    kinect.draw(0, 0, 640, 480);

    redDot->track();
    greenDot->track();


    ofEllipse(posX+320,-posY+240,100,100);

    ofSetColor(redDot->trackColor);
    ofNoFill();
//    ofEllipse(redDot->position.x-redDot->window/2 ,redDot->position.y-redDot->window/2,redDot->window,redDot->window);
    ofEllipse(redDot->position.x ,redDot->position.y,redDot->window,redDot->window);
//    redDot->track();

    ofSetColor(greenDot->trackColor);
    ofNoFill();
//    ofEllipse(greenDot->position.x-greenDot->window/2 ,greenDot->position.y-greenDot->window/2,greenDot->window,greenDot->window);
    ofEllipse(greenDot->position.x ,greenDot->position.y,greenDot->window,greenDot->window);
//    greenDot->track();

    ofLine(redDot->position.x,redDot->position.y,greenDot->position.x,greenDot->position.y);

    float y0=redDot->position.x-greenDot->position.x;
    float x0=redDot->position.y-greenDot->position.y;

    float ang=atan2( x0 , y0 );

    float xd=(redDot->position.x+greenDot->position.x)/2;
    float yd=(redDot->position.y+greenDot->position.y)/2;

    ofPushMatrix();
    ofTranslate(xd,yd);
    ofRotateZ(ofRadToDeg(ang));




/*

    float x=(greenDot->position.x+redDot->position.x )/2-320;
    float y=kinect.getHeight()-(greenDot->position.y+redDot->position.y )/2-240;

    float tx=cos(ang)*x-sin(ang)*y;
    float ty=sin(ang)*x+cos(ang)*y;


//    ofRect(0,0,x,y);

    ofRect(-5,0,10,ty);
    ofRect(0,-5,-tx,10);
*/


    ofSetColor(0,255,0);
    ofRect(-100,-10,200,20);
//    ofTranslate(-10,100);

    stringstream status;
    status << "Angle: " << ofRadToDeg(ang);
//    ofDrawBitmapString(status.str(),-20, 50);

    font.drawString(status.str(),-40, 30);
    stringstream angstr1;
    angstr1 << "distance: " << redDot->position.z;
//    ofDrawBitmapString(angstr1.str(),+20, 30);
    font.drawString(angstr1.str(),-50, -25);


    ofPopMatrix();


    ofRect(10,kinect.getHeight()-controller.indata[0]/3,8,controller.indata[0]/3);
    ofRect(20,kinect.getHeight()-controller.indata[1]/3,8,controller.indata[1]/3);
    ofRect(30,kinect.getHeight()-controller.indata[2]/3,8,controller.indata[2]/3);
    ofRect(40,kinect.getHeight()-controller.indata[3]/3,8,controller.indata[3]/3);
//    ofRect(50,kinect.getHeight()-throttle/3,8,throttle/3);



//    chart1->update(greenDot->position.z-(drone0.cmdZ-128));
//    chart1->paint();



	if(bDrawPointCloud) {
		easyCam.begin();
//		drawPointCloud();

        viewScene();
		easyCam.end();
	}

	/*
	 else {
		// draw from the live kinect
//		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);

//		grayImage.draw(10, 320, 400, 300);
//		contourFinder.draw(10, 320, 400, 300);

#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}

	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;

    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
    }

	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }

	ofDrawBitmapString(reportStream.str(), 20, 652);

*/
}

void testApp::viewScene()
{

    ofPushStyle();
    ofEnableLighting();
    pointLight.enable();
    ofEnableDepthTest();


    ofPushMatrix();
    ofTranslate(0,0,1000);
    ofBox(2000);
    ofPopMatrix();

    ofPushMatrix();
    ofTranslate(0,0,2000);
    stringstream floor;
    floor << "FLOOR";
    ofScale(-1,1);
    //ofRotateZ(M_PI);

//    kinect.draw(-320,-240);


    font1.drawString(floor.str(),-380,0);
    ofPopMatrix();


    ofPushMatrix();
    ofTranslate(0,0,-50);
    ofBox(0,0,0,280,40,60);

    ofPoint p0=kinect.getWorldCoordinateAt(redDot->position.x,redDot->position.y);
    ofPoint p1=kinect.getWorldCoordinateAt(greenDot->position.x,greenDot->position.y);

    ofFill();
    ofSetColor(redDot->trackColor);
    ofPushMatrix();
    ofTranslate(p0.x,p0.y,redDot->position.z);
    ofRotateZ(ofRadToDeg(rot));
    ofBox(100);
    ofPopMatrix();
//    ofSphere(p0.x,p0.y,p0.z,redDot->window);
    ofSetColor(greenDot->trackColor);

    ofPushMatrix();
    ofTranslate(p1.x,p1.y,greenDot->position.z);
    ofRotateZ(ofRadToDeg(rot));
    ofBox(100);
    ofPopMatrix();


//    ofBox(p1.x,p1.y,p1.z,100);
//    ofSphere(p1.x,p1.y,p1.z,greenDot->window);


    ofDisableDepthTest();
    ofPopMatrix();
    ofDisableLighting();
    ofPopStyle();
}



void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 1;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
                if(        (ofDist(x,y,redDot->position.x,redDot->position.y)<(redDot->window/4))||(ofDist(x,y,greenDot->position.x,greenDot->position.y)<(greenDot->window/4))
             //      (ofDist(redDot->position.x-redDot->window/2+x,redDot->position.y-redDot->window/2+y,redDot->position.x,redDot->position.y)<(redDot->window/2))
                    )
                {
                    mesh.addColor(kinect.getColorAt(x,y));
                    mesh.addVertex(kinect.getWorldCoordinateAt(x, y));

                }
                else
                {
                  //  mesh.addColor(ofColor(0));
                   // mesh.addVertex(kinect.getWorldCoordinateAt(x, y));

                }
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards'
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();


	ofSetColor(255);
	ofPoint p=kinect.getWorldCoordinateAt(redDot->position.x,redDot->position.y);
	ofLine(p.x,p.y,2000,p.x,p.y,redDot->position.z);

	p=kinect.getWorldCoordinateAt(greenDot->position.x,greenDot->position.y);
	ofLine(p.x,p.y,2000,p.x,p.y,greenDot->position.z);


	ofDisableDepthTest();
	ofPopMatrix();

}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();

#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;

		case'a':
//			manual=1;
            drone0.setMode(PILOT_ALL_MANUAL);

			break;
		case's':

            posX=0;
            posY=0;
            drone0.command(posX,posY,1500,0);
            drone0.setMode(PILOT_YAW);
//            drone0.setMode(PILOT_ALL_AUTO);
       //     drone0.throttlePID.reset(drone0.throttleOut);


//			manual=0;
//			altitude.I=0;
//			pitch.I=128;
//			roll.I=128;
			break;

		case'r':
			controller.outdata[4]=1;
			break;
		case't':
			controller.outdata[4]=2;
			break;


		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;

		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;

		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;

		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;

		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;

		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;

		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;

		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;

		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;

		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;

		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;

		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;

		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;

		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;

        case OF_KEY_UP:
            drone0.command(posX,posY++,900,0);
            break;

        case OF_KEY_DOWN:
            drone0.command(posX,posY--,900,0);
            break;

        case OF_KEY_LEFT:
            drone0.command(posX--,posY,900,0);
            break;

        case OF_KEY_RIGHT:
            drone0.command(posX++,posY,900,0);

            break;


/*		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;

*/



	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
    if(y>kinect.getHeight()-2)
        return;
    if(x>kinect.getWidth()-2)
        return;

    if(button==2)
    {
        redDot->setColor(kinect.getColorAt(x,y));
        redDot->setPosition(ofPoint(x,y));
        redDot->window=20;
    }
    if(button==0)
    {
        greenDot->setColor(kinect.getColorAt(x,y));
        greenDot->setPosition(ofPoint(x,y));
        greenDot->window=20;
    }




}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
