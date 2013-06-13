#include "testApp.h"
#include "homography.h"

#define COUNTDOWN 0
//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

    // enable depth->rgb image calibration
	kinect.setRegistration(true);

	//kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	kinect.init(false, false); // disable video image (faster fps)
	kinect.open();

    angle=0;
	kinect.setCameraTiltAngle(angle);
	//ofSleepMillis(1000);

	ofAddListener(touchTracker.blobAdded, this, &testApp::touchOn);
    ofAddListener(touchTracker.blobMoved, this, &testApp::touchMoved);
    ofAddListener(touchTracker.blobDeleted, this, &testApp::touchOff);

    background.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    backgroundTex.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    diffMask.allocate(kinect.width, kinect.height);

    nearThreshold=1100.;
    farThreshold=1400.;

    touchDiffFarThreshold=34.;
    touchDiffNearThreshold=7.;

    numPixels = kinect.width*kinect.height;

    maxBlobs = 10;
    minBlobPoints=40;
    maxBlobPoints=1000;
    cout<< "minBlobPoints " << minBlobPoints <<endl;
    cout<< "maxBlobPoints " << maxBlobPoints <<endl;

    hulk=20;

	bLearnBackground = false;
	timeLapse = 0;

	ofSetFrameRate(60);

	// start from the front
	bDrawIDs = true;

	calibrationMode=false;
    calibrationPoint=0;
    calibrated=false;
}

//--------------------------------------------------------------
void testApp::update() {

	ofBackground(100, 100, 100);

	kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

		// load depth image from the kinect source
		current=kinect.getDistancePixelsRef();
		// background subtracticon
        if ((bLearnBackground) && (ofGetElapsedTimeMillis() >= timeLapse)) {
            getBackground();
            bLearnBackground = false;
        }
        unsigned char * momento = new unsigned char[numPixels];
        float diff;
        for(int i=0;i<numPixels;i++)
        {
            momento[i]=0;
            if(current[i]<farThreshold && current[i]>nearThreshold)
            {
                diff=background[i]-current[i];
                if(diff>touchDiffNearThreshold && diff<touchDiffFarThreshold)
                {
                    momento[i]=(unsigned char)ofMap(diff,touchDiffNearThreshold,touchDiffFarThreshold,255,20);
                }
            }
        }
        diffMask.setFromPixels(momento, kinect.width, kinect.height);
        if(bDrawIDs)
        {
            cvErode(diffMask.getCvImage(), diffMask.getCvImage(), NULL, 3);
            cvDilate(diffMask.getCvImage(), diffMask.getCvImage(), NULL, 2);
        }

        touchTracker.update(diffMask,-1, minBlobPoints , maxBlobPoints, maxBlobs, hulk,false, true);
    }

#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void testApp::draw() {

	ofSetColor(255, 255, 255);

    // draw from the live kinect
    kinect.drawDepth(10, 10, 320, 240);

    backgroundTex.draw(340,10,320,240);

    diffMask.draw(10, 260, 320, 240);
    touchTracker.draw(10, 260, 320, 240);

    ofSetColor(200, 200, 200);
    ofLine(10+160,10,10+160,10+118);
    ofLine(10,10+120,10+158,10+120);

    if(calibrated)
    {
        ofSetColor(0);
        ofRect(340,260,320,240);
        ofSetColor(0,0,240);
        for(int i=0;i<touchTracker.size();i++)
        {
            ofCircle(homography*touchTracker[i].centroid,10);
        }
    }

	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "press b to set current frame as background  hulk:" << hulk  << endl
	<< "num touch points found: " << touchTracker.size()
	<< ", fps: " << ofToString(ofGetFrameRate(),2) << endl
	<< "press n and m to change touchDiffFarThreshold:"<< ofToString(touchDiffFarThreshold,2) << "   k and l to change touchDiffNearThreshold:"<< ofToString(touchDiffNearThreshold,2) << endl
	<< "press RIGHT and LEFT to change distance thresholds. near:"<< nearThreshold << " far:" << farThreshold<< endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
	if(calibrationMode)
        reportStream<<"CALIBRATION MODE: please locate the 4 corners bottomright-topright-topleft-bottomleft"<<endl;
	ofDrawBitmapString(reportStream.str(),20,510);
}

void testApp::exit() {
    //kinect.setCameraTiltAngle(0);
	kinect.close();

#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

void testApp::getBackground()
{
    background = current;   // let this frame be the background image from now on

    int rep=50;
    while(rep && kinect.isConnected())
    {
        kinect.update();

        // there is a new frame and we are connected
        if(kinect.isFrameNew()) {
            // load depth image from the kinect source
            current=kinect.getDistancePixelsRef();
            for(int i = 0; i < numPixels; i++) {
                if(background[i]==0)
                    background[i]=current[i];
                else
                    background[i]=(background[i]+current[i])/2.;
            }
            rep--;
        }
    }
    unsigned char * momento = new unsigned char[numPixels];
    for(int i=0;i<numPixels;i++)
    {
        if(background[i]>nearThreshold && background[i]<farThreshold)
            momento[i]=(unsigned char)ofMap(background[i],nearThreshold,farThreshold,255,0,true);
        else
            momento[i]=0;
    }
    backgroundTex.setFromPixels(momento,kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
}
//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
	    case 't':
            calibrationMode=true;
            break;

		case 'b':
            bLearnBackground = true;
            // to give you time to run away of the field of view
            timeLapse = COUNTDOWN+ofGetElapsedTimeMillis();
            break;

        case'i':
			bDrawIDs = !bDrawIDs;
			break;

		//case 'w':
		//	kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
		//	break;

		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;

		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
        case 'w':
            farThreshold+=20.; //5cm
            break;
        case 's':
            farThreshold-=20.; //5cm
            break;
        case 'e':
            nearThreshold+=20.; //5cm
            break;
        case 'd':
            nearThreshold-=20.; //5cm
            break;

        case 'n':
			touchDiffFarThreshold-=0.5;
			break;
       case 'm':
			touchDiffFarThreshold+=0.5;
			break;
        case 'k':
			touchDiffNearThreshold-=0.5;
			break;
        case 'l':
			touchDiffNearThreshold+=0.5;
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
/*
 *
 *	blob section
 *
 *	from here on in it's blobs
 *	thanks to stefanix and the opencv library :)
 *
 */

//--------------------------------------------------
void testApp::touchOn( ofxBlob& b ) {
    //cout << "blobOn() - id:" << id << " order:" << order << endl;
    if(calibrationMode)
    {
        src[calibrationPoint++]=b.centroid;
        if(calibrationPoint==4)
        {
            dst[0].x=340;dst[0].y=500;
            dst[1].x=340;dst[1].y=260;
            dst[2].x=660;dst[2].y=260;
            dst[3].x=660;dst[3].y=500;
            homography=findHomography(src,dst);
            calibrationPoint=0;
            calibrationMode=false;
            calibrated=true;
        }
    }
}

void testApp::touchMoved( ofxBlob& b ) {
    //cout << "blobMoved() - id:" << id << " order:" << order << endl;
  // full access to blob object ( get a reference)
//    ofxKinectTrackedBlob blob = blobTracker.getById( id );
 //  cout << "volume: " << blob.volume << endl;
}

void testApp::touchOff( ofxBlob& b ) {
   //cout << "blobOff() - id:" << id << " order:" << order << endl;
}
