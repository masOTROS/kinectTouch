#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxBlobTracker.h"

class testApp : public ofBaseApp {
public:

	void setup();
	void update();
	void draw();
	void exit();

	void keyPressed (int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    void touchOn( ofxBlob& b );
    void touchMoved( ofxBlob& b );
    void touchOff( ofxBlob& b );

    void getBackground();

	ofxKinect kinect;
    ofxBlobTracker touchTracker;

    ofFloatPixels background;
    ofImage backgroundTex;
    ofFloatPixels current;
    // for KinectBlobTracker
    ofxCvGrayscaleImage diffMask;

    float nearThreshold;
    float farThreshold;

    float touchDiffFarThreshold;
    float touchDiffNearThreshold;

	bool bDrawIDs;
	bool bLearnBackground;

	int angle;
	int numPixels;

	unsigned int minBlobPoints;
	unsigned int maxBlobPoints;
    unsigned int maxBlobs;

    unsigned int hulk;

    int timeLapse;

    bool calibrationMode;
    int calibrationPoint;
    ofPoint src[4];
    ofPoint dst[4];
    ofMatrix4x4 homography;
    bool calibrated;
};
