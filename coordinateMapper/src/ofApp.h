#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
		ofxKFW2::Device		kinect;

		// YOU NEED A COORDINATE MAPPER AND AN ARRAY
		// TO STORE THE DEPTH TO COLOR SPACE VALUES
		ICoordinateMapper*	m_pCoordinateMapper;
		DepthSpacePoint		m_pDepthCoordinates[1920*1080];

		void mapJoints2D();
};
