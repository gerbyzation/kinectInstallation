#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include <map>

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
		
		ofxKFW2::Device kinect;

		ofEasyCam		camera;
		ofMesh			mesh;

		void			drawJoints3D();
		
		static const string joints[];	

		enum Joints
		{
			SpineBase,
			SpineMid,
			Neck,
			Head,
			ShoulderLeft,
			ElbowLeft,
			WristLeft,
			HandLeft,
			ShoulderRight,
			ElbowRight,
			WristRight,
			HandRight,
			HipLeft,
			KneeLeft,
			AnkleLeft,
			FootLeft,
			HipRight,
			KneeRight,
			AnkleRight,
			FootRight,
			SpineShoulder, 
			HandTipLeft,
			ThumbLeft,
			HandTipRight,
			ThumbRight
		};
};
