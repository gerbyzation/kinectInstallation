#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	kinect.open();
	kinect.initBodySource();
	kinect.initColorSource();
	kinect.initDepthSource();

	camera.setDistance(10);

	ofSetWindowShape(1920, 1080);

	limbs[0] = "SpineBase"; limbs[1] = "SpineMid"; limbs[2] = "Neck"; limbs[3] = "Head"; limbs[4] = "ShoulderLeft"; limbs[5] = "ElbowLeft"; limbs[6] = "WristLeft"; limbs[7] = "HandLeft"; limbs[8] = "ShoulderRight"; limbs[9] = "ElbowRight"; limbs[10] = "WristRight"; limbs[11] = "HandRight"; limbs[12] = "HipLeft"; limbs[13] = "KneeLeft"; limbs[14] = "AnkleLeft"; limbs[15] = "FootLeft"; limbs[16] = "HipRight"; limbs[17] = "KneeRight"; limbs[18] = "AnkleRight"; limbs[19] = "FootRight"; limbs[20] = "SpineShoulder"; limbs[21] = "HandTipLieft"; limbs[22] = "ThumbLeft"; limbs[23] = "HandTipRight"; limbs[24] = "ThumbRight";
}

//--------------------------------------------------------------
void ofApp::update(){
	kinect.update();

	// mesh = kinect.getDepthSource()->getMesh(
	//	false, 
//		ofxKinectForWindows2::Source::Depth::PointCloudOptions::TextureCoordinates::ColorCamera);
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(75, 95, 115);

	ofSetColor(255);
	kinect.getColorSource()->draw(0, 0, 320, 180);

	camera.begin();

	ofPushStyle();
		ofScale(10, 10, -10);

		// mesh.draw();

		drawJoints3D();
	ofPopStyle();

	camera.end();

	ofDrawBitmapString(ofToString(ofGetFrameRate()), ofGetWidth() - 100, 50);
}

void ofApp::drawJoints3D() {
	// DRAW THE JOINTS IN A SALMON COLOR
	ofVec3f pos;


	string location;
	// THERE IS A MAXIMUM OF 6 BODIES TRACKED BY KINECT
	for (int i = 0; i<6; i++){

		// give every body diff joint color


		// IF THE BODY IS BEING TRACKED...
		if (this->kinect.getBodySource()->getBodies()[i].tracked){

			auto b = this->kinect.getBodySource()->getBodies()[i];
			
			std::map<JointType, ofxKFW2::Data::Joint>::iterator it;
			// ITERATE THROUGH ALL JOINTS IN THE TRACKED BODY...
			for (it = b.joints.begin(); it != b.joints.end(); ++it)
			{
				if (it->second.getTrackingState() == TrackingState_Tracked)
				{
					// GRAB THE JOINT'S 3D POSITION
					pos = it->second.getPosition();
					// draw box on position
					ofSetColor(ofColor::fromHsb(i*40, 255, 255));
					ofBox(pos.x, pos.y, pos.z, .05, .05, .05);

					ofSetColor(255);
					ofDrawBitmapString(limbs[it->first], pos.x, pos.y, pos.z);
				}

			}

		}
	}
	ofSetColor(255);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
