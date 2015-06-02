#include "ofApp.h"

const string ofApp::joints[] = { "SpineBase", "SpineMid", "Neck", "Head", "ShoulderLeft", "ElbowLeft", "WristLeft", "HandLeft", "ShoulderRight", "ElbowRight", "WristRight", "HandRight", "HipLeft", "KneeLeft", "AnkleLeft", "FootLeft", "HipRight", "KneeRight", "AnkleRight", "FootRight", "SpineShoulder", "HandTipLeft", "ThumbLeft", "HandTipRight", "ThumbRight" };

//--------------------------------------------------------------
void ofApp::setup(){
    kinect.open();
    kinect.initBodySource();
    kinect.initColorSource();
    kinect.initDepthSource();

    camera.setDistance(10);

    ofSetWindowShape(1920, 1080);
}

//--------------------------------------------------------------
void ofApp::update(){
    kinect.update();

    // mesh = kinect.getDepthSource()->getMesh(
    //  false, 
//      ofxKinectForWindows2::Source::Depth::PointCloudOptions::TextureCoordinates::ColorCamera);
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

bool ofApp::checkTracking( std::map<int, ofxKFW2::Data::Joint>::iterator &j1, 
                           std::map<int, ofxKFW2::Data::Joint>::iterator &j2, 
                           std::map<int, ofxKFW2::Data::Joint>::iterator &j3, 
                           std::map<int, ofxKFW2::Data::Joint> &jointsData) {

    if (j1 != jointsData.end() && j2 != jointsData.end() && j3 != jointsData.end()) {
        if (j1->second.getTrackingState() == 2 && j2->second.getTrackingState() == 2 && j3->second.getTrackingState() == 2) {
            return true;
        }
    }
    return false;
}

float ofApp::calcAngle ( std::map<int, ofxKFW2::Data::Joint>::iterator &j1, 
                         std::map<int, ofxKFW2::Data::Joint>::iterator &j2, 
                         std::map<int, ofxKFW2::Data::Joint>::iterator &j3 ) {

            // it works, but doesn't seem a clean solution to me...

    ofVec3f posJ1 = j1->second.getPosition();
    ofVec3f posJ2 = j2->second.getPosition();
    ofVec3f posJ3 = j3->second.getPosition();

    // calculate distances between points
    float shoulderY = posJ1.y;
    float elbowY = posJ2.y;
    float wristY = posJ3.y;

    float distSW = sqrtf(pow(posJ1.x - posJ3.x, 2.) + pow(posJ1.y - posJ3.y, 2.));
    float distSE = sqrtf(pow(posJ1.x - posJ2.x, 2.) + pow(posJ1.y - posJ2.y, 2.));
    float distEW = sqrtf(pow(posJ2.x - posJ3.x, 2.) + pow(posJ2.y - posJ3.y, 2.));
                        

    // cosine rule c^2 = a^2 + b^2 - 2ab * cos(C)
    // cos(C) = (a^2 + b^2 - c^2) / 2ab
    float angle = acosf((pow(distSE, 2.) + pow(distEW, 2.) - pow(distSW, 2.)) / (2. * distSE * distEW));
    // angle is in radians
    // rotation is over z-axis?
    cout << ofRadToDeg(angle) << endl;

    return angle;

}


void ofApp::drawJoints3D() {
    // DRAW THE JOINTS IN A SALMON COLOR
    ofVec3f pos;

    // THERE IS A MAXIMUM OF 6 BODIES TRACKED BY KINECT
    for (int i = 0; i<6; i++){

        // IF THE BODY IS BEING TRACKED...
        if (this->kinect.getBodySource()->getBodies()[i].tracked){

            auto b = this->kinect.getBodySource()->getBodies()[i];
            
            std::map<JointType, ofxKFW2::Data::Joint>::iterator it;
            std::map<int, ofxKFW2::Data::Joint> jointsData;
            // ITERATE THROUGH ALL JOINTS IN THE TRACKED BODY...
            for (it = b.joints.begin(); it != b.joints.end(); ++it) {
                
                jointsData.insert(std::pair<int, ofxKFW2::Data::Joint>(it->first, it->second));
                
                if (it->second.getTrackingState() == TrackingState_Tracked) {

                    // GRAB THE JOINT'S 3D POSITION
                    pos = it->second.getPosition();
                    // draw box on position
                    // give every body diff joint color
                    ofSetColor(ofColor::fromHsb(i*40, 255, 255));
                    ofBox(pos.x, pos.y, pos.z, .05, .05, .05);

                    ofSetColor(255);
                    ofDrawBitmapString(joints[it->first], pos.x, pos.y, pos.z);

                }

            }

            std::map<int, ofxKFW2::Data::Joint>::iterator jointElbowRight = jointsData.find(ElbowRight);
            auto jointShoulderRight = jointsData.find(ShoulderRight);
            auto jointWristRight = jointsData.find(WristRight);

            if (checkTracking(jointShoulderRight, jointElbowRight, jointWristRight, jointsData)) {
               float elbowAngle = calcAngle(jointShoulderRight, jointElbowRight, jointWristRight);
               cout << "angle elbow: " << elbowAngle << endl;
            }
            

            /*
            if (jointElbowRight != jointsData.end() && jointElbowRight != jointsData.end() && jointWristRight != jointsData.end()) {
                if (jointElbowRight->second.getTrackingState() == 2 && jointShoulderRight->second.getTrackingState() == 2 && jointWristRight->second.getTrackingState() == 2) {
                    // it works, but doesn't seem a clean solution to me...

                    ofVec3f posShoulder = jointShoulderRight->second.getPosition();
                    ofVec3f posElbow = jointElbowRight->second.getPosition();
                    ofVec3f posWrist = jointWristRight->second.getPosition();

                    // calculate distances between points
                    float shoulderY = posShoulder.y;
                    float wristY = posWrist.y;
                    float elbowY = posElbow.y;

                    float distSW = sqrtf(pow(posShoulder.x - posWrist.x, 2.) + pow(posShoulder.y - posWrist.y, 2.));
                    float distSE = sqrtf(pow(posShoulder.x - posElbow.x, 2.) + pow(posShoulder.y - posElbow.y, 2.));
                    float distEW = sqrtf(pow(posElbow.x - posWrist.x, 2.) + pow(posElbow.y - posWrist.y, 2.));
                        

                    // cosine rule c^2 = a^2 + b^2 - 2ab * cos(C)
                    // cos(C) = (a^2 + b^2 - c^2) / 2ab
                    float angle = acosf((pow(distSE, 2.) + pow(distEW, 2.) - pow(distSW, 2.)) / (2. * distSE * distEW));
                    // angle is in radians
                    // rotation is over z-axis?
                    cout << ofRadToDeg(angle) << endl;
                }
            }
            */
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
