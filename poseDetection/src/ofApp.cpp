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

    CalcParams elbowR       = { ShoulderRight, ElbowRight, WristRight };
    CalcParams elbowL       = { ShoulderLeft, ElbowLeft, WristLeft };
    CalcParams zShoulderR   = { HipRight, ShoulderRight, ElbowRight };
    CalcParams zShoulderL   = { HipLeft, ShoulderLeft, ElbowLeft };

    jointCalcParams.insert( make_pair("elbowRight", elbowR) );
    jointCalcParams.insert( make_pair("elbowLeft", elbowL) );
    jointCalcParams.insert( make_pair("zShoulderR", zShoulderR) );
    jointCalcParams.insert( make_pair("zShoulderL", zShoulderL) );

}

//--------------------------------------------------------------
void ofApp::update(){
    
    // clearing saved angles
    jointAngles.clear();
    kinect.update();

    // mesh = kinect.getDepthSource()->getMesh(
    //  false, 
    //  ofxKinectForWindows2::Source::Depth::PointCloudOptions::TextureCoordinates::ColorCamera);

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

// --------------------------------------------------------------
void ofApp::drawJoints3D() {
    // DRAW THE JOINTS IN A SALMON COLOR
    ofVec3f pos;

    // THERE IS A MAXIMUM OF 6 BODIES TRACKED BY KINECT
    for (int i = 0; i<6; i++){

        // IF THE BODY IS BEING TRACKED...
        if (this->kinect.getBodySource()->getBodies()[i].tracked){

            auto b = this->kinect.getBodySource()->getBodies()[i];
            
            map<JointType, ofxKFW2::Data::Joint>::iterator it;
            map<int, ofxKFW2::Data::Joint> jointsData;

            // ITERATE THROUGH ALL JOINTS IN THE TRACKED BODY...
            for (it = b.joints.begin(); it != b.joints.end(); ++it) {
                
                jointsData.insert( pair<int, ofxKFW2::Data::Joint>(it->first, it->second));
                
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

            for (map<string, CalcParams>::iterator paramsIterator = jointCalcParams.begin(); paramsIterator != jointCalcParams.end(); paramsIterator++) {
                auto j1 = jointsData.find(paramsIterator->second.j[0]);
                auto j2 = jointsData.find(paramsIterator->second.j[1]);
                auto j3 = jointsData.find(paramsIterator->second.j[2]);

                if ( checkTracking(j1, j2, j3, jointsData) ) {
                    float angle = calcAngle( j1, j2, j3 );
                    jointAngles.insert( pair<string, float>(paramsIterator->first, angle) );
                    cout << paramsIterator->first << " " << angle << endl;
                }
            }
        }
    }
    ofSetColor(255);
}

// -----------------------------------------------------------------------------
bool ofApp::checkTracking( map<int, ofxKFW2::Data::Joint>::iterator &j1, 
                           map<int, ofxKFW2::Data::Joint>::iterator &j2, 
                           map<int, ofxKFW2::Data::Joint>::iterator &j3, 
                           map<int, ofxKFW2::Data::Joint> &jointsData) {

    if (j1 != jointsData.end() && j2 != jointsData.end() && j3 != jointsData.end()) {
        if (j1->second.getTrackingState() == 2 && j2->second.getTrackingState() == 2 && j3->second.getTrackingState() == 2) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
float ofApp::calcAngle ( map<int, ofxKFW2::Data::Joint>::iterator &j1, 
                         map<int, ofxKFW2::Data::Joint>::iterator &j2, 
                         map<int, ofxKFW2::Data::Joint>::iterator &j3 ) {

    ofVec3f posJ1 = j1->second.getPosition();
    ofVec3f posJ2 = j2->second.getPosition();
    ofVec3f posJ3 = j3->second.getPosition();

    // calculate distances between points
    float distSW = sqrtf(pow(posJ1.x - posJ3.x, 2.) + pow(posJ1.y - posJ3.y, 2.));
    float distSE = sqrtf(pow(posJ1.x - posJ2.x, 2.) + pow(posJ1.y - posJ2.y, 2.));
    float distEW = sqrtf(pow(posJ2.x - posJ3.x, 2.) + pow(posJ2.y - posJ3.y, 2.));
                        
    // cosine rule: c^2 = a^2 + b^2 - 2ab * cos(C)
    // cos(C) = (a^2 + b^2 - c^2) / 2ab
    float angle = acosf((pow(distSE, 2.) + pow(distEW, 2.) - pow(distSW, 2.)) / (2. * distSE * distEW));
    // angle is in radians

    return angle;

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
