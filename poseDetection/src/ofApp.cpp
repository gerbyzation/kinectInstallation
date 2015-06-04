#include "ofApp.h"

const string ofApp::joints[] = { "SpineBase", "SpineMid", "Neck", "Head", "ShoulderLeft", "ElbowLeft", "WristLeft", "HandLeft", "ShoulderRight", "ElbowRight", "WristRight", "HandRight", "HipLeft", "KneeLeft", "AnkleLeft", "FootLeft", "HipRight", "KneeRight", "AnkleRight", "FootRight", "SpineShoulder", "HandTipLeft", "ThumbLeft", "HandTipRight", "ThumbRight" };

//--------------------------------------------------------------
void ofApp::setup(){

    sender.setup("127.0.0.1", 8888);

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
    jointCalcParams.insert( make_pair("zShoulderRight", zShoulderR) );
    jointCalcParams.insert( make_pair("zShoulderLeft", zShoulderL) );

    Pose pose1(180.0, 180.0, 100.0, 100.0);
    Pose pose2(180.0, 180.0, 17.0, 100.0);

    // leave 0 empty
    poses.insert( make_pair(1, pose1) );
    poses.insert( make_pair(2, pose2) );

}

//--------------------------------------------------------------
void ofApp::update(){
    
    activePoses.clear();
    bodies.clear();

    kinect.update();

    // 1. create tracked bodies/joints array
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
                
            }

            bodies.push_back(jointsData);
        }
    }

    // 2. calculate angles
    for (vector< map<int, ofxKFW2::Data::Joint> >::iterator i = bodies.begin(); i != bodies.end(); i++) {

        map<string, float> jointAngles;
        for (map<string, CalcParams>::iterator paramsIterator = jointCalcParams.begin(); paramsIterator != jointCalcParams.end(); paramsIterator++) {

            map<int, ofxKFW2::Data::Joint> b = *i;
            auto j1 = b.find(paramsIterator->second.j[0]);
            auto j2 = b.find(paramsIterator->second.j[1]);
            auto j3 = b.find(paramsIterator->second.j[2]);

            if ( checkTracking(j1, j2, j3, b) ) {
                float angle = calcAngle( j1, j2, j3 );
                jointAngles.insert( pair<string, float>(paramsIterator->first, angle) );
            }
        }

        jointAngleArray.push_back(jointAngles);     

        // 3. check for poses
        // check for angles matching poses
        for (map<int, Pose>::iterator i = poses.begin(); i != poses.end(); i++) {
            Pose pose = i->second;
            float elTar = pose.ElbowLeft;
            float erTar = pose.ElbowRight;
            float zslTar = pose.zShoulderLeft;
            float zsrTar = pose.zShoulderRight;

            // get angles or if not found to -1
            float elA = (jointAngles.find("elbowLeft") != jointAngles.end()) ? jointAngles.find("elbowLeft")->second : -1.0;
            float erA = (jointAngles.find("elbowRight") != jointAngles.end()) ? jointAngles.find("elbowRight")->second : -1.0;
            float zslA = (jointAngles.find("zShoulderLeft") != jointAngles.end()) ? jointAngles.find("zShoulderLeft")->second : -1.0;
            float zsrA = (jointAngles.find("zShoulderRight") != jointAngles.end()) ? jointAngles.find("zShoulderRight")->second : -1.0;

            float elDif = abs( elTar - elA );
            float erDif = abs( erTar - erA );
            float zslDif = abs( zslTar - zslA );
            float zsrDif = abs( zsrTar - zsrA );

            // tolerance value
            float tol = 15.0;

            cout << elA << endl;

            if ( elDif < tol && elA != -1.0 && erDif < tol && erA != -1.0 &&
                 zslDif < tol && zslA != -1.0 && zsrDif < 10.0 && zsrA != -1.0 ) {
                sendOscMessage(97 + i->first);
                // cout << "pose " << i->first << endl;
                activePoses.push_back(i->first);
            } else {
                activePoses.push_back(-1.0);
                // cout << "pose -1" << endl;
            }
        }
    }

    // mesh = kinect.getDepthSource()->getMesh(
    //  false, hh
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

    for (int i = 0; i < jointAngleArray.size(); i++) {
        // debug stuff
        if (jointAngleArray[i].size() == 4) {
            map<string, float>::iterator er = jointAngleArray[i].find("elbowRight");

            ofDrawBitmapString(er->first + " " + ofToString(er->second), ofGetWidth() - 200, ofGetHeight() - 200);
            /*
            map<string, float>::iterator el = jointAngleArray[i].find("elbowLeft");
            ofDrawBitmapString(el->first + " " + ofToString(el->second), ofGetWidth()/4, 100 + 100*i + 20);

            map<string, float>::iterator zsr = jointAngleArray[i].find("zShoulderRight");
            ofDrawBitmapString(zsr->first + " " + ofToString(zsr->second), ofGetWidth()/4, 100 + 100*i + 30);

            map<string, float>::iterator zsl = jointAngleArray[i].find("zShoulderLeft");
            ofDrawBitmapString(zsl->first + " " + ofToString(zsl->second), ofGetWidth()/4, 100 + 100*i + 40);
            */
        }   
    }  
}

// --------------------------------------------------------------
void ofApp::drawJoints3D() { 
    // DRAW THE JOINTS IN A SALMON COLOR
    ofVec3f pos;

    // THERE IS A MAXIMUM OF 6 BODIES TRACKED BY KINECT
    for (vector< map<int, ofxKFW2::Data::Joint> >::iterator i = bodies.begin(); i !=bodies.end(); i++){

        map<int, ofxKFW2::Data::Joint> b = *i;
        // IF THE BODY IS BEING TRACKED...
        for (map<int, ofxKFW2::Data::Joint>::iterator it = b.begin(); it != b.end(); ++it) {
                
            if (it->second.getTrackingState() == TrackingState_Tracked) {

                // GRAB THE JOINT'S 3D POSITION
                pos = it->second.getPosition();
                // draw box on position
                // give every body diff joint color
                ofSetColor(ofColor::fromHsb( (i-bodies.begin()) * 40, 255, 255));
                ofBox(pos.x, pos.y, pos.z, .05, .05, .05);

                ofSetColor(255);
                ofDrawBitmapString(joints[it->first], pos.x, pos.y, pos.z);

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
    float angle = ofRadToDeg( acosf( ( pow(distSE, 2. ) + pow( distEW, 2. ) - pow( distSW, 2. ) ) / ( 2. * distSE * distEW ) ) );
    // angle is in radians

    return angle;

}

void ofApp::sendOscMessage(char c) {
    ofxOscMessage m;
    m.setAddress(ofToString(c));
    sender.sendMessage(m);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    sendOscMessage(key);
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
