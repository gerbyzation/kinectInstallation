#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxOSC.h"
#include <map>
#include <array>

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

    void sendOscMessage(char c);

    float calcAngle( std::map<int, ofxKFW2::Data::Joint>::iterator &j1, 
                     std::map<int, ofxKFW2::Data::Joint>::iterator &j2, 
                     std::map<int, ofxKFW2::Data::Joint>::iterator &j3 );

    bool checkTracking( std::map<int, ofxKFW2::Data::Joint>::iterator &j1, 
                        std::map<int, ofxKFW2::Data::Joint>::iterator &j2, 
                        std::map<int, ofxKFW2::Data::Joint>::iterator &j3,
                        std::map<int, ofxKFW2::Data::Joint> &jointsData );

    ofxKFW2::Device kinect;
    ofxOscSender sender;

    ofEasyCam       camera;
    ofMesh          mesh;

    void            drawJoints3D();

    struct CalcParams {
        int j[3];
    };

    vector< map<int, ofxKFW2::Data::Joint> > bodies;

    map<string, CalcParams> jointCalcParams;

    struct Pose {
        float ElbowRight;
        float ElbowLeft;
        float zShoulderRight;
        float zShoulderLeft;

        Pose(float ER, float EL, float zSR, float zSL) {
            ElbowRight = ER;
            ElbowLeft = EL;
            zShoulderRight = zSR;
            zShoulderLeft = zSL;
        }
    };
    map<int, Pose> poses;
    vector<int> activePoses;

    vector< map<string, float> > jointAngleArray;

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
