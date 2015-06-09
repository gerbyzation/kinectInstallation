#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxOSC.h"
#include "ofxGui.h"
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

    float calcAngle( const std::map<int, ofxKFW2::Data::Joint>::iterator &j1, 
                     const std::map<int, ofxKFW2::Data::Joint>::iterator &j2, 
                     const std::map<int, ofxKFW2::Data::Joint>::iterator &j3 );

    bool checkTracking( const std::map<int, ofxKFW2::Data::Joint>::iterator &j1, 
                        const std::map<int, ofxKFW2::Data::Joint>::iterator &j2, 
                        const std::map<int, ofxKFW2::Data::Joint>::iterator &j3,
                        const std::map<int, ofxKFW2::Data::Joint> &jointsData );

    ofxKFW2::Device kinect;
    ofxOscSender sender;

    bool showGUI;
    ofxPanel gui;
    ofParameter<float> maxDistance;
    ofParameter<float> tol;

    float scanLineX;

    ICoordinateMapper* m_pCoordinateMapper;
    DepthSpacePoint    m_pDepthCoordinates[1920*1080];

    void drawJoints2D();

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
